/*-
 * Copyright (c) 2011-2012 Semihalf.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include "opt_dpaa.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/pcpu.h>
#include <sys/rman.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <machine/resource.h>
#ifdef __powerpc__
#include <machine/tlb.h>
#endif

#ifdef __aarch64__
#include <vm/vm.h>
#include <vm/pmap.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include "bman.h"
#include "qman.h"
#include "portals.h"

#ifdef QMAN_CEETM_SUPPORT
#include "qman_ceetm.h"
#endif

#define FQD_ENTRY_SIZE	64	/* matches NCSW qm.h */

extern struct dpaa_portals_softc *qp_sc;
struct qman_softc *qman_sc;

extern t_Handle qman_portal_setup(struct qman_softc *qsc);

static void
qman_exception(t_Handle app, e_QmExceptions exception)
{
	struct qman_softc *sc;
	const char *message;

	sc = app;

	switch (exception) {
	case e_QM_EX_CORENET_INITIATOR_DATA:
		message = "Initiator Data Error";
		break;
	case e_QM_EX_CORENET_TARGET_DATA:
		message = "CoreNet Target Data Error";
		break;
	case e_QM_EX_CORENET_INVALID_TARGET_TRANSACTION:
		message = "Invalid Target Transaction";
		break;
	case e_QM_EX_PFDR_THRESHOLD:
		message = "PFDR Low Watermark Interrupt";
		break;
	case e_QM_EX_PFDR_ENQUEUE_BLOCKED:
		message = "PFDR Enqueues Blocked Interrupt";
		break;
	case e_QM_EX_SINGLE_ECC:
		message = "Single Bit ECC Error Interrupt";
		break;
	case e_QM_EX_MULTI_ECC:
		message = "Multi Bit ECC Error Interrupt";
		break;
	case e_QM_EX_INVALID_COMMAND:
		message = "Invalid Command Verb Interrupt";
		break;
	case e_QM_EX_DEQUEUE_DCP:
		message = "Invalid Dequeue Direct Connect Portal Interrupt";
		break;
	case e_QM_EX_DEQUEUE_FQ:
		message = "Invalid Dequeue FQ Interrupt";
		break;
	case e_QM_EX_DEQUEUE_SOURCE:
		message = "Invalid Dequeue Source Interrupt";
		break;
	case e_QM_EX_DEQUEUE_QUEUE:
		message = "Invalid Dequeue Queue Interrupt";
		break;
	case e_QM_EX_ENQUEUE_OVERFLOW:
		message = "Invalid Enqueue Overflow Interrupt";
		break;
	case e_QM_EX_ENQUEUE_STATE:
		message = "Invalid Enqueue State Interrupt";
		break;
	case e_QM_EX_ENQUEUE_CHANNEL:
		message = "Invalid Enqueue Channel Interrupt";
		break;
	case e_QM_EX_ENQUEUE_QUEUE:
		message = "Invalid Enqueue Queue Interrupt";
		break;
	case e_QM_EX_CG_STATE_CHANGE:
		message = "CG change state notification";
		break;
	default:
		message = "Unknown error";
	}

	device_printf(sc->sc_dev, "QMan Exception: %s.\n", message);
}

/**
 * General received frame callback.
 * This is called, when user did not register his own callback for a given
 * frame queue range (fqr).
 */
e_RxStoreResponse
qman_received_frame_callback(t_Handle app, t_Handle qm_fqr, t_Handle qm_portal,
    uint32_t fqid_offset, t_DpaaFD *frame)
{
	static volatile int dflt_cb_count = 0;
	t_Handle pool;
	int n = ++dflt_cb_count;

	/*
	 * This callback fires for frames dequeued from FQs that have no
	 * registered callback (e.g., FQIDs created by FMC PCD schemes).
	 * When fqr==NULL, fqid_offset is the raw FQID, not an offset.
	 *
	 * Return the buffer to its BMan pool to prevent pool depletion.
	 */
	pool = bman_pool_for_bpid(frame->bpid);
	if (pool != NULL) {
		bman_put_buffer(pool, DPAA_FD_GET_ADDR(frame));
	} else if (n <= 50 || (n % 500) == 0) {
		printf("qman: DEFAULT CB #%d: fqr=%p fqid=%u bpid=%u "
		    "status=0x%08x len=%u addr=%p [NO POOL]\n",
		    n, qm_fqr, fqid_offset, frame->bpid,
		    DPAA_FD_GET_STATUS(frame),
		    DPAA_FD_GET_LENGTH(frame),
		    DPAA_FD_GET_ADDR(frame));
	}
	return (e_RX_STORE_RESPONSE_CONTINUE);
}

/**
 * General rejected frame callback.
 * This is called, when user did not register his own callback for a given
 * frame queue range (fqr).
 */
e_RxStoreResponse
qman_rejected_frame_callback(t_Handle app, t_Handle qm_fqr, t_Handle qm_portal,
    uint32_t fqid_offset, t_DpaaFD *frame,
    t_QmRejectedFrameInfo *qm_rejected_frame_info)
{
	static int ern_count;
	struct qman_softc *sc;

	sc = app;

	if (ern_count++ < 20)
		device_printf(sc->sc_dev,
		    "ERN: rejected frame fqid_off=%u rc=%d\n",
		    fqid_offset,
		    qm_rejected_frame_info ?
		    qm_rejected_frame_info->rejectionCode : -1);
	return (e_RX_STORE_RESPONSE_CONTINUE);
}

/*
 * Map a DT reserved-memory region for QBMan use.
 *
 * If the reserved-memory node has a 'reg' property (fixed physical address),
 * the region is already excluded from phys_avail[] by early boot code
 * (fdt_foreach_reserved_mem + no-map).  We map it with pmap_mapdev_attr()
 * as write-back cacheable — LS1046A CCI-400 provides HW coherency.
 *
 * Falls back to contigmalloc for nodes with only 'size' (no fixed address).
 *
 * Returns mapped VA, or NULL on failure.  *out_size receives the size.
 */
#ifdef __aarch64__
MALLOC_DECLARE(M_NETCOMMSW);

void *qbman_alloc_reserved_mem(device_t dev, phandle_t mem_node,
    uint32_t *out_size, const char *name);
void *
qbman_alloc_reserved_mem(device_t dev, phandle_t mem_node,
    uint32_t *out_size, const char *name)
{
	pcell_t reg[4];	/* addr_hi, addr_lo, size_hi, size_lo */
	pcell_t cells[2];
	uint64_t pa, size, alignment;
	void *va;

	/*
	 * Prefer fixed physical address from 'reg' property.
	 * DT reserved-memory with no-map + reg is excluded from phys_avail[]
	 * at early boot, so this memory is never touched by the VM system.
	 */
	if (OF_getencprop(mem_node, "reg", reg, sizeof(reg)) ==
	    sizeof(reg)) {
		pa = ((uint64_t)reg[0] << 32) | reg[1];
		size = ((uint64_t)reg[2] << 32) | reg[3];

		if (size == 0 || size > 64 * 1024 * 1024) {
			device_printf(dev, "%s: invalid reg size %#jx\n",
			    name, (uintmax_t)size);
			return (NULL);
		}

		va = pmap_mapdev_attr(pa, size, VM_MEMATTR_WRITE_BACK);
		if (va == NULL) {
			device_printf(dev,
			    "%s: pmap_mapdev_attr(%#jx, %#jx) failed\n",
			    name, (uintmax_t)pa, (uintmax_t)size);
			return (NULL);
		}

		memset(va, 0, size);

		*out_size = (uint32_t)size;
		device_printf(dev,
		    "%s: %uMB at pa=%#jx va=%p (no-map reserved)\n",
		    name, (unsigned)(size >> 20), (uintmax_t)pa, va);
		return (va);
	}

	/*
	 * Fallback: dynamic allocation from 'size' property.
	 */
	if (OF_getencprop(mem_node, "size", cells, sizeof(cells)) <= 0) {
		device_printf(dev, "%s: no 'reg' or 'size' property\n", name);
		return (NULL);
	}
	size = ((uint64_t)cells[0] << 32) | cells[1];

	if (OF_getencprop(mem_node, "alignment", cells, sizeof(cells)) <= 0)
		alignment = size;
	else
		alignment = ((uint64_t)cells[0] << 32) | cells[1];

	if (size == 0 || size > 64 * 1024 * 1024) {
		device_printf(dev, "%s: invalid size %#jx\n",
		    name, (uintmax_t)size);
		return (NULL);
	}

	va = contigmalloc(size, M_NETCOMMSW, M_WAITOK | M_ZERO,
	    0, 0xFFFFFFFFULL, alignment, 0);
	if (va == NULL) {
		device_printf(dev, "%s: contigmalloc(%#jx, align=%#jx) failed\n",
		    name, (uintmax_t)size, (uintmax_t)alignment);
		return (NULL);
	}

	*out_size = (uint32_t)size;
	device_printf(dev, "%s: %uMB at %p (contigmalloc, align=%#jx)\n",
	    name, (unsigned)(size >> 20), va, (uintmax_t)alignment);
	return (va);
}
#endif

int
qman_attach(device_t dev)
{
	struct qman_softc *sc;
	t_QmParam qp;
	t_Error error;
	t_QmRevisionInfo rev;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	qman_sc = sc;

	if (XX_MallocSmartInit() != E_OK) {
		device_printf(dev, "could not initialize smart allocator.\n");
		return (ENXIO);
	}

	sched_pin();

	/* Allocate resources */
	sc->sc_rrid = 0;
	sc->sc_rres = bus_alloc_resource(dev, SYS_RES_MEMORY,
	    &sc->sc_rrid, 0, ~0, QMAN_CCSR_SIZE, RF_ACTIVE);
	if (sc->sc_rres == NULL) {
		device_printf(dev, "could not allocate memory.\n");
		goto err;
	}

	sc->sc_irid = 0;
	sc->sc_ires = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->sc_ires == NULL) {
		device_printf(dev, "could not allocate error interrupt.\n");
		goto err;
	}

	if (qp_sc == NULL)
		goto err;

#ifndef __aarch64__
	dpaa_portal_map_registers(qp_sc);
#endif

	/* Initialize QMan */
	memset(&qp, 0, sizeof(qp));
	qp.guestId = NCSW_MASTER_ID;
	qp.baseAddress = rman_get_bushandle(sc->sc_rres);
	qp.swPortalsBaseAddress = rman_get_bushandle(qp_sc->sc_rres[0]);
	qp.liodn = 0;
	qp.totalNumOfFqids = QMAN_MAX_FQIDS;
	qp.fqdMemPartitionId = NCSW_MASTER_ID;
	qp.pfdrMemPartitionId = NCSW_MASTER_ID;
	qp.f_Exception = qman_exception;
	qp.h_App = sc;
	qp.errIrq = (uintptr_t)sc->sc_ires;
	qp.partFqidBase = QMAN_FQID_BASE;
	qp.partNumOfFqids = QMAN_MAX_FQIDS;
	qp.partCgsBase = 0;
	qp.partNumOfCgs = 0;

#ifdef __aarch64__
	/*
	 * Parse DT memory-region phandles to allocate FQD and PFDR memory
	 * from reserved-memory nodes, matching Linux's qbman_init_private_mem().
	 * DT: memory-region = <&qman_fqd &qman_pfdr>;
	 *   qman-fqd:  size=0x800000 (8MB)  → 131072 FQIDs
	 *   qman-pfdr: size=0x2000000 (32MB) → 524288 PFDRs
	 */
	{
		phandle_t node, mem_node;
		pcell_t mem_handles[2];

		node = ofw_bus_get_node(dev);
		if (node > 0 && OF_getencprop(node, "memory-region",
		    (void *)mem_handles, sizeof(mem_handles)) ==
		    sizeof(mem_handles)) {
			/* Index 0 = FQD, Index 1 = PFDR */
			mem_node = OF_node_from_xref(mem_handles[0]);
			if (mem_node > 0) {
				qp.p_FqdBase = qbman_alloc_reserved_mem(dev,
				    mem_node, &qp.fqdSize, "FQD");
				if (qp.p_FqdBase != NULL) {
					qp.totalNumOfFqids =
					    qp.fqdSize / FQD_ENTRY_SIZE;
					qp.partNumOfFqids =
					    qp.totalNumOfFqids;
				}
			}
			/* Index 1 = PFDR */
			mem_node = OF_node_from_xref(mem_handles[1]);
			if (mem_node > 0) {
				qp.p_PfdrBase =
				    qbman_alloc_reserved_mem(dev,
				    mem_node, &qp.pfdrSize, "PFDR");
			}
		} else {
			device_printf(dev,
			    "no memory-region property, using defaults\n");
		}
	}
#endif

	sc->sc_qh = QM_Config(&qp);
	if (sc->sc_qh == NULL) {
		device_printf(dev, "could not be configured\n");
		goto err;
	}

	error = QM_Init(sc->sc_qh);
	if (error != E_OK) {
		device_printf(dev, "could not be initialized\n");
		goto err;
	}

	error = QM_GetRevision(sc->sc_qh, &rev);
	if (error != E_OK) {
		device_printf(dev, "could not get QMan revision\n");
		goto err;
	}

	device_printf(dev, "Hardware version: %d.%d.\n",
	    rev.majorRev, rev.minorRev);

#ifndef __aarch64__
	qman_portal_setup(sc);
#endif

	sched_unpin();

#ifdef __aarch64__
	/*
	 * Initialize portals for all CPUs from boot CPU.
	 * ARM64: pmap_mapdev creates global kernel VA, no sched_bind needed.
	 * SDEST configured per-portal via QM_PORTAL_ConfigStash.
	 * All portals use NAPI-style interrupt handling (FILTER + taskqueue).
	 */
	for (int cpu = 0; cpu < mp_ncpus; cpu++) {
		if (qman_portal_init_cpu(sc, cpu) == NULL)
			device_printf(dev,
			    "could not setup QMan portal on CPU %d\n", cpu);
		else
			device_printf(dev,
			    "QMan portal %d initialized (SDEST=%d)\n",
			    cpu, cpu / 2);
	}

#ifdef QMAN_CEETM_SUPPORT
	/* Initialize CEETM if DT node is present */
	qman_ceetm_init();
#endif
#endif

	return (0);

err:
	sched_unpin();
	qman_detach(dev);
	return (ENXIO);
}

int
qman_detach(device_t dev)
{
	struct qman_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_qh)
		QM_Free(sc->sc_qh);

	if (sc->sc_ires != NULL)
		XX_DeallocIntr((uintptr_t)sc->sc_ires);

	if (sc->sc_ires != NULL)
		bus_release_resource(dev, SYS_RES_IRQ,
		    sc->sc_irid, sc->sc_ires);

	if (sc->sc_rres != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    sc->sc_rrid, sc->sc_rres);

	return (0);
}

int
qman_suspend(device_t dev)
{

	return (0);
}

int
qman_resume(device_t dev)
{

	return (0);
}

int
qman_shutdown(device_t dev)
{

	return (0);
}


/**
 * @group QMan API functions implementation.
 * @{
 */

t_Handle
qman_fqr_create(uint32_t fqids_num, e_QmFQChannel channel, uint8_t wq,
    bool force_fqid, uint32_t fqid_or_align, bool init_parked,
    bool hold_active, bool prefer_in_cache, bool congst_avoid_ena,
    t_Handle congst_group, int8_t overhead_accounting_len,
    uint32_t tail_drop_threshold)
{
	struct qman_softc *sc;
	t_QmFqrParams fqr;
	t_Handle fqrh, portal;

	sc = qman_sc;

	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		goto err;
	}

	fqr.h_Qm = sc->sc_qh;
	fqr.h_QmPortal = portal;
	fqr.initParked = init_parked;
	fqr.holdActive = hold_active;
	fqr.preferInCache = prefer_in_cache;

	/* We do not support stashing */
	fqr.useContextAForStash = FALSE;
	fqr.p_ContextA = 0;
	fqr.p_ContextB = 0;

	fqr.channel = channel;
	fqr.wq = wq;
	fqr.shadowMode = FALSE;
	fqr.numOfFqids = fqids_num;

	/* FQID */
	fqr.useForce = force_fqid;
	if (force_fqid) {
		fqr.qs.frcQ.fqid = fqid_or_align;
	} else {
		fqr.qs.nonFrcQs.align = fqid_or_align;
	}

	/* Congestion Avoidance */
	fqr.congestionAvoidanceEnable = congst_avoid_ena;
	if (congst_avoid_ena) {
		fqr.congestionAvoidanceParams.h_QmCg = congst_group;
		fqr.congestionAvoidanceParams.overheadAccountingLength =
		    overhead_accounting_len;
		fqr.congestionAvoidanceParams.fqTailDropThreshold =
		    tail_drop_threshold;
	} else {
		fqr.congestionAvoidanceParams.h_QmCg = 0;
		fqr.congestionAvoidanceParams.overheadAccountingLength = 0;
		fqr.congestionAvoidanceParams.fqTailDropThreshold = 0;
	}

	fqrh = QM_FQR_Create(&fqr);
	if (fqrh == NULL) {
		device_printf(sc->sc_dev, "could not create Frame Queue Range"
		    "\n");
		goto err;
	}

	if (QM_FQR_GetFqid(fqrh) < QMAN_MAX_FQIDS)
		sc->sc_fqr_cpu[QM_FQR_GetFqid(fqrh)] = PCPU_GET(cpuid);

	sched_unpin();

	return (fqrh);

err:
	sched_unpin();

	return (NULL);
}

t_Handle
qman_fqr_create_ctx(uint32_t fqids_num, e_QmFQChannel channel, uint8_t wq,
    bool force_fqid, uint32_t fqid_or_align, bool prefer_in_cache,
    t_QmContextA *p_context_a, t_QmContextB *p_context_b)
{
	struct qman_softc *sc;
	t_QmFqrParams fqr;
	t_Handle fqrh, portal;

	sc = qman_sc;

	sched_pin();

	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		goto err;
	}

	fqr.h_Qm = sc->sc_qh;
	fqr.h_QmPortal = portal;
	fqr.initParked = FALSE;
	fqr.holdActive = FALSE;
	fqr.preferInCache = prefer_in_cache;

	fqr.useContextAForStash = FALSE;
	fqr.p_ContextA = p_context_a;
	fqr.p_ContextB = p_context_b;

	fqr.channel = channel;
	fqr.wq = wq;
	fqr.shadowMode = FALSE;
	fqr.numOfFqids = fqids_num;

	fqr.useForce = force_fqid;
	if (force_fqid)
		fqr.qs.frcQ.fqid = fqid_or_align;
	else
		fqr.qs.nonFrcQs.align = fqid_or_align;

	fqr.congestionAvoidanceEnable = FALSE;
	fqr.congestionAvoidanceParams.h_QmCg = 0;
	fqr.congestionAvoidanceParams.overheadAccountingLength = 0;
	fqr.congestionAvoidanceParams.fqTailDropThreshold = 0;

	fqrh = QM_FQR_Create(&fqr);
	if (fqrh == NULL) {
		device_printf(sc->sc_dev,
		    "could not create Frame Queue Range (ctx)\n");
		goto err;
	}

	if (QM_FQR_GetFqid(fqrh) < QMAN_MAX_FQIDS)
		sc->sc_fqr_cpu[QM_FQR_GetFqid(fqrh)] = PCPU_GET(cpuid);

	sched_unpin();

	return (fqrh);

err:
	sched_unpin();

	return (NULL);
}

t_Error
qman_fqr_free(t_Handle fqr)
{
	struct qman_softc *sc;
	t_Error error;

	sc = qman_sc;
	if (QM_FQR_GetFqid(fqr) < QMAN_MAX_FQIDS) {
		thread_lock(curthread);
		sched_bind(curthread, sc->sc_fqr_cpu[QM_FQR_GetFqid(fqr)]);
		thread_unlock(curthread);
	}

	error = QM_FQR_Free(fqr);

	thread_lock(curthread);
	sched_unbind(curthread);
	thread_unlock(curthread);

	return (error);
}

t_Error
qman_fqr_register_cb(t_Handle fqr, t_QmReceivedFrameCallback *callback,
    t_Handle app)
{
	struct qman_softc *sc;
	t_Error error;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (E_NOT_SUPPORTED);
	}

	error = QM_FQR_RegisterCB(fqr, callback, app);

	sched_unpin();

	return (error);
}

t_Error
qman_fqr_enqueue(t_Handle fqr, uint32_t fqid_off, t_DpaaFD *frame)
{
	struct qman_softc *sc;
	t_Error error;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (E_NOT_SUPPORTED);
	}

	error = QM_FQR_Enqueue(fqr, portal, fqid_off, frame);

	sched_unpin();

	return (error);
}

t_Error
qman_enqueue_fqid(uint32_t fqid, t_DpaaFD *frame)
{
	struct qman_softc *sc;
	t_Error error;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (E_NOT_SUPPORTED);
	}

	error = QM_PORTAL_EnqueueFqid(portal, fqid, frame);

	sched_unpin();

	return (error);
}

uint32_t
qman_fqr_get_counter(t_Handle fqr, uint32_t fqid_off,
    e_QmFqrCounters counter)
{
	struct qman_softc *sc;
	uint32_t val;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (0);
	}

	val = QM_FQR_GetCounter(fqr, portal, fqid_off, counter);

	sched_unpin();

	return (val);
}

t_Error
qman_fqr_pull_frame(t_Handle fqr, uint32_t fqid_off, t_DpaaFD *frame)
{
	struct qman_softc *sc;
	t_Error error;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (E_NOT_SUPPORTED);
	}

	error = QM_FQR_PullFrame(fqr, portal, fqid_off, frame);

	sched_unpin();

	return (error);
}

uint32_t
qman_fqr_get_base_fqid(t_Handle fqr)
{
	struct qman_softc *sc;
	uint32_t val;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (0);
	}

	val = QM_FQR_GetFqid(fqr);

	sched_unpin();

	return (val);
}

t_Error
qman_poll(e_QmPortalPollSource source)
{
	struct qman_softc *sc;
	t_Error error;
	t_Handle portal;

	sc = qman_sc;
	sched_pin();

	/* Ensure we have got QMan port initialized */
	portal = qman_portal_setup(sc);
	if (portal == NULL) {
		device_printf(sc->sc_dev, "could not setup QMan portal\n");
		sched_unpin();
		return (E_NOT_SUPPORTED);
	}

	error = QM_Poll(sc->sc_qh, source);

	sched_unpin();

	return (error);
}

/*
 * QMan diagnostic counters — expose QM_GetCounter to dtsec driver.
 */
uint32_t
qman_get_pfdr_in_use(void)
{
	struct qman_softc *sc;

	sc = qman_sc;
	if (sc == NULL || sc->sc_qh == NULL)
		return (0);

	return (QM_GetCounter(sc->sc_qh, e_QM_COUNTERS_PFDR_IN_USE));
}

uint32_t
qman_get_sfdr_in_use(void)
{
	struct qman_softc *sc;

	sc = qman_sc;
	if (sc == NULL || sc->sc_qh == NULL)
		return (0);

	return (QM_GetCounter(sc->sc_qh, e_QM_COUNTERS_SFDR_IN_USE));
}

#ifdef __aarch64__
void
qman_ccsr_diag(void)
{
	struct qman_softc *sc;
	uint32_t err_isr, ecsr, ecir, pfdr_fpc, sfdr_in_use;
	uint32_t idle_stat, dcp0_cfg, dcp0_dlm_avg;
	uint32_t dcp_dd_ihrsr, dcp_dd_hasr;

	sc = qman_sc;
	if (sc == NULL || sc->sc_qh == NULL)
		return;

	QM_ErrorDiag(sc->sc_qh, &err_isr, &ecsr, &ecir, &pfdr_fpc,
	    &sfdr_in_use, &idle_stat, &dcp0_cfg, &dcp0_dlm_avg,
	    &dcp_dd_ihrsr, &dcp_dd_hasr);

	printf("qman: CCSR err_isr=0x%08x ecsr=0x%08x ecir=0x%08x\n",
	    err_isr, ecsr, ecir);
	if (ecir) {
		printf("qman:   ecir: %s portal %u fqid %u\n",
		    (ecir & 0x20000000) ? "DCP" : "SWP",
		    (ecir >> 24) & 0x1f,
		    ecir & 0x00ffffff);
	}
	printf("qman: pfdr_free=%u sfdr_in_use=%u idle=0x%08x\n",
	    pfdr_fpc, sfdr_in_use, idle_stat);
	printf("qman: dcp0: cfg=0x%08x dlm_avg=0x%08x\n",
	    dcp0_cfg, dcp0_dlm_avg);
	printf("qman: dcp_dd: ihrsr=0x%08x hasr=0x%08x\n",
	    dcp_dd_ihrsr, dcp_dd_hasr);
}
#endif

/** @} */
