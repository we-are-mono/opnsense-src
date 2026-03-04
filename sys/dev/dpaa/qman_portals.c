/*-
 * Copyright (c) 2012 Semihalf.
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

#include "opt_platform.h"
#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/pcpu.h>
#include <sys/sched.h>
#include <sys/smp.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>
#ifdef __powerpc__
#include <machine/tlb.h>
#include <powerpc/mpc85xx/mpc85xx.h>
#endif

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "qman.h"
#include "portals.h"

extern e_RxStoreResponse qman_received_frame_callback(t_Handle, t_Handle,
    t_Handle, uint32_t, t_DpaaFD *);
extern e_RxStoreResponse qman_rejected_frame_callback(t_Handle, t_Handle,
    t_Handle, uint32_t, t_DpaaFD *, t_QmRejectedFrameInfo *);

t_Handle qman_portal_setup(struct qman_softc *);

struct dpaa_portals_softc *qp_sc;

int
qman_portals_attach(device_t dev)
{
	qp_sc = device_get_softc(dev);

#ifdef __powerpc__
	struct dpaa_portals_softc *sc = qp_sc;
	/* Map qman portal to physical address space (PowerPC LAW) */
	if (law_enable(OCP85XX_TGTIF_QMAN, sc->sc_dp_pa, sc->sc_dp_size)) {
		qman_portals_detach(dev);
		return (ENXIO);
	}
#endif
	/* Set portal properties for XX_VirtToPhys() */
	XX_PortalSetInfo(dev);

	return (bus_generic_attach(dev));
}

int
qman_portals_detach(device_t dev)
{
	struct dpaa_portals_softc *sc;
	int i;

	qp_sc = NULL;
	sc = device_get_softc(dev);

	for (i = 0; i < ARRAY_SIZE(sc->sc_dp); i++) {
		if (sc->sc_dp[i].dp_ph != NULL) {
			thread_lock(curthread);
			sched_bind(curthread, i);
			thread_unlock(curthread);

			QM_PORTAL_Free(sc->sc_dp[i].dp_ph);

			thread_lock(curthread);
			sched_unbind(curthread);
			thread_unlock(curthread);
		}

		if (sc->sc_dp[i].dp_ires != NULL) {
			XX_DeallocIntr((uintptr_t)sc->sc_dp[i].dp_ires);
			bus_release_resource(dev, SYS_RES_IRQ,
			    sc->sc_dp[i].dp_irid, sc->sc_dp[i].dp_ires);
		}
	}
	for (i = 0; i < ARRAY_SIZE(sc->sc_rres); i++) {
		if (sc->sc_rres[i] != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY,
			    sc->sc_rrid[i],
			    sc->sc_rres[i]);
	}

	return (0);
}

t_Handle
qman_portal_setup(struct qman_softc *qsc)
{
	struct dpaa_portals_softc *sc;
	t_QmPortalParam qpp;
	unsigned int cpu;
	uintptr_t p;
	t_Handle portal;

	/* Return NULL if we're not ready or while detach */
	if (qp_sc == NULL)
		return (NULL);

	sc = qp_sc;

#ifdef __aarch64__
	/*
	 * On ARM64, all portals are pre-initialized with NAPI-style
	 * interrupt handling via qman_portal_init_cpu() in qman_attach().
	 * Just return the portal for the current CPU.
	 */
	sched_pin();
	cpu = PCPU_GET(cpuid);
	portal = sc->sc_dp[cpu].dp_ph;
	sched_unpin();
	return (portal);
#endif

	sched_pin();
	portal = NULL;
	cpu = PCPU_GET(cpuid);

	/* Check if portal is ready */
	while (atomic_cmpset_acq_ptr((uintptr_t *)&sc->sc_dp[cpu].dp_ph,
	    0, -1) == 0) {
		p = atomic_load_acq_ptr((uintptr_t *)&sc->sc_dp[cpu].dp_ph);

		/* Return if portal is already initialized */
		if (p != 0 && p != -1) {
			sched_unpin();
			return ((t_Handle)p);
		}

		/* Not inititialized and "owned" by another thread */
		sched_relinquish(curthread);
	}

	/* Map portal registers */
	dpaa_portal_map_registers(sc);


	/* Bail if portal mapping failed (e.g. no portal allocated for this CPU) */
	if (sc->sc_dp[cpu].dp_ce_va == 0 || sc->sc_dp[cpu].dp_ci_va == 0)
		goto err;

	/* Configure and initialize portal */
	qpp.ceBaseAddress = sc->sc_dp[cpu].dp_ce_va;
	qpp.ciBaseAddress = sc->sc_dp[cpu].dp_ci_va;
	qpp.h_Qm = qsc->sc_qh;
	qpp.swPortalId = cpu;
	qpp.irq = (uintptr_t)sc->sc_dp[cpu].dp_ires;
	qpp.fdLiodnOffset = 0;
	qpp.f_DfltFrame = qman_received_frame_callback;
	qpp.f_RejectedFrame = qman_rejected_frame_callback;
	qpp.h_App = qsc;

	portal = QM_PORTAL_Config(&qpp);
	if (portal == NULL)
		goto err;

	if (QM_PORTAL_Init(portal) != E_OK)
		goto err;

	if (QM_PORTAL_AddPoolChannel(portal, QMAN_COMMON_POOL_CHANNEL) != E_OK)
		goto err;

	atomic_store_rel_ptr((uintptr_t *)&sc->sc_dp[cpu].dp_ph,
	    (uintptr_t)portal);
	sched_unpin();

	return (portal);

err:
	if (portal != NULL)
		QM_PORTAL_Free(portal);

	atomic_store_rel_32((uint32_t *)&sc->sc_dp[cpu].dp_ph, 0);
	sched_unpin();

	return (NULL);
}

#ifdef __aarch64__

#include <sys/taskqueue.h>
#include <sys/epoch.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <net/if.h>
#include <net/if_var.h>

/* Per-portal NAPI state */
static struct {
	struct task	task;
	struct taskqueue *tq;
	struct mbufq	rxq;		/* deferred RX mbufs */
} qman_napi[MAXCPU];

/* Declared in xx_arm64.c */
extern void XX_ConfigPortalNapi(uintptr_t irq, struct taskqueue *tq,
    struct task *task);
extern void XX_SetPortalNapiHandle(uintptr_t irq, t_Handle portal);

/*
 * Enqueue an mbuf for deferred delivery after QM_PORTAL_Poll returns.
 * Called from the DQRR RX callback (inside NCSW_PLOCK) to avoid
 * re-entering the portal via if_input -> TCP -> TX -> QM_FQR_Enqueue.
 */
void
qman_rx_defer(struct mbuf *m)
{
	int cpu = PCPU_GET(cpuid);

	if (mbufq_enqueue(&qman_napi[cpu].rxq, m) != 0) {
		static volatile uint32_t defer_drop_cnt;
		uint32_t n = atomic_fetchadd_32(&defer_drop_cnt, 1) + 1;
		if (n <= 5)
			printf("qman: rx_defer queue full on CPU %d (#%u),"
			    " dropping\n", cpu, n);
		m_freem(m);
	}
}

/*
 * NAPI-style poll task for QMan portals (FreeBSD equivalent of Linux
 * dpaa_eth_poll).  Runs in a per-CPU taskqueue thread at PI_NET,
 * scheduled by the FILTER interrupt handler via XX_PortalFilter.
 *
 * Drains the entire DQRR (max 16 entries) and MR in one shot via
 * QM_PORTAL_Poll, then delivers deferred RX mbufs outside NCSW_PLOCK
 * to prevent nested portal enqueue from the TX path.
 */
static void
qman_portal_poll_task(void *arg, int pending)
{
	t_Handle portal = arg;
	struct mbuf *m;
	int cpu = PCPU_GET(cpuid);

	QM_PORTAL_Poll(portal, e_QM_PORTAL_POLL_SOURCE_BOTH);
	QM_PORTAL_Uninhibit(portal);

	/*
	 * Deliver deferred mbufs outside NCSW_PLOCK.
	 * M_PROTO1-tagged mbufs are WiFi TX (CDX→WiFi from dpaa_wifi) —
	 * deliver via if_transmit instead of if_input.
	 */
	while ((m = mbufq_dequeue(&qman_napi[cpu].rxq)) != NULL) {
		if (__predict_false(m->m_flags & M_PROTO1)) {
			m->m_flags &= ~M_PROTO1;
			if_transmit(m->m_pkthdr.rcvif, m);
		} else {
			if_input(m->m_pkthdr.rcvif, m);
		}
	}
}

/*
 * Initialize a QMan portal for a specific CPU with NAPI-style
 * interrupt handling.  Like Linux's qman_create_affine_portal +
 * dpaa_eth_add_channel:
 *
 *   1. Create a per-CPU taskqueue pinned to the portal's CPU
 *   2. Configure the portal interrupt for FILTER mode (via
 *      XX_ConfigPortalNapi) — the FILTER handler inhibits the
 *      portal and enqueues the taskqueue task
 *   3. Initialize the NCSW portal (Config + Stash + Init)
 *   4. Subscribe to the pool channel so QMan distributes RX
 *      frames across all portals (round-robin)
 *
 * On ARM64, pmap_mapdev creates global kernel VA, so no
 * sched_bind/migration is needed.
 */
t_Handle
qman_portal_init_cpu(struct qman_softc *qsc, int cpu)
{
	struct dpaa_portals_softc *sc;
	t_QmPortalParam qpp;
	t_QmPortalStashParam stash;
	t_Handle portal;
	cpuset_t cpumask;

	if (qp_sc == NULL)
		return (NULL);
	sc = qp_sc;

	/* Skip if already initialized */
	if (sc->sc_dp[cpu].dp_ph != NULL)
		return (sc->sc_dp[cpu].dp_ph);

	/* Map CE/CI registers (global kernel VA, accessible from any CPU) */
	dpaa_portal_map_registers_cpu(sc, cpu);

	if (sc->sc_dp[cpu].dp_ce_va == 0 || sc->sc_dp[cpu].dp_ci_va == 0)
		return (NULL);

	if (sc->sc_dp[cpu].dp_ires == NULL) {
		printf("qman: portal %d has no IRQ resource\n", cpu);
		return (NULL);
	}

	/* Init deferred RX mbuf queue (unlimited — DQRR ring is the natural cap) */
	mbufq_init(&qman_napi[cpu].rxq, 0);

	/*
	 * Create per-CPU taskqueue for NAPI-style polling.
	 * The task arg is set to the portal handle after Init.
	 */
	CPU_ZERO(&cpumask);
	CPU_SET(cpu, &cpumask);
	NET_TASK_INIT(&qman_napi[cpu].task, 0, qman_portal_poll_task, NULL);
	qman_napi[cpu].tq = taskqueue_create_fast("qman_poll", M_WAITOK,
	    taskqueue_thread_enqueue, &qman_napi[cpu].tq);
	taskqueue_start_threads_cpuset(&qman_napi[cpu].tq, 1, PI_NET,
	    &cpumask, "qman%d", cpu);

	/*
	 * Configure NAPI mode before QM_PORTAL_Init — Init calls
	 * XX_SetIntr internally, which checks for NAPI mode and
	 * registers a FILTER handler instead of an ithread handler.
	 */
	XX_ConfigPortalNapi((uintptr_t)sc->sc_dp[cpu].dp_ires,
	    qman_napi[cpu].tq, &qman_napi[cpu].task);

	/* Configure portal */
	memset(&qpp, 0, sizeof(qpp));
	qpp.ceBaseAddress = sc->sc_dp[cpu].dp_ce_va;
	qpp.ciBaseAddress = sc->sc_dp[cpu].dp_ci_va;
	qpp.h_Qm = qsc->sc_qh;
	qpp.swPortalId = cpu;
	qpp.irq = (uintptr_t)sc->sc_dp[cpu].dp_ires;
	qpp.fdLiodnOffset = 0;
	qpp.f_DfltFrame = qman_received_frame_callback;
	qpp.f_RejectedFrame = qman_rejected_frame_callback;
	qpp.h_App = qsc;

	portal = QM_PORTAL_Config(&qpp);
	if (portal == NULL)
		return (NULL);

	/*
	 * Configure stash destination (SDEST) for this CPU.
	 * Linux: qman_set_sdest(channel, cpu_idx / 2) for QMan v3+.
	 * Each pair of cores shares one Stash Request Queue (SRQ) in
	 * the CoreNet fabric.  LS1046A: SRQ0=CPU0-1, SRQ1=CPU2-3.
	 */
	memset(&stash, 0, sizeof(stash));
	stash.stashDestQueue = cpu / 2;
	QM_PORTAL_ConfigStash(portal, &stash);

	if (QM_PORTAL_Init(portal) != E_OK) {
		QM_PORTAL_Free(portal);
		return (NULL);
	}

	/*
	 * Set portal handle for the FILTER handler and task.
	 * The FILTER handler uses this to call QM_PORTAL_Inhibit.
	 * The task function receives it as the arg parameter.
	 */
	XX_SetPortalNapiHandle((uintptr_t)sc->sc_dp[cpu].dp_ires, portal);
	qman_napi[cpu].task.ta_context = portal;

	/*
	 * Subscribe to pool channel — all portals get RX frames.
	 * QMan hardware distributes frames round-robin across all
	 * portals subscribed to the pool channel, matching Linux's
	 * dpaa_eth_add_channel() which calls qman_p_static_dequeue_add
	 * for every CPU's portal.
	 *
	 * RX FQs are on per-CPU dedicated channels (RSS), so pool
	 * channel round-robin no longer causes TCP reordering.  Pool
	 * channel is still needed for TX confirm, CDX dist FQs, and
	 * CAAM QI response FQs.
	 */
	QM_PORTAL_AddPoolChannel(portal, QMAN_COMMON_POOL_CHANNEL);

	sc->sc_dp[cpu].dp_ph = portal;
	return (portal);
}

void
qman_portal_dqrr_diag(void)
{
	struct dpaa_portals_softc *sc;
	t_Handle portal;
	uint8_t sw_pi, sw_ci, sw_fill, hw_pi, hw_ci;
	uint32_t isr, ier, iir;
	int cpu;

	if (qp_sc == NULL)
		return;
	sc = qp_sc;

	for (cpu = 0; cpu < mp_ncpus; cpu++) {
		portal = sc->sc_dp[cpu].dp_ph;
		if (portal == NULL)
			continue;
		QM_PORTAL_DqrrDiag(portal, &sw_pi, &sw_ci, &sw_fill,
		    &hw_pi, &hw_ci);
		QM_PORTAL_IsrDiag(portal, &isr, &ier, &iir);
		printf("qman: portal%d: dqrr sw_pi=%u sw_ci=%u fill=%u "
		    "hw_pi=%u hw_ci=%u isr=0x%05x ier=0x%05x iir=%u\n",
		    cpu, sw_pi, sw_ci, sw_fill, hw_pi, hw_ci,
		    isr, ier, iir);
	}
}
#endif
