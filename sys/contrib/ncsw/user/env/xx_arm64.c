/*-
 * Copyright (c) 2011 Semihalf.
 * Copyright (c) 2026 Mono Technologies Inc.
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

/*
 * NetCommSW (ncsw) environment layer for FreeBSD ARM64.
 *
 * Adapted from xx.c (PowerPC version) by Semihalf.  Key differences:
 *
 * 1. No ccsrbar_va/pa (ARM64 uses FDT-based bus_space mappings)
 * 2. No law_enable (ARM64 doesn't have Local Access Windows)
 * 3. No powerpc_intr_mask/unmask (GIC handles interrupt masking)
 * 4. No TLB1 management (ARM64 uses page tables)
 * 5. XX_VirtToPhys/XX_PhysToVirt use DMAP exclusively
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/sched.h>
#include <machine/stdarg.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_page.h>

#include <machine/cpufunc.h>
#include <machine/pmap.h>

#include <dev/dpaa/bman.h>
#include <dev/dpaa/qman.h>
#include <dev/dpaa/portals.h>

#include "error_ext.h"
#include "std_ext.h"
#include "list_ext.h"
#include "mm_ext.h"

/* Malloc Pool for NetCommSW */
MALLOC_DEFINE(M_NETCOMMSW, "NetCommSW", "NetCommSW software stack");
MALLOC_DEFINE(M_NETCOMMSW_MT, "NetCommSWTrack",
    "NetCommSW software allocation tracker");

static struct mtx XX_MallocTrackLock;
MTX_SYSINIT(XX_MallocTrackLockInit, &XX_MallocTrackLock,
    "NetCommSW MallocTrack Lock", MTX_DEF);

/* Interrupt info */
#define XX_INTR_FLAG_PREALLOCATED	(1 << 0)

/*
 * ARM64 GIC can have many more interrupts than PowerPC (INTR_VECTORS=256).
 * DPAA1 only uses a handful; 512 is plenty.
 */
#define	XX_MAX_INTR	512

struct XX_IntrInfo {
	driver_intr_t	*handler;
	void		*arg;
	int		cpu;
	int		flags;
	void		*cookie;
	/* NAPI mode: non-NULL napi_tq enables FILTER handler */
	struct taskqueue *napi_tq;
	struct task	*napi_task;
	t_Handle	napi_portal;
};

static struct XX_IntrInfo XX_IntrInfo[XX_MAX_INTR];

/* Portal type identifiers */
enum XX_PortalIdent{
	BM_PORTAL = 0,
	QM_PORTAL,
};

/* Structure to store portals' properties */
struct XX_PortalInfo {
	vm_paddr_t	portal_ce_pa[2][MAXCPU];
	vm_paddr_t	portal_ci_pa[2][MAXCPU];
	uint32_t	portal_ce_size[2][MAXCPU];
	uint32_t	portal_ci_size[2][MAXCPU];
	vm_offset_t	portal_ce_va[2];
	vm_offset_t	portal_ci_va[2];
	uintptr_t	portal_intr[2][MAXCPU];
};

static struct XX_PortalInfo XX_PInfo;

void
XX_Exit(int status)
{

	panic("NetCommSW: Exit called with status %i", status);
}

void
XX_Print(char *str, ...)
{
	va_list ap;

	va_start(ap, str);
	vprintf(str, ap);
	va_end(ap);
}

void *
XX_Malloc(uint32_t size)
{
	void *p = (malloc(size, M_NETCOMMSW, M_NOWAIT));

	return (p);
}

/*
 * XX_MallocSmartInit — no-op, kept for API compatibility.
 * Previously allocated a fixed contigmalloc pool; now XX_MallocSmart
 * delegates to XX_Malloc (kernel heap) with manual alignment, matching
 * the Linux NCSW implementation.
 */
int
XX_MallocSmartInit(void)
{

	return (E_OK);
}

/*
 * XX_MallocSmart — aligned allocation via kernel heap.
 *
 * Matches Linux NCSW xx_arm_linux.c: allocate size + alignment + header,
 * align the returned pointer, store the original malloc pointer in the
 * word immediately before it so XX_FreeSmart can recover it.
 */
void *
XX_MallocSmart(uint32_t size, int memPartitionId, uint32_t alignment)
{
	uintptr_t *aligned, raw;

	if (alignment < sizeof(uintptr_t))
		alignment = sizeof(uintptr_t);

	size += alignment + sizeof(uintptr_t);
	raw = (uintptr_t)XX_Malloc(size);
	if (raw == 0)
		return (NULL);

	aligned = (uintptr_t *)((raw + alignment + sizeof(uintptr_t))
	    & ~((uintptr_t)alignment - 1));
	*(aligned - 1) = raw;

	return ((void *)aligned);
}

void
XX_FreeSmart(void *p)
{

	if (p != NULL)
		XX_Free((void *)(*(((uintptr_t *)p) - 1)));
}

void
XX_Free(void *p)
{

	free(p, M_NETCOMMSW);
}

uint32_t
XX_DisableAllIntr(void)
{

	return (intr_disable());
}

void
XX_RestoreAllIntr(uint32_t flags)
{

	intr_restore(flags);
}

static bool
XX_IsPortalIntr(uintptr_t irq)
{
	int cpu, type;
	/* Check interrupt numbers of all available portals */
	for (type = 0; type < 2; type++)
		for (cpu = 0; cpu < MAXCPU; cpu++)
			if (irq == XX_PInfo.portal_intr[type][cpu])
				return (1);

	return (0);
}

static void
XX_Dispatch(void *arg)
{
	struct XX_IntrInfo *info;

	info = arg;

	if (info->handler == NULL) {
		printf("%s(): IRQ handler is NULL!\n", __func__);
		return;
	}

	info->handler(info->arg);
}

/*
 * FILTER interrupt handler for NAPI-style portal processing.
 * Runs in hard IRQ context — inhibits portal interrupts at the
 * source (IIR=1) and schedules a taskqueue task for deferred
 * DQRR/MR drain.  Matches Linux's portal_isr → napi_schedule pattern.
 */
static int
XX_PortalFilter(void *arg)
{
	struct XX_IntrInfo *info = arg;

	if (__predict_false(info->napi_portal == NULL))
		return (FILTER_STRAY);

	QM_PORTAL_Inhibit(info->napi_portal);
	taskqueue_enqueue(info->napi_tq, info->napi_task);

	return (FILTER_HANDLED);
}

/*
 * Configure NAPI mode for a portal interrupt.  Must be called
 * before QM_PORTAL_Init (which calls XX_SetIntr internally).
 */
void XX_ConfigPortalNapi(uintptr_t irq, struct taskqueue *tq, struct task *task);
void XX_SetPortalNapiHandle(uintptr_t irq, t_Handle portal);
void
XX_ConfigPortalNapi(uintptr_t irq, struct taskqueue *tq, struct task *task)
{
	struct resource *r;
	unsigned int inum;

	r = (struct resource *)irq;
	inum = rman_get_start(r);

	if (inum >= XX_MAX_INTR)
		return;

	XX_IntrInfo[inum].napi_tq = tq;
	XX_IntrInfo[inum].napi_task = task;
	XX_IntrInfo[inum].napi_portal = NULL;
}

/*
 * Set the portal handle for a NAPI-configured interrupt.
 * Called after QM_PORTAL_Init returns the portal handle.
 */
void
XX_SetPortalNapiHandle(uintptr_t irq, t_Handle portal)
{
	struct resource *r;
	unsigned int inum;

	r = (struct resource *)irq;
	inum = rman_get_start(r);

	if (inum >= XX_MAX_INTR)
		return;

	XX_IntrInfo[inum].napi_portal = portal;
}

t_Error
XX_PreallocAndBindIntr(device_t dev, uintptr_t irq, unsigned int cpu)
{
	struct resource *r;
	unsigned int inum;

	r = (struct resource *)irq;
	inum = rman_get_start(r);

	if (inum >= XX_MAX_INTR) {
		printf("NetCommSW: IRQ %u exceeds XX_MAX_INTR (%d)\n",
		    inum, XX_MAX_INTR);
		return (E_FULL);
	}

	/*
	 * Don't call bus_setup_intr here — just record the CPU binding.
	 * The OS interrupt handler will be registered later when the
	 * portal driver calls XX_SetIntr with a real handler.  This
	 * prevents interrupt storms from stale portal state on warm
	 * reboot (portal hardware may assert IRQ before the driver
	 * is ready to clear it).
	 */
	XX_IntrInfo[inum].flags = XX_INTR_FLAG_PREALLOCATED;
	XX_IntrInfo[inum].cpu = cpu;

	return (E_OK);
}

t_Error
XX_DeallocIntr(uintptr_t irq)
{
	struct resource *r;
	unsigned int inum;

	r = (struct resource *)irq;
	inum = rman_get_start(r);

	if (inum >= XX_MAX_INTR)
		return (E_INVALID_STATE);

	if ((XX_IntrInfo[inum].flags & XX_INTR_FLAG_PREALLOCATED) == 0)
		return (E_INVALID_STATE);

	XX_IntrInfo[inum].flags = 0;
	XX_IntrInfo[inum].handler = NULL;
	XX_IntrInfo[inum].arg = NULL;

	/* Tear down OS handler only if it was registered */
	if (XX_IntrInfo[inum].cookie != NULL)
		return (XX_FreeIntr(irq));

	return (E_OK);
}

t_Error
XX_SetIntr(uintptr_t irq, t_Isr *f_Isr, t_Handle handle)
{
	device_t dev;
	struct resource *r;
	unsigned int flags;
	int err;

	r = (struct resource *)irq;
	dev = rman_get_device(r);
	irq = rman_get_start(r);

	if (irq >= XX_MAX_INTR)
		return (E_FULL);

	/* Handle preallocated interrupts */
	if (XX_IntrInfo[irq].flags & XX_INTR_FLAG_PREALLOCATED) {
		if (XX_IntrInfo[irq].handler != NULL)
			return (E_BUSY);

		XX_IntrInfo[irq].handler = f_Isr;
		XX_IntrInfo[irq].arg = handle;

		/* Register OS handler now that portal driver is ready */
		if (XX_IntrInfo[irq].cookie == NULL) {
			unsigned int iflags;
			int err;

			iflags = INTR_TYPE_NET | INTR_MPSAFE;
			if (XX_IsPortalIntr(irq))
				iflags |= INTR_EXCL;

			if (XX_IntrInfo[irq].napi_tq != NULL) {
				/* NAPI mode: FILTER-only, no ithread */
				err = bus_setup_intr(dev, r, iflags,
				    XX_PortalFilter, NULL,
				    &XX_IntrInfo[irq],
				    &XX_IntrInfo[irq].cookie);
			} else {
				/* Legacy ithread mode */
				err = bus_setup_intr(dev, r, iflags, NULL,
				    XX_Dispatch, &XX_IntrInfo[irq],
				    &XX_IntrInfo[irq].cookie);
			}
			if (err)
				return (E_NOT_AVAILABLE);

			bus_bind_intr(dev, r, XX_IntrInfo[irq].cpu);
		}

		return (E_OK);
	}

	flags = INTR_TYPE_NET | INTR_MPSAFE;

	/* BMAN/QMAN Portal interrupts must be exclusive */
	if (XX_IsPortalIntr(irq))
		flags |= INTR_EXCL;

	err = bus_setup_intr(dev, r, flags, NULL, f_Isr, handle,
		    &XX_IntrInfo[irq].cookie);

	return (err);
}

t_Error
XX_FreeIntr(uintptr_t irq)
{
	device_t dev;
	struct resource *r;

	r = (struct resource *)irq;
	dev = rman_get_device(r);
	irq = rman_get_start(r);

	if (irq >= XX_MAX_INTR)
		return (E_INVALID_STATE);

	/* Handle preallocated interrupts */
	if (XX_IntrInfo[irq].flags & XX_INTR_FLAG_PREALLOCATED) {
		if (XX_IntrInfo[irq].handler == NULL)
			return (E_INVALID_STATE);

		XX_IntrInfo[irq].handler = NULL;
		XX_IntrInfo[irq].arg = NULL;

		return (E_OK);
	}

	return (bus_teardown_intr(dev, r, XX_IntrInfo[irq].cookie));
}

t_Error
XX_EnableIntr(uintptr_t irq)
{
	/*
	 * On ARM64 with GIC, interrupt masking/unmasking is handled
	 * by the interrupt framework.  Portal interrupts are set up
	 * with INTR_EXCL; the GIC re-enables them after handler return.
	 */
	(void)irq;
	return (E_OK);
}

t_Error
XX_DisableIntr(uintptr_t irq)
{
	/*
	 * See XX_EnableIntr — no explicit mask needed on ARM64.
	 */
	(void)irq;
	return (E_OK);
}

t_TaskletHandle
XX_InitTasklet (void (*routine)(void *), void *data)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (NULL);
}


void
XX_FreeTasklet (t_TaskletHandle h_Tasklet)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
}

int
XX_ScheduleTask(t_TaskletHandle h_Tasklet, int immediate)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (0);
}

void
XX_FlushScheduledTasks(void)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
}

int
XX_TaskletIsQueued(t_TaskletHandle h_Tasklet)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (0);
}

void
XX_SetTaskletData(t_TaskletHandle h_Tasklet, t_Handle data)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
}

t_Handle
XX_GetTaskletData(t_TaskletHandle h_Tasklet)
{
	/* Not referenced */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (NULL);
}

t_Handle
XX_InitSpinlock(void)
{
	struct mtx *m;

	m = malloc(sizeof(*m), M_NETCOMMSW, M_NOWAIT | M_ZERO);
	if (!m)
		return (0);

	mtx_init(m, "NetCommSW Lock", NULL, MTX_DEF | MTX_DUPOK);

	return (m);
}

void
XX_FreeSpinlock(t_Handle h_Spinlock)
{
	struct mtx *m;

	m = h_Spinlock;

	mtx_destroy(m);
	free(m, M_NETCOMMSW);
}

void
XX_LockSpinlock(t_Handle h_Spinlock)
{
	struct mtx *m;

	m = h_Spinlock;
	mtx_lock(m);
}

void
XX_UnlockSpinlock(t_Handle h_Spinlock)
{
	struct mtx *m;

	m = h_Spinlock;
	mtx_unlock(m);
}

uint32_t
XX_LockIntrSpinlock(t_Handle h_Spinlock)
{

	XX_LockSpinlock(h_Spinlock);
	return (0);
}

void
XX_UnlockIntrSpinlock(t_Handle h_Spinlock, uint32_t intrFlags)
{

	XX_UnlockSpinlock(h_Spinlock);
}

uint32_t
XX_Sleep(uint32_t msecs)
{

	XX_UDelay(1000 * msecs);
	return (0);
}

void
XX_UDelay(uint32_t usecs)
{
	DELAY(usecs);
}

t_Error
XX_IpcRegisterMsgHandler(char addr[XX_IPC_MAX_ADDR_NAME_LENGTH],
    t_IpcMsgHandler *f_MsgHandler, t_Handle  h_Module, uint32_t replyLength)
{

	/*
	 * This function returns fake E_OK status and does nothing
	 * as NetCommSW IPC is not used by FreeBSD drivers.
	 */
	return (E_OK);
}

t_Error
XX_IpcUnregisterMsgHandler(char addr[XX_IPC_MAX_ADDR_NAME_LENGTH])
{
	/*
	 * This function returns fake E_OK status and does nothing
	 * as NetCommSW IPC is not used by FreeBSD drivers.
	 */
	return (E_OK);
}


t_Error
XX_IpcSendMessage(t_Handle h_Session,
    uint8_t *p_Msg, uint32_t msgLength, uint8_t *p_Reply,
    uint32_t *p_ReplyLength, t_IpcMsgCompletion *f_Completion, t_Handle h_Arg)
{

	/* Should not be called */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (E_OK);
}

t_Handle
XX_IpcInitSession(char destAddr[XX_IPC_MAX_ADDR_NAME_LENGTH],
    char srcAddr[XX_IPC_MAX_ADDR_NAME_LENGTH])
{

	/* Should not be called */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (NULL);
}

t_Error
XX_IpcFreeSession(t_Handle h_Session)
{

	/* Should not be called */
	printf("NetCommSW: Unimplemented function %s() called!\n", __func__);
	return (E_OK);
}

physAddress_t
XX_VirtToPhys(void *addr)
{
	vm_paddr_t paddr;
	int cpu;

	cpu = PCPU_GET(cpuid);

	/* Handle NULL address */
	if (addr == NULL)
		return (-1);

	/*
	 * No CCSR check needed on ARM64 — registers are mapped via
	 * bus_space, not through a global ccsrbar mapping.
	 */

	/* Handle BMAN mappings */
	if (((vm_offset_t)addr >= XX_PInfo.portal_ce_va[BM_PORTAL]) &&
	    ((vm_offset_t)addr < XX_PInfo.portal_ce_va[BM_PORTAL] +
	    XX_PInfo.portal_ce_size[BM_PORTAL][cpu]))
		return (XX_PInfo.portal_ce_pa[BM_PORTAL][cpu] +
		    (vm_offset_t)addr - XX_PInfo.portal_ce_va[BM_PORTAL]);

	if (((vm_offset_t)addr >= XX_PInfo.portal_ci_va[BM_PORTAL]) &&
	    ((vm_offset_t)addr < XX_PInfo.portal_ci_va[BM_PORTAL] +
	    XX_PInfo.portal_ci_size[BM_PORTAL][cpu]))
		return (XX_PInfo.portal_ci_pa[BM_PORTAL][cpu] +
		    (vm_offset_t)addr - XX_PInfo.portal_ci_va[BM_PORTAL]);

	/* Handle QMAN mappings */
	if (((vm_offset_t)addr >= XX_PInfo.portal_ce_va[QM_PORTAL]) &&
	    ((vm_offset_t)addr < XX_PInfo.portal_ce_va[QM_PORTAL] +
	    XX_PInfo.portal_ce_size[QM_PORTAL][cpu]))
		return (XX_PInfo.portal_ce_pa[QM_PORTAL][cpu] +
		    (vm_offset_t)addr - XX_PInfo.portal_ce_va[QM_PORTAL]);

	if (((vm_offset_t)addr >= XX_PInfo.portal_ci_va[QM_PORTAL]) &&
	    ((vm_offset_t)addr < XX_PInfo.portal_ci_va[QM_PORTAL] +
	    XX_PInfo.portal_ci_size[QM_PORTAL][cpu]))
		return (XX_PInfo.portal_ci_pa[QM_PORTAL][cpu] +
		    (vm_offset_t)addr - XX_PInfo.portal_ci_va[QM_PORTAL]);

	/* ARM64 always has DMAP */
	if ((vm_offset_t)addr >= DMAP_MIN_ADDRESS &&
	    (vm_offset_t)addr < DMAP_MAX_ADDRESS)
		return (DMAP_TO_PHYS((vm_offset_t)addr));

	paddr = pmap_kextract((vm_offset_t)addr);

	if (paddr == 0)
		printf("NetCommSW: "
		    "Unable to translate virtual address %p!\n", addr);

	return (paddr);
}

void *
XX_PhysToVirt(physAddress_t addr)
{
	int cpu;

	cpu = PCPU_GET(cpuid);

	/*
	 * No CCSR check needed on ARM64 — see XX_VirtToPhys comment.
	 */

	/* Handle BMAN mappings */
	if ((addr >= XX_PInfo.portal_ce_pa[BM_PORTAL][cpu]) &&
	    (addr < XX_PInfo.portal_ce_pa[BM_PORTAL][cpu] +
	    XX_PInfo.portal_ce_size[BM_PORTAL][cpu]))
		return ((void *)(XX_PInfo.portal_ce_va[BM_PORTAL] +
		    (vm_offset_t)(addr - XX_PInfo.portal_ce_pa[BM_PORTAL][cpu])));

	if ((addr >= XX_PInfo.portal_ci_pa[BM_PORTAL][cpu]) &&
	    (addr < XX_PInfo.portal_ci_pa[BM_PORTAL][cpu] +
	    XX_PInfo.portal_ci_size[BM_PORTAL][cpu]))
		return ((void *)(XX_PInfo.portal_ci_va[BM_PORTAL] +
		    (vm_offset_t)(addr - XX_PInfo.portal_ci_pa[BM_PORTAL][cpu])));

	/* Handle QMAN mappings */
	if ((addr >= XX_PInfo.portal_ce_pa[QM_PORTAL][cpu]) &&
	    (addr < XX_PInfo.portal_ce_pa[QM_PORTAL][cpu] +
	    XX_PInfo.portal_ce_size[QM_PORTAL][cpu]))
		return ((void *)(XX_PInfo.portal_ce_va[QM_PORTAL] +
		    (vm_offset_t)(addr - XX_PInfo.portal_ce_pa[QM_PORTAL][cpu])));

	if ((addr >= XX_PInfo.portal_ci_pa[QM_PORTAL][cpu]) &&
	    (addr < XX_PInfo.portal_ci_pa[QM_PORTAL][cpu] +
	    XX_PInfo.portal_ci_size[QM_PORTAL][cpu]))
		return ((void *)(XX_PInfo.portal_ci_va[QM_PORTAL] +
		    (vm_offset_t)(addr - XX_PInfo.portal_ci_pa[QM_PORTAL][cpu])));

	/*
	 * Reject PA outside DRAM range (catches stale FD addresses from
	 * a previous boot and PA=0 which maps below dmap_phys_base).
	 */
	if (__predict_false(!PHYS_IN_DMAP_RANGE(addr)))
		return (NULL);
	return ((void *)(uintptr_t)PHYS_TO_DMAP(addr));
}

void
XX_PortalSetInfo(device_t dev)
{
	struct dpaa_portals_softc *sc;
	const char *dev_name;
	int i, type;

	dev_name = device_get_name(dev);

	if (strcmp(dev_name, "bman-portals") == 0)
		type = BM_PORTAL;
	else if (strcmp(dev_name, "qman-portals") == 0)
		type = QM_PORTAL;
	else
		return;

	sc = device_get_softc(dev);

	for (i = 0; i < MAXCPU && sc->sc_dp[i].dp_ce_pa != 0; i++) {
		XX_PInfo.portal_ce_pa[type][i] = sc->sc_dp[i].dp_ce_pa;
		XX_PInfo.portal_ci_pa[type][i] = sc->sc_dp[i].dp_ci_pa;
		XX_PInfo.portal_ce_size[type][i] = sc->sc_dp[i].dp_ce_size;
		XX_PInfo.portal_ci_size[type][i] = sc->sc_dp[i].dp_ci_size;
		XX_PInfo.portal_intr[type][i] = sc->sc_dp[i].dp_intr_num;
	}

	XX_PInfo.portal_ce_va[type] = rman_get_bushandle(sc->sc_rres[0]);
	XX_PInfo.portal_ci_va[type] = rman_get_bushandle(sc->sc_rres[1]);
}

/*
 * Stub for dead FManv3 clock-change code in fm_ncsw.c (FM_ChangeClock).
 * fm_clk_down() is referenced but defined under #ifdef TODO_SOC_SUSPEND
 * in fm_port.c — never compiled.  g_MemacRegs is provided by memac.c.
 */
void fm_clk_down(void);
void
fm_clk_down(void)
{
	printf("NetCommSW: fm_clk_down() stub called — not implemented\n");
}
