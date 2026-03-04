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
#include <sys/proc.h>
#include <sys/pcpu.h>
#include <sys/rman.h>
#include <sys/sched.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/resource.h>
#ifdef __powerpc__
#include <machine/tlb.h>
#endif
#ifdef __aarch64__
#include <machine/pmap.h>
#endif

#include <contrib/ncsw/inc/error_ext.h>
#include <contrib/ncsw/inc/xx_ext.h>

#include "portals.h"


int
dpaa_portal_alloc_res(device_t dev, struct dpaa_portals_devinfo *di, int cpu)
{
	struct dpaa_portals_softc *sc = device_get_softc(dev);
	struct resource_list_entry *rle;
	int err;
	struct resource_list *res;

	/* Check if MallocSmart allocator is ready */
	if (XX_MallocSmartInit() != E_OK)
		return (ENXIO);

	res = &di->di_res;

	/*
	 * Allocate memory.
	 * Reserve only one pair of CE/CI virtual memory regions
	 * for all CPUs, in order to save the space.
	 */
	if (sc->sc_rres[0] == NULL) {
		/* Cache enabled area */
		rle = resource_list_find(res, SYS_RES_MEMORY, 0);
		sc->sc_rrid[0] = 0;
		device_printf(dev, "CE alloc: pa=%#lx-%#lx size=%#lx dp_pa=%#lx\n",
		    (unsigned long)(rle->start + sc->sc_dp_pa),
		    (unsigned long)(rle->end + sc->sc_dp_pa),
		    (unsigned long)rle->count,
		    (unsigned long)sc->sc_dp_pa);
		sc->sc_rres[0] = bus_alloc_resource(dev,
		    SYS_RES_MEMORY, &sc->sc_rrid[0], rle->start + sc->sc_dp_pa,
		    rle->end + sc->sc_dp_pa, rle->count, RF_ACTIVE);
		if (sc->sc_rres[0] == NULL) {
			device_printf(dev,
			    "Could not allocate cache enabled memory.\n");
			return (ENXIO);
		}
		device_printf(dev, "CE mapped: va=%#lx\n",
		    (unsigned long)rman_get_bushandle(sc->sc_rres[0]));
#ifdef __powerpc__
		tlb1_set_entry(rman_get_bushandle(sc->sc_rres[0]),
		    rle->start + sc->sc_dp_pa, rle->count, _TLB_ENTRY_MEM);
#endif
#ifdef __aarch64__
		/*
		 * ARM64: The CE portal region must be Normal Non-Cacheable.
		 * Linux maps this as MEMREMAP_WC (pgprot_writecombine =
		 * Normal-NC) on ARM64.  Stores go directly to the portal
		 * hardware without cache involvement.  dc zva works on
		 * Normal-NC (zeros written as burst), dc cvac is a no-op.
		 */
		pmap_change_attr(
		    (vm_offset_t)rman_get_bushandle(sc->sc_rres[0]),
		    rle->count, VM_MEMATTR_UNCACHEABLE);
		device_printf(dev, "CE pa_verify: va=%#lx -> pa=%#lx "
		    "(expected %#lx)\n",
		    (unsigned long)rman_get_bushandle(sc->sc_rres[0]),
		    (unsigned long)pmap_kextract(
		        (vm_offset_t)rman_get_bushandle(sc->sc_rres[0])),
		    (unsigned long)(rle->start + sc->sc_dp_pa));
#endif
		/* Cache inhibited area */
		rle = resource_list_find(res, SYS_RES_MEMORY, 1);
		sc->sc_rrid[1] = 1;
		device_printf(dev, "CI alloc: pa=%#lx-%#lx size=%#lx\n",
		    (unsigned long)(rle->start + sc->sc_dp_pa),
		    (unsigned long)(rle->end + sc->sc_dp_pa),
		    (unsigned long)rle->count);
		sc->sc_rres[1] = bus_alloc_resource(dev,
		    SYS_RES_MEMORY, &sc->sc_rrid[1], rle->start + sc->sc_dp_pa,
		    rle->end + sc->sc_dp_pa, rle->count, RF_ACTIVE);
		if (sc->sc_rres[1] == NULL) {
			device_printf(dev,
			    "Could not allocate cache inhibited memory.\n");
			bus_release_resource(dev, SYS_RES_MEMORY,
			    sc->sc_rrid[0], sc->sc_rres[0]);
			return (ENXIO);
		}
		device_printf(dev, "CI mapped: va=%#lx\n",
		    (unsigned long)rman_get_bushandle(sc->sc_rres[1]));
#ifdef __powerpc__
		tlb1_set_entry(rman_get_bushandle(sc->sc_rres[1]),
		    rle->start + sc->sc_dp_pa, rle->count, _TLB_ENTRY_IO);
#endif
		sc->sc_dp[cpu].dp_ce_va =
		    rman_get_bushandle(sc->sc_rres[0]);
		sc->sc_dp[cpu].dp_ci_va =
		    rman_get_bushandle(sc->sc_rres[1]);
		sc->sc_dp[cpu].dp_regs_mapped = 1;
	}
	/* Acquire portal's CE_PA and CI_PA */
	rle = resource_list_find(res, SYS_RES_MEMORY, 0);
	sc->sc_dp[cpu].dp_ce_pa = rle->start + sc->sc_dp_pa;
	sc->sc_dp[cpu].dp_ce_size = rle->count;
	rle = resource_list_find(res, SYS_RES_MEMORY, 1);
	sc->sc_dp[cpu].dp_ci_pa = rle->start + sc->sc_dp_pa;
	sc->sc_dp[cpu].dp_ci_size = rle->count;

	/* Allocate interrupts */
	rle = resource_list_find(res, SYS_RES_IRQ, 0);
	sc->sc_dp[cpu].dp_irid = 0;
	sc->sc_dp[cpu].dp_ires = bus_alloc_resource(dev,
	    SYS_RES_IRQ, &sc->sc_dp[cpu].dp_irid, rle->start, rle->end,
	    rle->count, RF_ACTIVE);
	/* Save interrupt number for later use */
	sc->sc_dp[cpu].dp_intr_num = rle->start;

	if (sc->sc_dp[cpu].dp_ires == NULL) {
		device_printf(dev, "Could not allocate irq.\n");
		return (ENXIO);
	}
	err = XX_PreallocAndBindIntr(dev, (uintptr_t)sc->sc_dp[cpu].dp_ires, cpu);

	if (err != E_OK) {
		device_printf(dev, "Could not prealloc and bind interrupt\n");
		bus_release_resource(dev, SYS_RES_IRQ,
		    sc->sc_dp[cpu].dp_irid, sc->sc_dp[cpu].dp_ires);
		sc->sc_dp[cpu].dp_ires = NULL;
		return (ENXIO);
	}

#if 0
	err = bus_generic_config_intr(dev, rle->start, di->di_intr_trig,
	    di->di_intr_pol);
	if (err != 0) {
		device_printf(dev, "Could not configure interrupt\n");
		bus_release_resource(dev, SYS_RES_IRQ,
		    sc->sc_dp[cpu].dp_irid, sc->sc_dp[cpu].dp_ires);
		sc->sc_dp[cpu].dp_ires = NULL;
		return (err);
	}
#endif

	return (0);
}

void
dpaa_portal_map_registers(struct dpaa_portals_softc *sc)
{
	unsigned int cpu;

	sched_pin();
	cpu = PCPU_GET(cpuid);
	if (sc->sc_dp[cpu].dp_regs_mapped)
		goto out;

#ifdef __powerpc__
	/* PowerPC: remap the shared VA to this CPU's portal PA via TLB */
	tlb1_set_entry(rman_get_bushandle(sc->sc_rres[0]),
	    sc->sc_dp[cpu].dp_ce_pa, sc->sc_dp[cpu].dp_ce_size,
	    _TLB_ENTRY_MEM);
	tlb1_set_entry(rman_get_bushandle(sc->sc_rres[1]),
	    sc->sc_dp[cpu].dp_ci_pa, sc->sc_dp[cpu].dp_ci_size,
	    _TLB_ENTRY_IO);
	sc->sc_dp[cpu].dp_ce_va = rman_get_bushandle(sc->sc_rres[0]);
	sc->sc_dp[cpu].dp_ci_va = rman_get_bushandle(sc->sc_rres[1]);
#endif
#ifdef __aarch64__
	/*
	 * ARM64: Each CPU needs its own VA→PA mapping because we can't
	 * remap a shared VA per-CPU like PowerPC does with TLB1 entries.
	 * Map each portal's CE as Normal Non-Cacheable, CI as Device.
	 */
	if (sc->sc_dp[cpu].dp_ce_pa == 0) {
		printf("dpaa_portal_map_registers: cpu %u has no portal allocated\n", cpu);
		goto out;
	}
	sc->sc_dp[cpu].dp_ce_va = (vm_offset_t)pmap_mapdev_attr(
	    sc->sc_dp[cpu].dp_ce_pa, sc->sc_dp[cpu].dp_ce_size,
	    VM_MEMATTR_UNCACHEABLE);
	sc->sc_dp[cpu].dp_ci_va = (vm_offset_t)pmap_mapdev(
	    sc->sc_dp[cpu].dp_ci_pa, sc->sc_dp[cpu].dp_ci_size);
#endif

	sc->sc_dp[cpu].dp_regs_mapped = 1;

out:
	sched_unpin();
}

#ifdef __aarch64__
/*
 * Map portal CE/CI registers for a specific CPU.
 * Unlike dpaa_portal_map_registers() which uses PCPU_GET(cpuid),
 * this takes an explicit CPU parameter so it can be called from
 * any CPU context during early boot (before smp_started=1).
 * pmap_mapdev_attr/pmap_mapdev create global kernel VA mappings.
 */
void
dpaa_portal_map_registers_cpu(struct dpaa_portals_softc *sc, int cpu)
{

	if (sc->sc_dp[cpu].dp_regs_mapped)
		return;
	if (sc->sc_dp[cpu].dp_ce_pa == 0) {
		printf("dpaa_portal_map_registers_cpu: "
		    "cpu %d has no portal allocated\n", cpu);
		return;
	}
	sc->sc_dp[cpu].dp_ce_va = (vm_offset_t)pmap_mapdev_attr(
	    sc->sc_dp[cpu].dp_ce_pa, sc->sc_dp[cpu].dp_ce_size,
	    VM_MEMATTR_UNCACHEABLE);
	sc->sc_dp[cpu].dp_ci_va = (vm_offset_t)pmap_mapdev(
	    sc->sc_dp[cpu].dp_ci_pa, sc->sc_dp[cpu].dp_ci_size);
	sc->sc_dp[cpu].dp_regs_mapped = 1;
}
#endif
