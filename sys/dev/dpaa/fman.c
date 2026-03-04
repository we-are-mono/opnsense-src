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
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include "opt_platform.h"
#include "opt_dpaa.h"

#include <contrib/ncsw/inc/Peripherals/fm_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_muram_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_pcd_ext.h>
#include <contrib/ncsw/inc/ncsw_ext.h>
#include <contrib/ncsw/inc/net_ext.h>
#include <contrib/ncsw/integrations/fman_ucode.h>

#include "fman.h"
#include "fman_chardev.h"
#include <dev/dpaa/qman.h>

/* NCSW internal headers — needed for hot-adding HC to existing PCD */
#include <contrib/ncsw/Peripherals/FM/Pcd/fm_pcd.h>
#include <contrib/ncsw/Peripherals/FM/inc/fm_hc.h>


static MALLOC_DEFINE(M_FMAN, "fman", "fman devices information");


/**
 * @group FMan private defines.
 * @{
 */
enum fman_irq_enum {
	FMAN_IRQ_NUM		= 0,
	FMAN_ERR_IRQ_NUM	= 1
};

enum fman_mu_ram_map {
	FMAN_MURAM_OFF		= 0x0,
#ifdef __aarch64__
	FMAN_MURAM_SIZE		= 0x60000	/* LS1046A FManv3: 384KB */
#else
	FMAN_MURAM_SIZE		= 0x28000	/* P5020 FManv2: 160KB */
#endif
};

struct fman_config {
	device_t fman_device;
	uintptr_t mem_base_addr;
	uintptr_t irq_num;
	uintptr_t err_irq_num;
	uint8_t fm_id;
	t_FmExceptionsCallback *exception_callback;
	t_FmBusErrorCallback *bus_error_callback;
};

/**
 * @group FMan private methods/members.
 * @{
 */
/**
 * Frame Manager firmware.
 * On ARM64 (LS1046A), U-Boot preloads FManv3 microcode into IRAM,
 * so we skip embedding firmware.  On PowerPC, use the P3041 image.
 */
#ifndef __aarch64__
const uint32_t fman_firmware[] = FMAN_UC_IMG;
const uint32_t fman_firmware_size = sizeof(fman_firmware);
#endif
int
fman_activate_resource(device_t bus, device_t child, int type, int rid,
    struct resource *res)
{
	struct fman_softc *sc;
	bus_space_tag_t bt;
	bus_space_handle_t bh;
	int i, rv;

	sc = device_get_softc(bus);
	if (type != SYS_RES_IRQ) {
		for (i = 0; i < sc->sc_base.nranges; i++) {
			if (rman_is_region_manager(res, &sc->rman) != 0) {
				bt = rman_get_bustag(sc->mem_res);
				rv = bus_space_subregion(bt,
				    rman_get_bushandle(sc->mem_res),
				    rman_get_start(res) -
				    rman_get_start(sc->mem_res),
				    rman_get_size(res), &bh);
				if (rv != 0)
					return (rv);
				rman_set_bustag(res, bt);
				rman_set_bushandle(res, bh);
				return (rman_activate_resource(res));
			}
		}
		return (EINVAL);
	}
	return (bus_generic_activate_resource(bus, child, type, rid, res));
}

int
fman_release_resource(device_t bus, device_t child, int type, int rid,
    struct resource *res)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;
	int passthrough, rv;

	passthrough = (device_get_parent(child) != bus);
	rl = BUS_GET_RESOURCE_LIST(bus, child);
	if (type != SYS_RES_IRQ) {
		if ((rman_get_flags(res) & RF_ACTIVE) != 0 ){
			rv = bus_deactivate_resource(child, type, rid, res);
			if (rv != 0)
				return (rv);
		}
		rv = rman_release_resource(res);
		if (rv != 0)
			return (rv);
		if (!passthrough) {
			rle = resource_list_find(rl, type, rid);
			KASSERT(rle != NULL,
			    ("%s: resource entry not found!", __func__));
			KASSERT(rle->res != NULL,
			   ("%s: resource entry is not busy", __func__));
			rle->res = NULL;
		}
		return (0);
	}
	return (resource_list_release(rl, bus, child, type, rid, res));
}

struct resource *
fman_alloc_resource(device_t bus, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct fman_softc *sc;
	struct resource_list *rl;
	struct resource_list_entry *rle = NULL;
	struct resource *res;
	int i, isdefault, passthrough;

	isdefault = RMAN_IS_DEFAULT_RANGE(start, end);
	passthrough = (device_get_parent(child) != bus);
	sc = device_get_softc(bus);
	rl = BUS_GET_RESOURCE_LIST(bus, child);
	switch (type) {
	case SYS_RES_MEMORY:
		KASSERT(!(isdefault && passthrough),
		    ("%s: passthrough of default allocation", __func__));
		if (!passthrough) {
			rle = resource_list_find(rl, type, *rid);
			if (rle == NULL)
				return (NULL);
			KASSERT(rle->res == NULL,
			    ("%s: resource entry is busy", __func__));
			if (isdefault) {
				start = rle->start;
				count = ulmax(count, rle->count);
				end = ulmax(rle->end, start + count - 1);
			}
		}

		res = NULL;
		/* Map fman ranges to nexus ranges. */
		for (i = 0; i < sc->sc_base.nranges; i++) {
			if (start >= sc->sc_base.ranges[i].bus && end <
			    sc->sc_base.ranges[i].bus + sc->sc_base.ranges[i].size) {
				start += rman_get_start(sc->mem_res);
				end += rman_get_start(sc->mem_res);
				res = rman_reserve_resource(&sc->rman, start,
				    end, count, flags & ~RF_ACTIVE, child);
				if (res == NULL)
					return (NULL);
				rman_set_rid(res, *rid);
				rman_set_type(res, type);
				if ((flags & RF_ACTIVE) != 0 && bus_activate_resource(
				    child, type, *rid, res) != 0) {
					rman_release_resource(res);
					return (NULL);
				}
				break;
			}
		}
		if (!passthrough)
			rle->res = res;
		return (res);
	case SYS_RES_IRQ:
		return (resource_list_alloc(rl, bus, child, type, rid, start,
		    end, count, flags));
	}
	return (NULL);
}

static int
fman_fill_ranges(phandle_t node, struct simplebus_softc *sc)
{
	int host_address_cells;
	cell_t *base_ranges;
	ssize_t nbase_ranges;
	int err;
	int i, j, k;

	err = OF_searchencprop(OF_parent(node), "#address-cells",
	    &host_address_cells, sizeof(host_address_cells));
	if (err <= 0)
		return (-1);

	nbase_ranges = OF_getproplen(node, "ranges");
	if (nbase_ranges < 0)
		return (-1);
	sc->nranges = nbase_ranges / sizeof(cell_t) /
	    (sc->acells + host_address_cells + sc->scells);
	if (sc->nranges == 0)
		return (0);

	sc->ranges = malloc(sc->nranges * sizeof(sc->ranges[0]),
	    M_DEVBUF, M_WAITOK);
	base_ranges = malloc(nbase_ranges, M_DEVBUF, M_WAITOK);
	OF_getencprop(node, "ranges", base_ranges, nbase_ranges);

	for (i = 0, j = 0; i < sc->nranges; i++) {
		sc->ranges[i].bus = 0;
		for (k = 0; k < sc->acells; k++) {
			sc->ranges[i].bus <<= 32;
			sc->ranges[i].bus |= base_ranges[j++];
		}
		sc->ranges[i].host = 0;
		for (k = 0; k < host_address_cells; k++) {
			sc->ranges[i].host <<= 32;
			sc->ranges[i].host |= base_ranges[j++];
		}
		sc->ranges[i].size = 0;
		for (k = 0; k < sc->scells; k++) {
			sc->ranges[i].size <<= 32;
			sc->ranges[i].size |= base_ranges[j++];
		}
	}

	free(base_ranges, M_DEVBUF);
	return (sc->nranges);
}

static void
fman_pcd_exception_callback(t_Handle app_handle,
    e_FmPcdExceptions exception)
{
	struct fman_softc *sc;

	sc = app_handle;
	device_printf(sc->sc_base.dev, "PCD exception %d occurred.\n",
	    exception);
}

static void
fman_pcd_id_exception_callback(t_Handle app_handle,
    e_FmPcdExceptions exception, uint16_t index)
{
	struct fman_softc *sc;

	sc = app_handle;
	device_printf(sc->sc_base.dev,
	    "PCD indexed exception %d, index %u.\n", exception, index);
}

/*
 * Host Command (HC) port parameters.
 *
 * The HC port uses OH port cell-index 0x2 (the first offline port),
 * which is reserved in the Mono Gateway DTS with status="disabled"
 * so dpaa_oh doesn't claim it.  FMANv3 requires portId=0 for HC.
 */
#define	FMAN_HC_OH_REG_OFFSET	0x82000	/* OH port 0x2 register offset */
#define	FMAN_HC_PORT_ID		0	/* MUST be 0 for FMANv3 */
#define	FMAN_HC_OH_CELL_INDEX	0x02	/* DT cell-index for QMan channel */
#define	FMAN_HC_LIODN_BASE	0	/* ARM64 SMMU handles LIODN */
#define	FMAN_HC_FQ_WQ		1	/* Work queue priority */

/*
 * Byte-swap every uint32_t in a buffer.
 *
 * FMan is big-endian and reads HC frame data via DMA with no hardware
 * swap (DMA_NO_SWAP).  On PPC (big-endian), native uint32_t values
 * are already in the correct byte order.  On ARM64 (little-endian),
 * every uint32_t must be byte-swapped so FMan reads correct values.
 *
 * The Linux NXP SDK does exactly this in QmEnqueueCB (before enqueue)
 * and qm_tx_conf_dqrr_cb (after confirmation) in lnxwrp_fm_port.c.
 */
static __inline void
fman_hc_bswap_frame(void *buf, uint32_t len)
{
#if _BYTE_ORDER == _LITTLE_ENDIAN
	uint32_t *p = (uint32_t *)buf;
	uint32_t i;

	for (i = 0; i < len / 4; i++)
		p[i] = __builtin_bswap32(p[i]);
#endif
}

/*
 * QMan enqueue callback for the HC port.
 * Called by NCSW HC code (hc.c EnQFrm) to send command frames
 * through QMan to the FMan HC offline port.
 */
static t_Error
fman_hc_qm_enqueue(t_Handle h_arg, void *p_fd)
{
	struct fman_softc *sc;
	t_DpaaFD *fd = (t_DpaaFD *)p_fd;

	sc = h_arg;

	/* Byte-swap HC frame data for FMan big-endian DMA */
	fman_hc_bswap_frame(DPAA_FD_GET_ADDR(fd), DPAA_FD_GET_LENGTH(fd));

	return (qman_fqr_enqueue(sc->hc_tx_fqr, 0, fd));
}

/*
 * QMan receive callback for HC confirmation frames.
 *
 * FMan sends HC command completions on the confirmation FQ.  QMan
 * delivers them here; we forward to FM_PCD_HcTxConf() which clears
 * the "enqueued" flag in hc.c, unblocking the spinning EnQFrm().
 *
 * The HC frame data was byte-swapped before enqueue (for FMan BE DMA).
 * Swap it back to native byte order so FmHcTxConf can read fields
 * like commandSequence in native format.
 */
static e_RxStoreResponse
fman_hc_conf_callback(t_Handle app, t_Handle qm_fqr, t_Handle qm_portal,
    uint32_t fqid_offset, t_DpaaFD *frame)
{
	struct fman_softc *sc;

	sc = app;

	/* Byte-swap HC frame data back to native byte order */
	fman_hc_bswap_frame(DPAA_FD_GET_ADDR(frame),
	    DPAA_FD_GET_LENGTH(frame));

	FM_PCD_HcTxConf(sc->pcd_handle, frame);
	return (e_RX_STORE_RESPONSE_CONTINUE);
}

static t_Handle
fman_init(struct fman_softc *sc, struct fman_config *cfg)
{
	phandle_t node;
	t_FmParams fm_params;
	t_Handle muram_handle, fm_handle;
	t_Error error;
	t_FmRevisionInfo revision_info;
	uint16_t clock;
	uint32_t tmp, mod;

	memset(&fm_params, 0, sizeof(fm_params));

	/* MURAM configuration */
	muram_handle = FM_MURAM_ConfigAndInit(cfg->mem_base_addr +
	    FMAN_MURAM_OFF, FMAN_MURAM_SIZE);
	if (muram_handle == NULL) {
		device_printf(cfg->fman_device, "couldn't init FM MURAM module"
		    "\n");
		return (NULL);
	}
	sc->muram_handle = muram_handle;

	/*
	 * MURAM stays as Device-nGnRnE (the default bus resource mapping),
	 * matching Linux devm_ioremap().  All MURAM accesses must use
	 * WRITE_UINT32/GET_UINT32/IOMemSet32 — never plain memset/memcpy.
	 */

	/* Fill in FM configuration */
	fm_params.fmId = cfg->fm_id;
	/* XXX we support only one partition thus each fman has master id */
	fm_params.guestId = NCSW_MASTER_ID;

	fm_params.baseAddr = cfg->mem_base_addr;
	fm_params.h_FmMuram = muram_handle;

	/* Get FMan clock in Hz */
	if ((tmp = fman_get_clock(sc)) == 0)
		return (NULL);

	/* Convert FMan clock to MHz */
	clock = (uint16_t)(tmp / 1000000);
	mod = tmp % 1000000;

	if (mod >= 500000)
		++clock;

	fm_params.fmClkFreq = clock;
	fm_params.f_Exception = cfg->exception_callback;
	fm_params.f_BusError = cfg->bus_error_callback;
	fm_params.h_App = cfg->fman_device;
	fm_params.irq = cfg->irq_num;
	fm_params.errIrq = cfg->err_irq_num;

#ifdef __aarch64__
	/* U-Boot preloads FMan microcode into IRAM; skip firmware loading */
	fm_params.firmware.size = 0;
	fm_params.firmware.p_Code = NULL;
#else
	fm_params.firmware.size = fman_firmware_size;
	fm_params.firmware.p_Code = (uint32_t*)fman_firmware;
#endif

#if (DPAA_VERSION >= 11)
	/* FManv3 Virtual Storage Profiles — master owns all 64 entries.
	 * VSP register block at FMan base + 0xdc000 (from fm_common.h FM_MM_SP). */
	fm_params.partVSPBase = 0;
	fm_params.partNumOfVSPs = FM_VSP_MAX_NUM_OF_ENTRIES;
	fm_params.vspBaseAddr = cfg->mem_base_addr + 0xdc000;
#endif

	fm_handle = FM_Config(&fm_params);
	if (fm_handle == NULL) {
		device_printf(cfg->fman_device, "couldn't configure FM "
		    "module\n");
		goto err;
	}

#ifdef __aarch64__
	/* Don't reset FMan — would wipe U-Boot's pre-loaded microcode */
	FM_ConfigResetOnInit(fm_handle, FALSE);
#else
	FM_ConfigResetOnInit(fm_handle, TRUE);
#endif

	error = FM_Init(fm_handle);
	if (error != E_OK) {
		device_printf(cfg->fman_device, "couldn't init FM module\n");
		goto err2;
	}

	error = FM_GetRevision(fm_handle, &revision_info);
	if (error != E_OK) {
		device_printf(cfg->fman_device, "couldn't get FM revision\n");
		goto err2;
	}

	device_printf(cfg->fman_device, "Hardware version: %d.%d.\n",
	    revision_info.majorRev, revision_info.minorRev);

	{
		t_FmCtrlCodeRevisionInfo ucode_rev;
		if (FM_GetFmanCtrlCodeRevision(fm_handle, &ucode_rev) == E_OK)
			device_printf(cfg->fman_device,
			    "Microcode: package %u, version %u.%u%s\n",
			    ucode_rev.packageRev,
			    ucode_rev.majorRev, ucode_rev.minorRev,
			    ucode_rev.packageRev >= 209 ? " (CDX/ASK)" :
			    ucode_rev.packageRev == 106 ? " (IP offload)" :
			    " (unknown)");
	}

	/*
	 * Initialize PCD (Parse-Classify-Distribute) with Parser +
	 * KeyGen + CC for RSS/multi-queue RX distribution and
	 * classification support.
	 *
	 * CC is enabled here so CDX can use it for hash tables.
	 * Host Command (HC) is NOT enabled here because QMan portals
	 * aren't ready during early FMan attach.  CDX calls
	 * fman_reinit_pcd_with_hc() later to hot-add HC to this PCD
	 * without rebuilding it — preserving existing dtsec schemes.
	 */
	{
		t_FmPcdParams pcd_params;
		t_FmPcdNetEnvParams netenv_params;

		sc->pcd_handle = NULL;
		sc->netenv_handle = NULL;
		sc->hc_tx_fqr = NULL;
		sc->hc_err_fqr = NULL;
		sc->hc_conf_fqr = NULL;

		memset(&pcd_params, 0, sizeof(pcd_params));
		pcd_params.h_Fm = fm_handle;
		pcd_params.prsSupport = TRUE;
		pcd_params.ccSupport = TRUE;
		pcd_params.kgSupport = TRUE;
		pcd_params.plcrSupport = TRUE;
		pcd_params.numOfSchemes = FM_PCD_KG_NUM_OF_SCHEMES;
		pcd_params.f_Exception = fman_pcd_exception_callback;
		pcd_params.f_ExceptionId = fman_pcd_id_exception_callback;
		pcd_params.h_App = sc;

		sc->pcd_handle = FM_PCD_Config(&pcd_params);
		if (sc->pcd_handle == NULL) {
			device_printf(cfg->fman_device,
			    "couldn't configure PCD\n");
			goto skip_pcd;
		}

		FM_PCD_ConfigPlcrNumOfSharedProfiles(sc->pcd_handle, 16);

		error = FM_PCD_Init(sc->pcd_handle);
		if (error != E_OK) {
			device_printf(cfg->fman_device,
			    "couldn't init PCD: %d\n", error);
			FM_PCD_Free(sc->pcd_handle);
			sc->pcd_handle = NULL;
			goto skip_pcd;
		}

		error = FM_PCD_Enable(sc->pcd_handle);
		if (error != E_OK) {
			device_printf(cfg->fman_device,
			    "couldn't enable PCD: %d\n", error);
			FM_PCD_Free(sc->pcd_handle);
			sc->pcd_handle = NULL;
			goto skip_pcd;
		}

		/*
		 * Network environment: IPv4/IPv6 + TCP/UDP distinction.
		 * This tells PCD which protocol layers to recognize.
		 */
		memset(&netenv_params, 0, sizeof(netenv_params));
		netenv_params.numOfDistinctionUnits = 2;
		netenv_params.units[0].hdrs[0].hdr = HEADER_TYPE_IPv4;
		netenv_params.units[0].hdrs[1].hdr = HEADER_TYPE_IPv6;
		netenv_params.units[1].hdrs[0].hdr = HEADER_TYPE_TCP;
		netenv_params.units[1].hdrs[1].hdr = HEADER_TYPE_UDP;

		sc->netenv_handle = FM_PCD_NetEnvCharacteristicsSet(
		    sc->pcd_handle, &netenv_params);
		if (sc->netenv_handle == NULL)
			device_printf(cfg->fman_device,
			    "couldn't set NetEnv characteristics\n");

		device_printf(cfg->fman_device,
		    "PCD initialized: Parser + KeyGen\n");
skip_pcd:
		;
	}

	/* Initialize the simplebus part of things */
	simplebus_init(sc->sc_base.dev, 0);

	node = ofw_bus_get_node(sc->sc_base.dev);
	fman_fill_ranges(node, &sc->sc_base);
	sc->rman.rm_type = RMAN_ARRAY;
	sc->rman.rm_descr = "FMan range";
	rman_init_from_resource(&sc->rman, sc->mem_res);
	for (node = OF_child(node); node > 0; node = OF_peer(node)) {
		simplebus_add_device(sc->sc_base.dev, node, 0, NULL, -1, NULL);
	}

	return (fm_handle);

err2:
	FM_Free(fm_handle);
err:
	FM_MURAM_Free(muram_handle);
	sc->muram_handle = NULL;
	return (NULL);
}

static void
fman_exception_callback(t_Handle app_handle, e_FmExceptions exception)
{
	struct fman_softc *sc;

	sc = app_handle;
	device_printf(sc->sc_base.dev, "FMan exception %d\n", exception);
}

static void
fman_error_callback(t_Handle app_handle, e_FmPortType port_type,
    uint8_t port_id, uint64_t addr, uint8_t tnum, uint16_t liodn)
{
	struct fman_softc *sc;

	sc = app_handle;
	device_printf(sc->sc_base.dev,
	    "FMan error: portType=%d portId=%u addr=0x%llx tnum=%u liodn=%u\n",
	    port_type, port_id, (unsigned long long)addr, tnum, liodn);
}

/** @} */


/**
 * @group FMan driver interface.
 * @{
 */

int
fman_get_handle(device_t dev, t_Handle *fmh)
{
	struct fman_softc *sc = device_get_softc(dev);

	*fmh = sc->fm_handle;

	return (0);
}

int
fman_get_muram_handle(device_t dev, t_Handle *muramh)
{
	struct fman_softc *sc = device_get_softc(dev);

	*muramh = sc->muram_handle;

	return (0);
}

int
fman_get_bushandle(device_t dev, vm_offset_t *fm_base)
{
	struct fman_softc *sc = device_get_softc(dev);

	*fm_base = rman_get_bushandle(sc->mem_res);

	return (0);
}

int
fman_get_pcd_handle(device_t dev, t_Handle *pcdh)
{
	struct fman_softc *sc = device_get_softc(dev);

	*pcdh = sc->pcd_handle;

	return (0);
}

int
fman_get_netenv_handle(device_t dev, t_Handle *netenvh)
{
	struct fman_softc *sc = device_get_softc(dev);

	*netenvh = sc->netenv_handle;

	return (0);
}

struct fmcd_softc *
fman_get_fmcd(device_t dev)
{
	struct fman_softc *sc = device_get_softc(dev);

	return (sc->fmcd);
}

int
fman_attach(device_t dev)
{
	struct fman_softc *sc;
	struct fman_config cfg;
	pcell_t qchan_range[2];
	phandle_t node;

	sc = device_get_softc(dev);
	sc->sc_base.dev = dev;

	/* Check if MallocSmart allocator is ready */
	if (XX_MallocSmartInit() != E_OK) {
		device_printf(dev, "could not initialize smart allocator.\n");
		return (ENXIO);
	}

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "fsl,qman-channel-range", qchan_range,
	    sizeof(qchan_range)) <= 0) {
		device_printf(dev, "Missing QMan channel range property!\n");
		return (ENXIO);
	}
	sc->qman_chan_base = qchan_range[0];
	sc->qman_chan_count = qchan_range[1];
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (!sc->mem_res) {
		device_printf(dev, "could not allocate memory.\n");
		return (ENXIO);
	}

	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (!sc->irq_res) {
		device_printf(dev, "could not allocate interrupt.\n");
		goto err;
	}

	sc->err_irq_rid = 1;
	sc->err_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->err_irq_rid, RF_ACTIVE | RF_SHAREABLE);
	if (!sc->err_irq_res) {
		device_printf(dev, "could not allocate error interrupt.\n");
		goto err;
	}

	/* Set FMan configuration */
	cfg.fman_device = dev;
	cfg.fm_id = device_get_unit(dev);
	cfg.mem_base_addr = rman_get_bushandle(sc->mem_res);
	cfg.irq_num = (uintptr_t)sc->irq_res;
	cfg.err_irq_num = (uintptr_t)sc->err_irq_res;
	cfg.exception_callback = fman_exception_callback;
	cfg.bus_error_callback = fman_error_callback;

	sc->fm_handle = fman_init(sc, &cfg);
	if (sc->fm_handle == NULL) {
		device_printf(dev, "could not be configured\n");
		goto err;
	}

	/* Initialize FMD chardev interface for userspace PCD control */
	if (sc->pcd_handle != NULL) {
		sc->fmcd = malloc(sizeof(struct fmcd_softc), M_FMAN,
		    M_WAITOK | M_ZERO);
		if (fmcd_init(dev, sc->fmcd) != 0) {
			free(sc->fmcd, M_FMAN);
			sc->fmcd = NULL;
			device_printf(dev,
			    "fmcd: chardev init failed (non-fatal)\n");
		}
	}

	bus_attach_children(dev);
	return (0);

err:
	fman_detach(dev);
	return (ENXIO);
}

int
fman_detach(device_t dev)
{
	struct fman_softc *sc;

	sc = device_get_softc(dev);

	if (sc->fmcd) {
		fmcd_destroy(sc->fmcd);
		free(sc->fmcd, M_FMAN);
		sc->fmcd = NULL;
	}
	if (sc->netenv_handle) {
		FM_PCD_NetEnvCharacteristicsDelete(sc->netenv_handle);
		sc->netenv_handle = NULL;
	}
	if (sc->pcd_handle) {
		FM_PCD_Disable(sc->pcd_handle);
		FM_PCD_Free(sc->pcd_handle);
		sc->pcd_handle = NULL;
	}

	if (sc->fm_handle) {
		FM_Free(sc->fm_handle);
		sc->fm_handle = NULL;
	}

	if (sc->muram_handle) {
		FM_MURAM_Free(sc->muram_handle);
		sc->muram_handle = NULL;
	}

	if (sc->mem_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);
	}

	if (sc->irq_res) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
	}

	if (sc->err_irq_res) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->err_irq_rid,
		    sc->err_irq_res);
	}

	return (0);
}

int
fman_suspend(device_t dev)
{

	return (0);
}

int
fman_resume_dev(device_t dev)
{

	return (0);
}

int
fman_shutdown(device_t dev)
{

	return (0);
}

int
fman_qman_channel_id(device_t dev, int port)
{
	struct fman_softc *sc;
#ifdef __aarch64__
	/* FMan v3 / Major >= 6 (LS1046A): 2x 10G + 6x 1G + 6x OH ports.
	 * Order matches Linux fman_get_qman_channel_id() for Major >= 6. */
	int qman_port_id[] = {0x30, 0x31, 0x28, 0x29, 0x2a, 0x2b,
	    0x2c, 0x2d, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
#else
	int qman_port_id[] = {0x31, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e,
	    0x2f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
#endif
	int i;

	sc = device_get_softc(dev);
	for (i = 0; i < sc->qman_chan_count; i++) {
		if (qman_port_id[i] == port)
			return (sc->qman_chan_base + i);
	}

	return (0);
}

/*
 * Hot-add Host Command port to existing PCD for advanced offload.
 *
 * Called by CDX module init (after QMan portals are ready) to add HC
 * support to the existing PCD.  Unlike the previous approach of
 * destroying and rebuilding PCD (which invalidated dtsec schemes and
 * caused FM_FD_ERR_NO_SCHEME), this preserves the PCD, all existing
 * KeyGen schemes, and dtsec port bindings.
 *
 * Sequence:
 *   1. Create QMan FQRs for HC TX/confirmation/error
 *   2. Disable PCD (brief pause — schemes survive)
 *   3. Create HC port via FmHcConfigAndInit
 *   4. Set p_FmPcd->h_Hc to enable HC in the PCD
 *   5. FM_PCD_SetAdvancedOffloadSupport (requires HC + PCD disabled)
 *   6. Re-enable PCD
 *
 * Returns 0 on success, errno on failure.
 */
int
fman_reinit_pcd_with_hc(device_t dev)
{
	struct fman_softc *sc;
	t_FmPcd *p_FmPcd;
	t_FmHcParams hc_params;
	t_Handle hc_handle;
	uint32_t hc_qman_chan;
	t_Handle fqr;
	t_Error error;

	sc = device_get_softc(dev);

	if (sc->pcd_handle == NULL) {
		device_printf(dev, "PCD not initialized\n");
		return (ENXIO);
	}

	/* Resolve QMan channel for HC offline port (cell-index 0x2) */
	hc_qman_chan = fman_qman_channel_id(dev, FMAN_HC_OH_CELL_INDEX);
	if (hc_qman_chan == 0) {
		device_printf(dev,
		    "couldn't get QMan channel for HC port\n");
		return (ENXIO);
	}

	/* TX FQR — HC enqueues command frames here for FMan */
	fqr = qman_fqr_create(1, hc_qman_chan, FMAN_HC_FQ_WQ,
	    false, 0, false, false, true, false, 0, 0, 0);
	if (fqr == NULL) {
		device_printf(dev, "couldn't create HC TX FQR\n");
		return (ENOMEM);
	}
	sc->hc_tx_fqr = fqr;

	/* Confirmation FQR — FMan sends HC completions here */
	fqr = qman_fqr_create(1, e_QM_FQ_CHANNEL_POOL1,
	    FMAN_HC_FQ_WQ, false, 0, false, false, true, false,
	    0, 0, 0);
	if (fqr == NULL) {
		device_printf(dev, "couldn't create HC confirmation FQR\n");
		return (ENOMEM);
	}
	sc->hc_conf_fqr = fqr;

	/*
	 * Register callback for HC confirmation frames.
	 * FMan sends command completions on confFqid; without this
	 * callback, they hit QMan's dummy handler and EnQFrm() in
	 * hc.c spins until timeout.
	 */
	error = qman_fqr_register_cb(sc->hc_conf_fqr,
	    fman_hc_conf_callback, sc);
	if (error != E_OK) {
		device_printf(dev,
		    "couldn't register HC confirmation callback: %d\n",
		    error);
		return (ENXIO);
	}

	/* Error FQR — FMan sends HC errors here */
	fqr = qman_fqr_create(1, e_QM_FQ_CHANNEL_POOL1,
	    FMAN_HC_FQ_WQ, false, 0, false, false, true, false,
	    0, 0, 0);
	if (fqr == NULL) {
		device_printf(dev, "couldn't create HC error FQR\n");
		return (ENOMEM);
	}
	sc->hc_err_fqr = fqr;

	/*
	 * Temporarily disable PCD to add HC and enable advanced
	 * offload.  Existing KeyGen schemes and port bindings survive
	 * the Disable/Enable cycle — only the PRS/KG/PLCR hardware
	 * engines are paused.
	 */
	error = FM_PCD_Disable(sc->pcd_handle);
	if (error != E_OK) {
		device_printf(dev, "couldn't disable PCD: %d\n", error);
		return (ENXIO);
	}

	/* Create HC port and attach to existing PCD */
	memset(&hc_params, 0, sizeof(hc_params));
	hc_params.h_Fm = sc->fm_handle;
	hc_params.h_FmPcd = sc->pcd_handle;
	hc_params.params.portBaseAddr = rman_get_bushandle(sc->mem_res) +
	    FMAN_HC_OH_REG_OFFSET;
	hc_params.params.portId = FMAN_HC_PORT_ID;
	hc_params.params.liodnBase = FMAN_HC_LIODN_BASE;
	hc_params.params.errFqid =
	    qman_fqr_get_base_fqid(sc->hc_err_fqr);
	hc_params.params.confFqid =
	    qman_fqr_get_base_fqid(sc->hc_conf_fqr);
	hc_params.params.qmChannel = hc_qman_chan;
	hc_params.params.f_QmEnqueue = fman_hc_qm_enqueue;
	hc_params.params.h_QmArg = sc;

	hc_handle = FmHcConfigAndInit(&hc_params);
	if (hc_handle == NULL) {
		device_printf(dev, "couldn't create HC port\n");
		FM_PCD_Enable(sc->pcd_handle);
		return (ENXIO);
	}

	/* Wire HC into the existing PCD */
	p_FmPcd = (t_FmPcd *)sc->pcd_handle;
	p_FmPcd->h_Hc = hc_handle;

	/* Enable advanced offload (requires HC + PCD disabled) */
	error = FM_PCD_SetAdvancedOffloadSupport(sc->pcd_handle);
	if (error != E_OK) {
		device_printf(dev,
		    "couldn't set advanced offload support: %d\n", error);
		/* Non-fatal — HC still works, just no offload flag */
	}

	/* Re-enable PCD — resumes Parser/KeyGen/CC processing */
	error = FM_PCD_Enable(sc->pcd_handle);
	if (error != E_OK) {
		device_printf(dev, "couldn't re-enable PCD: %d\n", error);
		return (ENXIO);
	}

	device_printf(dev, "PCD upgraded: HC port added, "
	    "existing schemes preserved\n");
	return (0);
}

/** @} */
