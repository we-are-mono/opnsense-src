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

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/sockio.h>
#include <sys/proc.h>
#include <sys/sched.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_arp.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include "miibus_if.h"

#include "opt_dpaa.h"

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <machine/in_cksum.h>

#include <contrib/ncsw/inc/integrations/dpaa_integration_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_mac_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_port_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_pcd_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_vsp_ext.h>
#include <contrib/ncsw/inc/xx_ext.h>
#include <contrib/ncsw/inc/net_ext.h>

#include "fman.h"
#include "fman_chardev.h"
#include "bman.h"
#include "qman.h"
#include "if_dtsec.h"
#include "if_dtsec_rm.h"


/**
 * @group dTSEC RM private defines.
 * @{
 */
#define	DTSEC_BPOOLS_USED	(1)
#define	DTSEC_MAX_TX_QUEUE_LEN	256

#define	DTSEC_RX_QUEUES		128	/* Power of 2, 7-bit hash mask like Linux */

/* FMan parse result L3/L4 type indicators (big-endian in parse result) */
#define	FM_L3_PARSE_RESULT_IPV4	0x8000
#define	FM_L3_PARSE_RESULT_IPV6	0x4000
#define	FM_L4_PARSE_RESULT_UDP	0x40
#define	FM_L4_PARSE_RESULT_TCP	0x20

struct dtsec_rm_frame_info {
	struct mbuf			*fi_mbuf;
	t_DpaaSGTE			fi_sgt[DPAA_NUM_OF_SG_TABLE_ENTRY];
	void				*fi_tx_buf; /* SG table buffer (prefix + SG entries) */
	uint8_t				fi_cpu;	    /* CPU index for in-flight tracking */
};

enum dtsec_rm_pool_params {
	DTSEC_RM_POOL_RX_LOW_MARK	= 256,
	DTSEC_RM_POOL_RX_HIGH_MARK	= 2048,
	DTSEC_RM_POOL_RX_MAX_SIZE	= 2048,
	/*
	 * Maximum in-flight RX buffers per port.
	 *
	 * Buffers are freed back to UMA when the mbuf is released
	 * (consumption model), so this bounds simultaneous allocation,
	 * not permanent memory commitment.  The original KVA pointer
	 * is stashed in the buffer's privData area before handing to
	 * BMan, then recovered via the DMAP address after the hardware
	 * round-trip to pass back to uma_zfree.
	 *
	 * 4096 × 9664 = ~38 MB peak per port.
	 */
	DTSEC_RM_POOL_RX_MAX_TOTAL	= 4096,
	/*
	 * Active refill threshold: when free buffer count drops below
	 * this, the RX callback allocates new buffers.  FMan acquires
	 * buffers directly from BMan hardware, bypassing software
	 * depletion callbacks — without active refill the pool exhausts
	 * under sustained RX load.
	 */
	DTSEC_RM_POOL_RX_REFILL_THRESH	= 1024,
	DTSEC_RM_POOL_RX_REFILL_COUNT	= 256,

	DTSEC_RM_POOL_FI_LOW_MARK	= 16,
	DTSEC_RM_POOL_FI_HIGH_MARK	= 64,
	DTSEC_RM_POOL_FI_MAX_SIZE	= 256,
};

#ifndef __aarch64__
#define	DTSEC_RM_FQR_RX_CHANNEL		e_QM_FQ_CHANNEL_POOL1
#endif
#define	DTSEC_RM_FQR_TX_CONF_CHANNEL	e_QM_FQ_CHANNEL_POOL1
enum dtsec_rm_fqr_params {
	DTSEC_RM_FQR_RX_WQ		= 1,
	DTSEC_RM_FQR_TX_WQ		= 1,
	DTSEC_RM_FQR_TX_CONF_WQ		= 1
};
/** @} */


/**
 * @group dTSEC Frame Info routines.
 * @{
 */
void
dtsec_rm_fi_pool_free(struct dtsec_softc *sc)
{

	if (sc->sc_fi_zone != NULL)
		uma_zdestroy(sc->sc_fi_zone);
	if (sc->sc_sgt_zone != NULL)
		uma_zdestroy(sc->sc_sgt_zone);
}

int
dtsec_rm_fi_pool_init(struct dtsec_softc *sc)
{

	snprintf(sc->sc_fi_zname, sizeof(sc->sc_fi_zname), "%s: Frame Info",
	    device_get_nameunit(sc->sc_dev));

	sc->sc_fi_zone = uma_zcreate(sc->sc_fi_zname,
	    sizeof(struct dtsec_rm_frame_info), NULL, NULL, NULL, NULL,
	    UMA_ALIGN_PTR, 0);

	return (0);
}

static struct dtsec_rm_frame_info *
dtsec_rm_fi_alloc(struct dtsec_softc *sc)
{
	struct dtsec_rm_frame_info *fi;

	fi = uma_zalloc(sc->sc_fi_zone, M_NOWAIT);

	return (fi);
}

static void
dtsec_rm_fi_free(struct dtsec_softc *sc, struct dtsec_rm_frame_info *fi)
{

	uma_zfree(sc->sc_fi_zone, fi);
}
/** @} */


/**
 * @group dTSEC FMan PORT routines.
 * @{
 */
int
dtsec_rm_fm_port_rx_init(struct dtsec_softc *sc, int unit)
{
	t_FmPortParams params;
	t_FmPortRxParams *rx_params;
	t_FmExtPools *pool_params;
	t_Error error;

	memset(&params, 0, sizeof(params));

	params.baseAddr = sc->sc_fm_base + sc->sc_port_rx_hw_id;
	params.h_Fm = sc->sc_fmh;
	params.portType = dtsec_fm_port_rx_type(sc->sc_eth_dev_type);
	params.portId = sc->sc_eth_id;
	params.independentModeEnable = false;
	params.liodnBase = FM_PORT_LIODN_BASE;
	params.f_Exception = dtsec_fm_port_rx_exception_callback;
	params.h_App = sc;

	rx_params = &params.specificParams.rxParams;
	rx_params->errFqid = sc->sc_rx_fqid;
	rx_params->dfltFqid = sc->sc_rx_fqid;
	rx_params->liodnOffset = 0;

	pool_params = &rx_params->extBufPools;
	pool_params->numOfPoolsUsed = DTSEC_BPOOLS_USED;
	pool_params->extBufPool->id = sc->sc_rx_bpid;
	pool_params->extBufPool->size = FM_PORT_BUFFER_SIZE;

	sc->sc_rxph = FM_PORT_Config(&params);
	if (sc->sc_rxph == NULL) {
		device_printf(sc->sc_dev, "couldn't configure FM Port RX.\n");
		return (ENXIO);
	}

	if (sc->sc_pcdh != NULL) {
		/*
		 * Configure RX buffer prefix: parse result for checksum
		 * validation and hash result for RSS distribution.
		 *
		 * manipExtraSpace = 96 matches the Linux SDK DTS
		 * buffer-layout = <0x60 0x40> which reserves 96 bytes
		 * for header manipulation (NAT, encap) by CDX/FMan.
		 */
		t_FmBufferPrefixContent prefix;

		memset(&prefix, 0, sizeof(prefix));
		prefix.privDataSize = 16;
		prefix.passPrsResult = TRUE;
		prefix.passHashResult = TRUE;
		prefix.dataAlign = 64;
		prefix.manipExtraSpace = 96;

		error = FM_PORT_ConfigBufferPrefixContent(sc->sc_rxph,
		    &prefix);
		if (error != E_OK) {
			device_printf(sc->sc_dev,
			    "couldn't configure RX buffer prefix.\n");
			FM_PORT_Free(sc->sc_rxph);
			sc->sc_rxph = NULL;
			return (ENXIO);
		}
	}

#ifdef FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669
	/* Errata A005669: discard frames with physical errors (FCS, SGMII
	 * disparity, FIFO overflow) in FMan hardware and increase FIFO by
	 * 4KB to prevent FMan hang under heavy traffic. */
	FM_PORT_ConfigBCBWorkaround(sc->sc_rxph);
#endif

	error = FM_PORT_Init(sc->sc_rxph);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "couldn't initialize FM Port RX.\n");
		FM_PORT_Free(sc->sc_rxph);
		sc->sc_rxph = NULL;
		return (ENXIO);
	}

	/* Enable BMI statistics counters for PCD diagnostics */
	FM_PORT_SetStatisticsCounters(sc->sc_rxph, TRUE);

	/*
	 * Reserve 1 port-private policer profile slot for this RX port.
	 * CDX QoS creates the actual profile via FM_PCD_PlcrProfileSet()
	 * when the CDX module loads (before dpa_app calls FM_PORT_SetPCD).
	 * Must be called after FM_PORT_Init() and before FM_PORT_SetPCD().
	 */
	error = FM_PORT_PcdPlcrAllocProfiles(sc->sc_rxph, 1);
	if (error != E_OK)
		device_printf(sc->sc_dev,
		    "FM_PORT_PcdPlcrAllocProfiles failed: %d\n", error);

	/*
	 * Allocate Virtual Storage Profiles for this RX port.
	 * FManv3+ (DPAA >= 11) requires per-port VSP allocation for
	 * the CC/PCD pipeline to work correctly after CDX/dpa_app
	 * replaces the default PCD.  This matches the Linux SDK driver
	 * which reads vsp-window = <2 0> from the DTS extended-args.
	 *
	 * Must be called after FM_PORT_Init() and before FM_PORT_SetPCD()
	 * and FM_PORT_Enable().
	 */
#if (DPAA_VERSION >= 11)
	if (sc->sc_pcdh != NULL && sc->sc_txph != NULL) {
		t_FmPortVSPAllocParams vsp_alloc;
		t_FmVspParams vsp_params;
		t_FmBufferPrefixContent vsp_prefix;
		t_Handle h_vsp;

		/* Step 1: Allocate 2 VSP slots for this port */
		memset(&vsp_alloc, 0, sizeof(vsp_alloc));
		vsp_alloc.numOfProfiles = 2;
		vsp_alloc.dfltRelativeId = 0;
		vsp_alloc.h_FmTxPort = sc->sc_txph;

		error = FM_PORT_VSPAlloc(sc->sc_rxph, &vsp_alloc);
		if (error != E_OK) {
			device_printf(sc->sc_dev,
			    "couldn't allocate VSP: error %d\n", error);
			goto vsp_done;
		}

		/* Step 2: Configure default VSP (profile 0) */
		memset(&vsp_params, 0, sizeof(vsp_params));
		vsp_params.h_Fm = sc->sc_fmh;
		vsp_params.portParams.portType =
		    dtsec_fm_port_rx_type(sc->sc_eth_dev_type);
		vsp_params.portParams.portId = sc->sc_eth_id;
		vsp_params.relativeProfileId = 0;
		vsp_params.liodnOffset = 0;
		vsp_params.extBufPools.numOfPoolsUsed = DTSEC_BPOOLS_USED;
		vsp_params.extBufPools.extBufPool[0].id = sc->sc_rx_bpid;
		vsp_params.extBufPools.extBufPool[0].size = FM_PORT_BUFFER_SIZE;

		h_vsp = FM_VSP_Config(&vsp_params);
		if (h_vsp == NULL) {
			device_printf(sc->sc_dev,
			    "couldn't configure VSP.\n");
			goto vsp_done;
		}

		/* Step 3: Set VSP buffer prefix (matching SDK DTS
		 * buffer-layout = <0x60 0x40>) */
		memset(&vsp_prefix, 0, sizeof(vsp_prefix));
		vsp_prefix.privDataSize = 16;
		vsp_prefix.passPrsResult = TRUE;
		vsp_prefix.passHashResult = TRUE;
		vsp_prefix.dataAlign = 64;
		vsp_prefix.manipExtraSpace = 96;

		error = FM_VSP_ConfigBufferPrefixContent(h_vsp, &vsp_prefix);
		if (error != E_OK) {
			device_printf(sc->sc_dev,
			    "couldn't configure VSP buffer prefix: %d\n",
			    error);
			FM_VSP_Free(h_vsp);
			goto vsp_done;
		}

		/* Step 4: Initialize VSP hardware registers */
		error = FM_VSP_Init(h_vsp);
		if (error != E_OK) {
			device_printf(sc->sc_dev,
			    "couldn't initialize VSP: %d\n", error);
			FM_VSP_Free(h_vsp);
			goto vsp_done;
		}

		sc->sc_vsph = h_vsp;
		device_printf(sc->sc_dev,
		    "VSP allocated: 2 profiles, default ID 0\n");
vsp_done: ;
	}
#endif /* DPAA_VERSION >= 11 */

	/* Register RX port with FMD chardev for userspace PCD control */
	{
		struct fmcd_softc *fmcd;

		fmcd = fman_get_fmcd(device_get_parent(sc->sc_dev));
		if (fmcd != NULL)
			fmcd_register_rx_port(fmcd,
			    sc->sc_mac_cell_index, sc->sc_rxph,
			    sc->sc_rx_fqid);
	}

	if (sc->sc_pcdh != NULL) {
		sc->sc_rx_data_offset =
		    FM_PORT_GetBufferDataOffset(sc->sc_rxph);
		if (bootverbose)
			device_printf(sc->sc_dev,
			    "RX buffer data offset: %u\n",
			    sc->sc_rx_data_offset);
	}

	if (bootverbose)
		device_printf(sc->sc_dev, "RX hw port 0x%02x initialized.\n",
		    sc->sc_port_rx_hw_id);

	return (0);
}

int
dtsec_rm_fm_port_tx_init(struct dtsec_softc *sc, int unit)
{
	t_FmPortParams params;
	t_FmPortNonRxParams *tx_params;
	t_Error error;

	memset(&params, 0, sizeof(params));

	params.baseAddr = sc->sc_fm_base + sc->sc_port_tx_hw_id;
	params.h_Fm = sc->sc_fmh;
	params.portType = dtsec_fm_port_tx_type(sc->sc_eth_dev_type);
	params.portId = sc->sc_eth_id;
	params.independentModeEnable = false;
	params.liodnBase = FM_PORT_LIODN_BASE;
	params.f_Exception = dtsec_fm_port_tx_exception_callback;
	params.h_App = sc;

	tx_params = &params.specificParams.nonRxParams;
	tx_params->errFqid = sc->sc_tx_conf_fqid;
	tx_params->dfltFqid = sc->sc_tx_conf_fqid;
	tx_params->qmChannel = sc->sc_port_tx_qman_chan;
#ifdef FM_OP_PARTITION_ERRATA_FMANx8
	tx_params->opLiodnOffset = 0;
#endif

	sc->sc_txph = FM_PORT_Config(&params);
	if (sc->sc_txph == NULL) {
		device_printf(sc->sc_dev, "couldn't configure FM Port TX.\n");
		return (ENXIO);
	}

	/* Configure TX buffer prefix to include parse results area.
	 * When TX checksum offload is requested, the driver fills in the
	 * parse result and sets FM_FD_CMD_RPD | FM_FD_CMD_DTC.
	 *
	 * manipExtraSpace = 96 matches the Linux SDK DTS
	 * buffer-layout = <0x60 0x40> for TX port extended-args.
	 */
	{
		t_FmBufferPrefixContent prefix;

		memset(&prefix, 0, sizeof(prefix));
		prefix.privDataSize = 16;
		prefix.passPrsResult = TRUE;
		prefix.passTimeStamp = FALSE;
		prefix.passHashResult = FALSE;
		prefix.passAllOtherPCDInfo = FALSE;
		prefix.dataAlign = 64;
		prefix.manipExtraSpace = 96;

		error = FM_PORT_ConfigBufferPrefixContent(sc->sc_txph,
		    &prefix);
		if (error != E_OK) {
			device_printf(sc->sc_dev,
			    "couldn't configure TX buffer prefix.\n");
			FM_PORT_Free(sc->sc_txph);
			sc->sc_txph = NULL;
			return (ENXIO);
		}
	}

	error = FM_PORT_Init(sc->sc_txph);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "couldn't initialize FM Port TX.\n");
		FM_PORT_Free(sc->sc_txph);
		sc->sc_txph = NULL;
		return (ENXIO);
	}

	sc->sc_tx_data_offset = FM_PORT_GetBufferDataOffset(sc->sc_txph);
	if (bootverbose)
		device_printf(sc->sc_dev, "TX buffer data offset: %u\n",
		    sc->sc_tx_data_offset);

	/* Create UMA zone for SG table buffers (prefix + SG entries) */
	sc->sc_sgt_buf_size = sc->sc_tx_data_offset +
	    DPAA_NUM_OF_SG_TABLE_ENTRY * sizeof(t_DpaaSGTE);
	snprintf(sc->sc_sgt_zname, sizeof(sc->sc_sgt_zname), "%s: SGT buf",
	    device_get_nameunit(sc->sc_dev));
	sc->sc_sgt_zone = uma_zcreate(sc->sc_sgt_zname, sc->sc_sgt_buf_size,
	    NULL, NULL, NULL, NULL, 63 /* 64-byte align */, 0);

	if (bootverbose)
		device_printf(sc->sc_dev, "TX hw port 0x%02x initialized.\n",
		    sc->sc_port_tx_hw_id);

	return (0);
}
/** @} */


/**
 * @group dTSEC buffer pools routines.
 * @{
 */

/*
 * Store the original KVA pointer from uma_zalloc in the buffer's
 * private data area (first 8 bytes).  FMan never touches privData
 * (privDataSize=16), so this survives the BMan/FMan/QMan round-trip.
 *
 * After the round-trip, XX_PhysToVirt returns a DMAP address — a
 * different virtual address for the same physical memory.  We read
 * back the original KVA pointer via the DMAP mapping to pass to
 * uma_zfree, which requires the exact address it originally returned.
 */
static inline void
dtsec_rm_buf_stash_ptr(void *buf)
{

	*(uintptr_t *)buf = (uintptr_t)buf;
}

static inline void *
dtsec_rm_buf_recover_ptr(void *buf)
{

	return ((void *)(*(uintptr_t *)buf));
}

static t_Error
dtsec_rm_pool_rx_put_buffer(t_Handle h_BufferPool, uint8_t *buffer,
    t_Handle context)
{
	struct dtsec_softc *sc;

	sc = h_BufferPool;
	uma_zfree(sc->sc_rx_zone, dtsec_rm_buf_recover_ptr(buffer));

	return (E_OK);
}

static uint8_t *
dtsec_rm_pool_rx_get_buffer(t_Handle h_BufferPool, t_Handle *context)
{
	struct dtsec_softc *sc;
	uint8_t *buffer;

	sc = h_BufferPool;
	buffer = uma_zalloc(sc->sc_rx_zone, M_NOWAIT);
	if (buffer != NULL)
		dtsec_rm_buf_stash_ptr(buffer);

	return (buffer);
}

/*
 * Refill the BMan buffer pool directly from UMA, bypassing
 * BM_POOL_FillBufs.  This avoids the NCSW RETURN_ERROR printf
 * when uma_zalloc(M_NOWAIT) transiently fails — a single printf
 * to serial console at 115200 baud takes ~5ms, blocking the CPU
 * in interrupt context and causing cascading buffer exhaustion.
 *
 * Returns the number of buffers actually allocated.
 */
static unsigned int
dtsec_rm_pool_rx_refill(struct dtsec_softc *sc, unsigned int nbufs)
{
	unsigned int i;
	uint8_t *buf;

	for (i = 0; i < nbufs; i++) {
		buf = uma_zalloc(sc->sc_rx_zone, M_NOWAIT);
		if (buf == NULL)
			break;
		dtsec_rm_buf_stash_ptr(buf);
		if (bman_put_buffer(sc->sc_rx_pool, buf) != 0) {
			uma_zfree(sc->sc_rx_zone, buf);
			break;
		}
	}
	return (i);
}

static void
dtsec_rm_pool_rx_depleted(t_Handle h_App, bool in)
{
	struct dtsec_softc *sc;
	unsigned int count, headroom, nbufs;

	sc = h_App;

	if (!in)
		return;

	/* Only refill up to the per-port cap */
	if (atomic_load_32(&sc->sc_rx_buf_total) >=
	    DTSEC_RM_POOL_RX_MAX_TOTAL)
		return;

	count = bman_count(sc->sc_rx_pool);
	if (count < DTSEC_RM_POOL_RX_HIGH_MARK) {
		headroom = DTSEC_RM_POOL_RX_MAX_TOTAL -
		    atomic_load_32(&sc->sc_rx_buf_total);
		nbufs = DTSEC_RM_POOL_RX_HIGH_MARK - count;
		if (nbufs > headroom)
			nbufs = headroom;
		nbufs = dtsec_rm_pool_rx_refill(sc, nbufs);
		atomic_add_32(&sc->sc_rx_buf_total, nbufs);
	}
}

void
dtsec_rm_pool_rx_free(struct dtsec_softc *sc)
{

	if (sc->sc_rx_pool != NULL)
		bman_pool_destroy(sc->sc_rx_pool);

	if (sc->sc_rx_zone != NULL)
		uma_zdestroy(sc->sc_rx_zone);
}

/*
 * BPID→dtsec_softc mapping for cross-module buffer release.
 *
 * When dpaa_wifi.ko receives a frame from CDX (buffer from a dtsec BMan
 * pool), it needs to free the buffer back to the correct dtsec's UMA zone
 * and decrement sc_rx_buf_total.  This table maps BPID to dtsec_softc.
 */
static struct dtsec_softc *dtsec_bpid_map[256];

int
dtsec_rm_pool_rx_init(struct dtsec_softc *sc)
{

	snprintf(sc->sc_rx_zname, sizeof(sc->sc_rx_zname), "%s: RX Buffers",
	    device_get_nameunit(sc->sc_dev));

	sc->sc_rx_zone = uma_zcreate(sc->sc_rx_zname, FM_PORT_BUFFER_SIZE, NULL,
	    NULL, NULL, NULL, 255 /* 256-byte align for DMA */, 0);

	sc->sc_rx_pool = bman_pool_create(&sc->sc_rx_bpid, FM_PORT_BUFFER_SIZE,
	    0, 0, DTSEC_RM_POOL_RX_MAX_SIZE, dtsec_rm_pool_rx_get_buffer,
	    dtsec_rm_pool_rx_put_buffer, DTSEC_RM_POOL_RX_LOW_MARK,
	    DTSEC_RM_POOL_RX_HIGH_MARK, 0, 0, dtsec_rm_pool_rx_depleted, sc, NULL,
	    NULL);
	if (sc->sc_rx_pool == NULL) {
		device_printf(sc->sc_dev, "NULL rx pool  somehow\n");
		dtsec_rm_pool_rx_free(sc);
		return (EIO);
	}

	sc->sc_rx_buf_total = DTSEC_RM_POOL_RX_MAX_SIZE;

	/* Register BPID→softc mapping for external buffer release */
	dtsec_bpid_map[sc->sc_rx_bpid] = sc;

	return (0);
}

void
dtsec_rm_buf_free_external(uint8_t bpid, void *buf)
{
	struct dtsec_softc *sc;

	sc = dtsec_bpid_map[bpid];
	if (__predict_false(sc == NULL)) {
		printf("dtsec_rm_buf_free_external: unknown BPID %u\n", bpid);
		return;
	}

	uma_zfree(sc->sc_rx_zone,
	    (void *)(*(uintptr_t *)buf));	/* recover stashed KVA */
	atomic_subtract_32(&sc->sc_rx_buf_total, 1);
}
/** @} */


/**
 * @group dTSEC Frame Queue Range routines.
 * @{
 */
static void
dtsec_rm_fqr_mext_free(struct mbuf *m)
{
	void *buffer;
	struct dtsec_softc *sc;

	buffer = m->m_ext.ext_arg1;
	sc = m->m_ext.ext_arg2;

	/*
	 * Consumption model: free the buffer back to UMA using the
	 * original KVA pointer stashed in the privData area.  The
	 * buffer address here is a DMAP address (from XX_PhysToVirt),
	 * but the stashed pointer at offset 0 is the original KVA
	 * address that uma_zalloc returned.
	 */
	uma_zfree(sc->sc_rx_zone, dtsec_rm_buf_recover_ptr(buffer));
	atomic_subtract_32(&sc->sc_rx_buf_total, 1);
}

static e_RxStoreResponse
dtsec_rm_fqr_rx_callback(t_Handle app, t_Handle fqr, t_Handle portal,
    uint32_t fqid_off, t_DpaaFD *frame)
{
	struct dtsec_softc *sc;
	struct mbuf *m;
	void *frame_va;

	m = NULL;
	sc = app;

	/*
	 * Refill BMan buffer pool if running low.
	 * FMan acquires buffers directly from BMan hardware, bypassing
	 * software depletion callbacks.  Without active refill, the pool
	 * exhausts under sustained RX load: all buffers end up in mbufs
	 * held by the TCP stack, FMan drops every subsequent frame, and
	 * retransmissions are also dropped — permanent deadlock.
	 */
	if (__predict_false(bman_count(sc->sc_rx_pool) <
	    DTSEC_RM_POOL_RX_REFILL_THRESH) &&
	    atomic_load_32(&sc->sc_rx_buf_total) <
	    DTSEC_RM_POOL_RX_MAX_TOTAL) {
		unsigned int n, added;
		uint32_t total;

		total = atomic_load_32(&sc->sc_rx_buf_total);
		n = DTSEC_RM_POOL_RX_REFILL_COUNT;
		if (n > DTSEC_RM_POOL_RX_MAX_TOTAL - total)
			n = DTSEC_RM_POOL_RX_MAX_TOTAL - total;
		added = dtsec_rm_pool_rx_refill(sc, n);
		atomic_add_32(&sc->sc_rx_buf_total, added);
	}

	frame_va = DPAA_FD_GET_ADDR(frame);
	if (frame_va == NULL)
		return (e_RX_STORE_RESPONSE_CONTINUE);

	if (DPAA_FD_GET_FORMAT(frame) != e_DPAA_FD_FORMAT_TYPE_SHORT_SBSF) {
		/* SG/compound FDs should not occur with our buffer pool size.
		 * Log and drop rather than panic. */
		if_printf(sc->sc_ifnet,
		    "RX: unexpected FD format 0x%02X, dropping\n",
		    DPAA_FD_GET_FORMAT(frame));
		goto err;
	}

	{
		uint32_t fd_st = DPAA_FD_GET_STATUS(frame);
		uint32_t err_bits = fd_st &
		    (FM_FD_RX_STATUS_ERR_MASK & ~FM_FD_ERR_NO_SCHEME);
		if (err_bits) {
			static volatile uint32_t rx_err_cnt;
			uint32_t n = atomic_fetchadd_32(&rx_err_cnt, 1) + 1;
			if (n <= 10 || (n % 100) == 0)
				if_printf(sc->sc_ifnet,
				    "RX error #%u: fd_status=0x%08X"
				    " (phy=%d cls=%d red=%d yel=%d"
				    " plcr=%d prs=%d ext=%d)\n",
				    n, fd_st,
				    !!(fd_st & FM_FD_ERR_PHYSICAL),
				    !!(fd_st & FM_FD_ERR_CLS_DISCARD),
				    !!(fd_st & FM_FD_ERR_COLOR_RED),
				    !!(fd_st & FM_FD_ERR_COLOR_YELLOW),
				    !!(fd_st & FM_FD_ERR_ILL_PLCR),
				    !!(fd_st & FM_FD_ERR_PRS_HDR_ERR),
				    !!(fd_st & FM_FD_ERR_EXTRACTION));
			goto err;
		}
	}

	m = m_gethdr(M_NOWAIT, MT_HEADER);
	if (m == NULL)
		goto err;

	m_extadd(m, frame_va, FM_PORT_BUFFER_SIZE,
	    dtsec_rm_fqr_mext_free, frame_va, sc, 0,
	    EXT_NET_DRV);

	m->m_pkthdr.rcvif = sc->sc_ifnet;
	m->m_data = (char *)frame_va + DPAA_FD_GET_OFFSET(frame);
	m->m_len = DPAA_FD_GET_LENGTH(frame);
	m->m_pkthdr.len = m->m_len;

	/* Extract RX checksum result from parse result in buffer prefix */
	if (sc->sc_rx_data_offset > 0) {
		t_FmPrsResult *prs;

		prs = FM_PORT_GetBufferPrsResult(sc->sc_rxph,
		    (char *)frame_va);
		if (prs != NULL) {
			uint16_t l3r = be16toh(prs->l3r);

			if (l3r & (FM_L3_PARSE_RESULT_IPV4 |
			    FM_L3_PARSE_RESULT_IPV6)) {
				m->m_pkthdr.csum_flags |=
				    CSUM_L3_CALC | CSUM_L3_VALID;

				if (prs->l4r & (FM_L4_PARSE_RESULT_TCP |
				    FM_L4_PARSE_RESULT_UDP)) {
					uint32_t fd_status =
					    DPAA_FD_GET_STATUS(frame);
					/*
					 * DCL4C = "Didn't Compute L4 Checksum".
					 * Only claim HW-verified when clear.
					 * When set, leave flags alone so the
					 * stack verifies L4 csum in software.
					 *
					 * csum_data = 0xffff: tells the stack
					 * the full checksum (incl pseudo-hdr)
					 * is valid. Without this, the stack
					 * reads csum_data=0 and drops the
					 * packet as a checksum failure.
					 */
					if (!(fd_status & FM_FD_CMD_DCL4C)) {
						m->m_pkthdr.csum_flags |=
						    CSUM_L4_CALC |
						    CSUM_L4_VALID;
						m->m_pkthdr.csum_data =
						    0xffff;
					}
				}
			}
		}
	}

	if_inc_counter(sc->sc_ifnet, IFCOUNTER_IPACKETS, 1);
	if_inc_counter(sc->sc_ifnet, IFCOUNTER_IBYTES, m->m_pkthdr.len);
#ifdef __aarch64__
	qman_rx_defer(m);
#else
	if_input(sc->sc_ifnet, m);
#endif

	return (e_RX_STORE_RESPONSE_CONTINUE);

err:
	if_inc_counter(sc->sc_ifnet, IFCOUNTER_IERRORS, 1);
	if (frame_va != NULL) {
		uma_zfree(sc->sc_rx_zone,
		    dtsec_rm_buf_recover_ptr(frame_va));
		atomic_subtract_32(&sc->sc_rx_buf_total, 1);
	}
	if (m != NULL)
		m_freem(m);

	return (e_RX_STORE_RESPONSE_CONTINUE);
}

static e_RxStoreResponse
dtsec_rm_fqr_tx_confirm_callback(t_Handle app, t_Handle fqr, t_Handle portal,
    uint32_t fqid_off, t_DpaaFD *frame)
{
	struct dtsec_rm_frame_info *fi;
	struct dtsec_softc *sc;
	t_DpaaSGTE *sgt0;
	void *buf;
	int cpu;

	sc = app;

	buf = DPAA_FD_GET_ADDR(frame);
	if (buf == NULL)
		return (e_RX_STORE_RESPONSE_CONTINUE);

	if (DPAA_FD_GET_FORMAT(frame) == e_DPAA_FD_FORMAT_TYPE_SHORT_MBSF &&
	    DPAA_FD_GET_OFFSET(frame) > 0) {
		/* SG+prefix FD (checksum offload path).
		 * fi pointer stored in private data area at start of buf.
		 * SG table buffer allocated from sc_sgt_zone. */
		fi = *(struct dtsec_rm_frame_info **)buf;
		cpu = fi->fi_cpu;
		m_freem(fi->fi_mbuf);
		uma_zfree(sc->sc_sgt_zone, buf);
		dtsec_rm_fi_free(sc, fi);
	} else {
		/* Legacy SG FD (no prefix, offset=0).
		 * fi pointer stored in first SG entry. */
		sgt0 = buf;
		fi = DPAA_SGTE_GET_ADDR(sgt0);
		cpu = fi->fi_cpu;
		m_freem(fi->fi_mbuf);
		dtsec_rm_fi_free(sc, fi);
	}

	atomic_subtract_int(&sc->sc_tx_inflight[cpu], 1);

	return (e_RX_STORE_RESPONSE_CONTINUE);
}

void
dtsec_rm_fqr_rx_free(struct dtsec_softc *sc)
{
	int i;

	for (i = 0; i < DTSEC_RX_QUEUES; i++) {
		if (sc->sc_rx_fqr[i] != NULL) {
			qman_fqr_free(sc->sc_rx_fqr[i]);
			sc->sc_rx_fqr[i] = NULL;
		}
	}
}

/*
 * dtsec_release_rss_fqrs — Free RSS FQRs 1..127 from all dtsec ports.
 *
 * Called by CDX module during takeover, before creating CDX distribution
 * FQRs.  After CDX replaces the driver's PCD, the RSS FQRs are orphaned
 * (no KeyGen scheme directs traffic to them).  FQR 0 (base FQID) is kept
 * alive — CDX miss-action routes unmatched frames there for stack delivery.
 *
 * Safe to call at boot before any traffic has flowed (instant, no drain).
 */
void
dtsec_release_rss_fqrs(void)
{
	devclass_t dc;
	device_t dev;
	struct dtsec_softc *sc;
	int i, unit;

	dc = devclass_find("dtsec");
	if (dc == NULL)
		return;

	for (unit = 0; unit < devclass_get_maxunit(dc); unit++) {
		dev = devclass_get_device(dc, unit);
		if (dev == NULL)
			continue;
		sc = device_get_softc(dev);
		if (sc == NULL || sc->sc_rx_fqr[1] == NULL)
			continue;
		for (i = 1; i < DTSEC_RX_QUEUES; i++) {
			if (sc->sc_rx_fqr[i] != NULL) {
				qman_fqr_free(sc->sc_rx_fqr[i]);
				sc->sc_rx_fqr[i] = NULL;
			}
		}
		device_printf(dev,
		    "released %d RSS FQRs for CDX (base FQID %u kept)\n",
		    DTSEC_RX_QUEUES - 1, sc->sc_rx_fqid);
	}
}

int
dtsec_rm_fqr_rx_init(struct dtsec_softc *sc)
{
	t_Error error;
	t_Handle fqr;
	uint32_t base_fqid;
	int i;

#ifdef __aarch64__
	/*
	 * RSS via per-CPU dedicated QMan channels (matching Linux).
	 *
	 * Create DTSEC_RX_QUEUES (128) single-FQID FQRs with contiguous
	 * FQIDs, round-robin assigned to dedicated SW portal channels:
	 *   FQID base+0 -> SWPORTAL0, base+1 -> SWPORTAL1, ...
	 *   base+4 -> SWPORTAL0, base+5 -> SWPORTAL1, etc.
	 *
	 * KeyGen hashes 4-tuple -> baseFqid + (hash & 127).  With a
	 * 7-bit hash mask, different flows distribute well across CPUs.
	 * Same flow always hashes to same FQID -> same CPU (no reorder).
	 *
	 * The first FQR is allocated with alignment = DTSEC_RX_QUEUES
	 * to satisfy the KeyGen alignment requirement.  The remaining
	 * 127 FQRs use force_fqid for contiguous FQIDs.
	 */

	/* FQID 0: allocate with alignment to get contiguous base */
	fqr = qman_fqr_create(1,
	    (e_QmFQChannel)(e_QM_FQ_CHANNEL_SWPORTAL0),
	    DTSEC_RM_FQR_RX_WQ, false,
	    DTSEC_RX_QUEUES,  /* alignment */
	    false, false, true, false, 0, 0, 0);
	if (fqr == NULL) {
		device_printf(sc->sc_dev,
		    "could not create aligned RX FQR for FQID 0\n");
		return (EIO);
	}

	sc->sc_rx_fqr[0] = fqr;
	base_fqid = qman_fqr_get_base_fqid(fqr);
	sc->sc_rx_fqid = base_fqid;

	error = qman_fqr_register_cb(fqr, dtsec_rm_fqr_rx_callback, sc);
	if (error != E_OK) {
		device_printf(sc->sc_dev,
		    "could not register RX callback for FQID 0\n");
		qman_fqr_free(fqr);
		sc->sc_rx_fqr[0] = NULL;
		return (EIO);
	}

	/* FQIDs 1..N-1: force-allocate, round-robin across CPUs */
	for (i = 1; i < DTSEC_RX_QUEUES; i++) {
		e_QmFQChannel channel =
		    (e_QmFQChannel)(e_QM_FQ_CHANNEL_SWPORTAL0 +
		    (i % DTSEC_TX_NUM_FQS));

		fqr = qman_fqr_create(1, channel, DTSEC_RM_FQR_RX_WQ,
		    true, base_fqid + i,
		    false, false, true, false, 0, 0, 0);
		if (fqr == NULL) {
			device_printf(sc->sc_dev,
			    "could not create RX FQR %d "
			    "(FQID %u, channel %d)\n",
			    i, base_fqid + i, (int)channel);
			goto fail;
		}

		sc->sc_rx_fqr[i] = fqr;

		error = qman_fqr_register_cb(fqr,
		    dtsec_rm_fqr_rx_callback, sc);
		if (error != E_OK) {
			device_printf(sc->sc_dev,
			    "could not register RX callback for FQ %d\n", i);
			qman_fqr_free(fqr);
			sc->sc_rx_fqr[i] = NULL;
			goto fail;
		}
	}

	device_printf(sc->sc_dev,
	    "RX RSS: base FQID %u, %d queues across %d CPUs\n",
	    base_fqid, DTSEC_RX_QUEUES, DTSEC_TX_NUM_FQS);

	return (0);

fail:
	dtsec_rm_fqr_rx_free(sc);
	return (EIO);

#else /* !__aarch64__ — PowerPC: single FQR on pool channel */
	fqr = qman_fqr_create(DTSEC_RX_QUEUES, DTSEC_RM_FQR_RX_CHANNEL,
	    DTSEC_RM_FQR_RX_WQ, false,
	    DTSEC_RX_QUEUES > 1 ? DTSEC_RX_QUEUES : 0,
	    false, false, true, false, 0, 0, 0);
	if (fqr == NULL) {
		device_printf(sc->sc_dev, "could not create RX queue range\n");
		return (EIO);
	}

	sc->sc_rx_fqr[0] = fqr;
	sc->sc_rx_fqid = qman_fqr_get_base_fqid(fqr);

	error = qman_fqr_register_cb(fqr, dtsec_rm_fqr_rx_callback, sc);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "could not register RX callback\n");
		dtsec_rm_fqr_rx_free(sc);
		return (EIO);
	}

	if (bootverbose)
		device_printf(sc->sc_dev, "RX FQR: base FQID %u, %d queues\n",
		    sc->sc_rx_fqid, DTSEC_RX_QUEUES);

	return (0);
#endif
}

void
dtsec_rm_fqr_tx_free(struct dtsec_softc *sc)
{
	int i;

	for (i = 0; i < DTSEC_TX_NUM_FQS; i++) {
		if (sc->sc_tx_fqs[i] != NULL) {
			qman_fqr_free(sc->sc_tx_fqs[i]);
			sc->sc_tx_fqs[i] = NULL;
		}
	}

	if (sc->sc_tx_conf_fqr != NULL) {
		qman_fqr_free(sc->sc_tx_conf_fqr);
		sc->sc_tx_conf_fqr = NULL;
	}
}

int
dtsec_rm_fqr_tx_init(struct dtsec_softc *sc)
{
	t_Error error;
	t_Handle fqr;
	int i;

	/* Per-CPU TX Frame Queues — all target the same FMan TX port channel */
	for (i = 0; i < DTSEC_TX_NUM_FQS; i++) {
		fqr = qman_fqr_create(1, sc->sc_port_tx_qman_chan,
		    DTSEC_RM_FQR_TX_WQ, false, 0, false, false, true, false,
		    0, 0, 0);
		if (fqr == NULL) {
			device_printf(sc->sc_dev,
			    "could not create TX queue for CPU %d\n", i);
			dtsec_rm_fqr_tx_free(sc);
			return (EIO);
		}
		sc->sc_tx_fqs[i] = fqr;
		sc->sc_tx_inflight[i] = 0;
	}

	/* TX Confirmation Frame Queue (single, on pool channel) */
	fqr = qman_fqr_create(1, DTSEC_RM_FQR_TX_CONF_CHANNEL,
	    DTSEC_RM_FQR_TX_CONF_WQ, false, 0, false, false, true, false, 0, 0,
	    0);
	if (fqr == NULL) {
		device_printf(sc->sc_dev, "could not create TX confirmation "
		    "queue\n");
		dtsec_rm_fqr_tx_free(sc);
		return (EIO);
	}

	sc->sc_tx_conf_fqr = fqr;
	sc->sc_tx_conf_fqid = qman_fqr_get_base_fqid(fqr);

	error = qman_fqr_register_cb(fqr, dtsec_rm_fqr_tx_confirm_callback, sc);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "could not register TX confirmation "
		    "callback\n");
		dtsec_rm_fqr_tx_free(sc);
		return (EIO);
	}

	if (bootverbose)
		device_printf(sc->sc_dev,
		    "TX FQs: %d per-CPU queues, confirm FQID %u\n",
		    DTSEC_TX_NUM_FQS, sc->sc_tx_conf_fqid);

	return (0);
}
/** @} */


/**
 * @group dTSEC PCD (Parse-Classify-Distribute) routines.
 * @{
 */

/**
 * Initialize PCD on the RX port: create a KeyGen scheme for RSS
 * hash distribution across DTSEC_RX_QUEUES, then attach PCD
 * (Parser + KeyGen) to the RX port.
 *
 * Called from dtsec_attach() after RX port and FQR are initialized.
 * Non-fatal: returns 0 on success or if PCD is unavailable.
 */
int
dtsec_rm_pcd_init(struct dtsec_softc *sc)
{
	t_FmPcdKgSchemeParams scheme;
	t_FmPortPcdParams port_pcd;
	t_FmPortPcdPrsParams prs;
	t_FmPortPcdKgParams kg;
	t_Error error;
	int i;

	if (sc->sc_pcdh == NULL)
		return (0);

	/* --- Create KeyGen scheme --- */
	memset(&scheme, 0, sizeof(scheme));
	scheme.modify = FALSE;
	scheme.id.relativeSchemeId = device_get_unit(sc->sc_dev);
	scheme.alwaysDirect = TRUE;
	scheme.useHash = TRUE;
	scheme.bypassFqidGeneration = FALSE;
	scheme.baseFqid = sc->sc_rx_fqid;
	scheme.nextEngine = e_FM_PCD_DONE;
	scheme.kgNextEngineParams.doneAction = e_FM_PCD_ENQ_FRAME;

	/*
	 * Extract 4-tuple for hash: IPv4 SRC/DST + L4 SRC/DST port.
	 * Using HEADER_TYPE_TCP for L4 ports — the KeyGen's known-field
	 * registers (KG_SCH_KN_L4PSRC/L4PDST) are protocol-agnostic and
	 * extract L4 ports for both TCP and UDP.  For non-IP/non-L4
	 * frames, the default value (zeros) is used, so they all hash
	 * to baseFqid + 0.
	 */
	i = 0;

	/* IPv4 source IP */
	scheme.keyExtractAndHashParams.extractArray[i].type =
	    e_FM_PCD_EXTRACT_BY_HDR;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdr =
	    HEADER_TYPE_IPv4;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdrIndex =
	    e_FM_PCD_HDR_INDEX_NONE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .ignoreProtocolValidation = FALSE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.type =
	    e_FM_PCD_EXTRACT_FULL_FIELD;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_SRC_IP;
	i++;

	/* IPv4 destination IP */
	scheme.keyExtractAndHashParams.extractArray[i].type =
	    e_FM_PCD_EXTRACT_BY_HDR;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdr =
	    HEADER_TYPE_IPv4;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdrIndex =
	    e_FM_PCD_HDR_INDEX_NONE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .ignoreProtocolValidation = FALSE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.type =
	    e_FM_PCD_EXTRACT_FULL_FIELD;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .extractByHdrType.fullField.ipv4 = NET_HEADER_FIELD_IPv4_DST_IP;
	i++;

	/* TCP/UDP source port (KG_SCH_KN_L4PSRC covers both TCP and UDP) */
	scheme.keyExtractAndHashParams.extractArray[i].type =
	    e_FM_PCD_EXTRACT_BY_HDR;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdr =
	    HEADER_TYPE_TCP;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdrIndex =
	    e_FM_PCD_HDR_INDEX_NONE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .ignoreProtocolValidation = FALSE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.type =
	    e_FM_PCD_EXTRACT_FULL_FIELD;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .extractByHdrType.fullField.tcp = NET_HEADER_FIELD_TCP_PORT_SRC;
	i++;

	/* TCP/UDP destination port (KG_SCH_KN_L4PDST covers both TCP and UDP) */
	scheme.keyExtractAndHashParams.extractArray[i].type =
	    e_FM_PCD_EXTRACT_BY_HDR;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdr =
	    HEADER_TYPE_TCP;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.hdrIndex =
	    e_FM_PCD_HDR_INDEX_NONE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .ignoreProtocolValidation = FALSE;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr.type =
	    e_FM_PCD_EXTRACT_FULL_FIELD;
	scheme.keyExtractAndHashParams.extractArray[i].extractByHdr
	    .extractByHdrType.fullField.tcp = NET_HEADER_FIELD_TCP_PORT_DST;
	i++;

	scheme.keyExtractAndHashParams.numOfUsedExtracts = i;
	scheme.keyExtractAndHashParams.hashDistributionNumOfFqids =
	    DTSEC_RX_QUEUES;
	scheme.keyExtractAndHashParams.hashShift = 0;
	scheme.keyExtractAndHashParams.hashDistributionFqidsShift = 0;
	scheme.keyExtractAndHashParams.symmetricHash = FALSE;

	/* Defaults for missing headers: use privateDflt0 (all zeros) */
	scheme.keyExtractAndHashParams.numOfUsedDflts = 2;
	scheme.keyExtractAndHashParams.dflts[0].type = e_FM_PCD_KG_IP_ADDR;
	scheme.keyExtractAndHashParams.dflts[0].dfltSelect =
	    e_FM_PCD_KG_DFLT_PRIVATE_0;
	scheme.keyExtractAndHashParams.dflts[1].type = e_FM_PCD_KG_L4_PORT;
	scheme.keyExtractAndHashParams.dflts[1].dfltSelect =
	    e_FM_PCD_KG_DFLT_PRIVATE_0;

	sc->sc_scheme = FM_PCD_KgSchemeSet(sc->sc_pcdh, &scheme);
	if (sc->sc_scheme == NULL) {
		device_printf(sc->sc_dev, "couldn't create KeyGen scheme\n");
		return (ENXIO);
	}

	device_printf(sc->sc_dev,
	    "KeyGen scheme created: baseFqid=%u, queues=%d\n",
	    sc->sc_rx_fqid, DTSEC_RX_QUEUES);

	/* --- Attach PCD to RX port --- */
	memset(&port_pcd, 0, sizeof(port_pcd));
	port_pcd.pcdSupport = e_FM_PORT_PCD_SUPPORT_PRS_AND_KG;
	port_pcd.h_NetEnv = sc->sc_netenvh;

	memset(&prs, 0, sizeof(prs));
	prs.firstPrsHdr = HEADER_TYPE_ETH;
	port_pcd.p_PrsParams = &prs;

	memset(&kg, 0, sizeof(kg));
	kg.numOfSchemes = 1;
	kg.h_Schemes[0] = sc->sc_scheme;
	kg.directScheme = TRUE;
	kg.h_DirectScheme = sc->sc_scheme;
	port_pcd.p_KgParams = &kg;

	error = FM_PORT_SetPCD(sc->sc_rxph, &port_pcd);
	if (error != E_OK) {
		device_printf(sc->sc_dev,
		    "couldn't set PCD on RX port: %d\n", error);
		FM_PCD_KgSchemeDelete(sc->sc_scheme);
		sc->sc_scheme = NULL;
		return (ENXIO);
	}

	device_printf(sc->sc_dev, "PCD enabled: PRS+KG on RX port\n");


	/* Register scheme with chardev so it can tear down PCD when
	 * userspace (FMC/dpa_app) takes over PCD management. */
	{
		struct fmcd_softc *fmcd;

		fmcd = fman_get_fmcd(device_get_parent(sc->sc_dev));
		if (fmcd != NULL)
			fmcd_register_rx_port_scheme(fmcd,
			    sc->sc_mac_cell_index,
			    sc->sc_scheme, &sc->sc_scheme);
	}

	return (0);
}

void
dtsec_rm_pcd_free(struct dtsec_softc *sc)
{

	if (sc->sc_scheme != NULL) {
		FM_PORT_DeletePCD(sc->sc_rxph);
		FM_PCD_KgSchemeDelete(sc->sc_scheme);
		sc->sc_scheme = NULL;
	}
}
/** @} */


/**
 * Fill parse result for TX checksum offload.
 * Returns 0 on success, non-zero if the frame can't be checksummed by HW.
 */
static int
dtsec_rm_tx_csum_fill(struct dtsec_softc *sc, struct mbuf *m,
    t_FmPrsResult *prs)
{
	struct ether_header *eh;
	struct ip *ip;
	struct ip6_hdr *ip6;
	uint16_t etype;
	int ehlen;

	if (m->m_len < sizeof(*eh))
		return (EINVAL);

	eh = mtod(m, struct ether_header *);
	etype = ntohs(eh->ether_type);
	ehlen = sizeof(*eh);

	/* Handle VLAN-tagged frames */
	if (etype == ETHERTYPE_VLAN) {
		if (m->m_len < sizeof(*eh) + 4)
			return (EINVAL);
		etype = ntohs(*(uint16_t *)((char *)eh + sizeof(*eh) + 2));
		ehlen += 4;
	}

	memset(prs, 0, sizeof(*prs));

	switch (etype) {
	case ETHERTYPE_IP:
		if (m->m_len < ehlen + sizeof(*ip))
			return (EINVAL);
		prs->l3r = htobe16(FM_L3_PARSE_RESULT_IPV4);
		ip = (struct ip *)((char *)eh + ehlen);
		prs->ip_off[0] = ehlen;
		prs->l4_off = ehlen + (ip->ip_hl << 2);
		switch (ip->ip_p) {
		case IPPROTO_TCP:
			prs->l4r = FM_L4_PARSE_RESULT_TCP;
			break;
		case IPPROTO_UDP:
			prs->l4r = FM_L4_PARSE_RESULT_UDP;
			break;
		default:
			return (EINVAL);
		}
		break;

	case ETHERTYPE_IPV6:
		if (m->m_len < ehlen + sizeof(*ip6))
			return (EINVAL);
		prs->l3r = htobe16(FM_L3_PARSE_RESULT_IPV6);
		ip6 = (struct ip6_hdr *)((char *)eh + ehlen);
		prs->ip_off[0] = ehlen;
		prs->l4_off = ehlen + sizeof(*ip6);
		switch (ip6->ip6_nxt) {
		case IPPROTO_TCP:
			prs->l4r = FM_L4_PARSE_RESULT_TCP;
			break;
		case IPPROTO_UDP:
			prs->l4r = FM_L4_PARSE_RESULT_UDP;
			break;
		default:
			return (EINVAL);
		}
		break;

	default:
		return (EINVAL);
	}

	return (0);
}

/**
 * @group dTSEC IFnet routines.
 * @{
 */

/**
 * Extract DSCP from the IP header of an Ethernet frame.
 * Returns 0 for non-IP or unparseable frames (DSCP 0 maps to
 * unmapped in the CEETM array, falling back to the default FQ).
 */
static inline uint8_t
dtsec_rm_extract_dscp(struct mbuf *m)
{
	struct ether_header *eh;
	uint16_t etype;
	int ehlen;

	if (__predict_false(m->m_len < (int)sizeof(*eh)))
		return (0);

	eh = mtod(m, struct ether_header *);
	etype = ntohs(eh->ether_type);
	ehlen = sizeof(*eh);

	if (etype == ETHERTYPE_VLAN) {
		if (__predict_false(m->m_len < ehlen + 4))
			return (0);
		etype = ntohs(*(uint16_t *)((char *)eh + ehlen + 2));
		ehlen += 4;
	}

	if (etype == ETHERTYPE_IP) {
		struct ip *ip;
		if (__predict_false(m->m_len < ehlen + (int)sizeof(*ip)))
			return (0);
		ip = (struct ip *)((char *)eh + ehlen);
		return ((ip->ip_tos >> 2) & 0x3f);
	}

	if (etype == ETHERTYPE_IPV6) {
		struct ip6_hdr *ip6;
		if (__predict_false(m->m_len < ehlen + (int)sizeof(*ip6)))
			return (0);
		ip6 = (struct ip6_hdr *)((char *)eh + ehlen);
		return ((ntohl(ip6->ip6_flow) >> 22) & 0x3f);
	}

	return (0);
}

/**
 * Modern if_transmit entry point for RM (Regular Mode).
 *
 * Each CPU builds the FD and enqueues to its own per-CPU TX FQ on its
 * own QMan portal.  No locks, no sendq, no contention.  The stack
 * calls this directly with one mbuf at a time.
 */
int
dtsec_rm_if_transmit(if_t ifp, struct mbuf *m0)
{
	struct dtsec_softc *sc;
	struct dtsec_rm_frame_info *fi;
	vm_size_t dsize, psize, ssize;
	unsigned int i;
	struct mbuf *m;
	vm_offset_t vaddr;
	t_DpaaFD fd;
	int cpu, error;
	t_DpaaSGTE *sgt;
	void *sgt_buf;

	sc = if_getsoftc(ifp);

	if (__predict_false(!(if_getdrvflags(ifp) & IFF_DRV_RUNNING))) {
		m_freem(m0);
		return (ENETDOWN);
	}

	/* Pin to current CPU for consistent FQ selection and portal use */
	sched_pin();
	cpu = curcpu % DTSEC_TX_NUM_FQS;

	/* CEETM DSCP-based FQ selection for software-path packets */
	uint32_t ceetm_fqid = 0;
	if (__predict_false(sc->sc_ceetm_en)) {
		uint8_t dscp = dtsec_rm_extract_dscp(m0);
		ceetm_fqid = sc->sc_ceetm_dscp_fqid[dscp];
	}

	/* Check per-CPU in-flight limit (software counter, no portal read) */
	if (__predict_false(atomic_load_int(&sc->sc_tx_inflight[cpu]) >=
	    DTSEC_MAX_TX_QUEUE_LEN)) {
		sched_unpin();
		m_freem(m0);
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		return (ENOBUFS);
	}

	fi = dtsec_rm_fi_alloc(sc);
	if (__predict_false(fi == NULL)) {
		sched_unpin();
		m_freem(m0);
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		return (ENOBUFS);
	}

	fi->fi_cpu = cpu;

	fi->fi_tx_buf = NULL;

	/*
	 * TX checksum offload: build SG FD with buffer prefix
	 * containing parse result.  SG entries point directly to
	 * mbuf fragment data — zero-copy, no linearization.
	 */
	if ((m0->m_pkthdr.csum_flags &
	    (CSUM_IP | CSUM_IP_TCP | CSUM_IP_UDP |
	     CSUM_IP6_TCP | CSUM_IP6_UDP)) &&
	    sc->sc_tx_data_offset > 0) {
		t_FmPrsResult prs;
		t_FmPrsResult *buf_prs;

		/* Fill parse result for hardware checksum */
		if (dtsec_rm_tx_csum_fill(sc, m0, &prs) != 0) {
			/* Can't offload L4 (e.g. ICMP) — compute
			 * IP header checksum in software since the
			 * stack left ip_sum=0 relying on CSUM_IP. */
			if (m0->m_pkthdr.csum_flags & CSUM_IP) {
				struct ether_header *eh2;
				struct ip *ip2;
				uint16_t et;
				int ehl;

				eh2 = mtod(m0, struct ether_header *);
				et = ntohs(eh2->ether_type);
				ehl = sizeof(*eh2);
				if (et == ETHERTYPE_VLAN)
					ehl += 4;
				if (m0->m_len >= ehl + (int)sizeof(*ip2)) {
					ip2 = (struct ip *)
					    (mtod(m0, char *) + ehl);
					ip2->ip_sum = 0;
					ip2->ip_sum = in_cksum_hdr(ip2);
				}
			}
			m0->m_pkthdr.csum_flags = 0;
			goto tx_sg_path;
		}

		/* Allocate SG table buffer: [prefix | SG entries] */
		sgt_buf = uma_zalloc(sc->sc_sgt_zone, M_NOWAIT);
		if (sgt_buf == NULL) {
			sched_unpin();
			dtsec_rm_fi_free(sc, fi);
			m_freem(m0);
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			return (ENOBUFS);
		}
		memset(sgt_buf, 0, sc->sc_sgt_buf_size);

		/* Store fi pointer in private data area (offset 0) */
		*(struct dtsec_rm_frame_info **)sgt_buf = fi;

		/* Copy parse result into prefix */
		buf_prs = FM_PORT_GetBufferPrsResult(
		    sc->sc_txph, (char *)sgt_buf);
		if (buf_prs != NULL)
			memcpy(buf_prs, &prs, sizeof(prs));

		/* Build SG entries at sgt_buf + tx_data_offset */
		sgt = (t_DpaaSGTE *)((char *)sgt_buf +
		    sc->sc_tx_data_offset);
		i = 0;
		psize = 0;
		for (m = m0; m != NULL; m = m->m_next) {
			if (m->m_len == 0)
				continue;
			dsize = m->m_len;
			vaddr = (vm_offset_t)m->m_data;
			while (dsize > 0 &&
			    i < DPAA_NUM_OF_SG_TABLE_ENTRY) {
				ssize = PAGE_SIZE -
				    (vaddr & PAGE_MASK);
				if (ssize > dsize)
					ssize = dsize;

				DPAA_SGTE_SET_ADDR(&sgt[i],
				    (void *)vaddr);
				DPAA_SGTE_SET_LENGTH(&sgt[i], ssize);
				DPAA_SGTE_SET_EXTENSION(&sgt[i], 0);
				DPAA_SGTE_SET_FINAL(&sgt[i], 0);
				DPAA_SGTE_SET_BPID(&sgt[i], 0);
				DPAA_SGTE_SET_OFFSET(&sgt[i], 0);

				dsize -= ssize;
				vaddr += ssize;
				psize += ssize;
				i++;
			}
			if (dsize > 0)
				break;
		}

		if (m != NULL || i == 0) {
			/* SG table overflow or empty — drop */
			sched_unpin();
			uma_zfree(sc->sc_sgt_zone, sgt_buf);
			dtsec_rm_fi_free(sc, fi);
			m_freem(m0);
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			return (EIO);
		}

		DPAA_SGTE_SET_FINAL(&sgt[i - 1], 1);

		fi->fi_mbuf = m0;
		fi->fi_tx_buf = sgt_buf;

		memset(&fd, 0, sizeof(fd));
		DPAA_FD_SET_ADDR(&fd, sgt_buf);
		DPAA_FD_SET_LENGTH(&fd, psize);
		DPAA_FD_SET_FORMAT(&fd,
		    e_DPAA_FD_FORMAT_TYPE_SHORT_MBSF);
		DPAA_FD_SET_OFFSET(&fd, sc->sc_tx_data_offset);
		DPAA_FD_SET_STATUS(&fd,
		    FM_FD_CMD_RPD | FM_FD_CMD_DTC);

		if (ceetm_fqid)
			error = qman_enqueue_fqid(ceetm_fqid, &fd);
		else
			error = qman_fqr_enqueue(sc->sc_tx_fqs[cpu],
			    0, &fd);
		if (__predict_false(error != E_OK)) {
			sched_unpin();
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			uma_zfree(sc->sc_sgt_zone, fi->fi_tx_buf);
			dtsec_rm_fi_free(sc, fi);
			m_freem(m0);
			return (EIO);
		}

		atomic_add_int(&sc->sc_tx_inflight[cpu], 1);
		sched_unpin();
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
		if_inc_counter(ifp, IFCOUNTER_OBYTES, psize);
		return (0);
	}
tx_sg_path:

	/*
	 * Legacy SG path (no checksum offload).
	 * First SG entry stores fi self-reference for TX confirm.
	 */
	i = 0;
	psize = 0;
	fi->fi_mbuf = m0;

	/* SGT[0]: fi pointer (metadata, zero-length) */
	DPAA_SGTE_SET_ADDR(&fi->fi_sgt[0], (void *)fi);
	DPAA_SGTE_SET_LENGTH(&fi->fi_sgt[0], 0);
	DPAA_SGTE_SET_EXTENSION(&fi->fi_sgt[0], 0);
	DPAA_SGTE_SET_FINAL(&fi->fi_sgt[0], 0);
	DPAA_SGTE_SET_BPID(&fi->fi_sgt[0], 0);
	DPAA_SGTE_SET_OFFSET(&fi->fi_sgt[0], 0);
	i = 1;

	/* SGT[1..N]: data entries from mbuf chain */
	for (m = m0; m != NULL && i < DPAA_NUM_OF_SG_TABLE_ENTRY;
	    m = m->m_next) {
		if (m->m_len == 0)
			continue;

		dsize = m->m_len;
		vaddr = (vm_offset_t)m->m_data;
		while (dsize > 0 && i < DPAA_NUM_OF_SG_TABLE_ENTRY) {
			ssize = PAGE_SIZE - (vaddr & PAGE_MASK);
			if (ssize > dsize)
				ssize = dsize;

			DPAA_SGTE_SET_ADDR(&fi->fi_sgt[i],
			    (void *)vaddr);
			DPAA_SGTE_SET_LENGTH(&fi->fi_sgt[i], ssize);
			DPAA_SGTE_SET_EXTENSION(&fi->fi_sgt[i], 0);
			DPAA_SGTE_SET_FINAL(&fi->fi_sgt[i], 0);
			DPAA_SGTE_SET_BPID(&fi->fi_sgt[i], 0);
			DPAA_SGTE_SET_OFFSET(&fi->fi_sgt[i], 0);

			dsize -= ssize;
			vaddr += ssize;
			psize += ssize;
			i++;
		}

		if (dsize > 0)
			break;
	}

	/* Check if SG table was constructed properly */
	if (m != NULL || psize == 0) {
		sched_unpin();
		dtsec_rm_fi_free(sc, fi);
		m_freem(m0);
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		return (EIO);
	}

	DPAA_SGTE_SET_FINAL(&fi->fi_sgt[i - 1], 1);

	memset(&fd, 0, sizeof(fd));
	DPAA_FD_SET_ADDR(&fd, fi->fi_sgt);
	DPAA_FD_SET_LENGTH(&fd, psize);
	DPAA_FD_SET_FORMAT(&fd, e_DPAA_FD_FORMAT_TYPE_SHORT_MBSF);
	DPAA_FD_SET_OFFSET(&fd, 0);
	DPAA_FD_SET_STATUS(&fd, 0);

	if (ceetm_fqid)
		error = qman_enqueue_fqid(ceetm_fqid, &fd);
	else
		error = qman_fqr_enqueue(sc->sc_tx_fqs[cpu], 0, &fd);
	if (__predict_false(error != E_OK)) {
		sched_unpin();
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		dtsec_rm_fi_free(sc, fi);
		m_freem(m0);
		return (EIO);
	}

	atomic_add_int(&sc->sc_tx_inflight[cpu], 1);
	sched_unpin();
	if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	if_inc_counter(ifp, IFCOUNTER_OBYTES, psize);
	return (0);
}
/** @} */
