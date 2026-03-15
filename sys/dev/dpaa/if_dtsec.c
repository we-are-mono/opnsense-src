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
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/sockio.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_arp.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/iicbus/iic.h>
#include <dev/iicbus/iiconf.h>

#include "miibus_if.h"

#include "opt_dpaa.h"

#include <contrib/ncsw/inc/integrations/dpaa_integration_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_mac_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_port_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_vsp_ext.h>
#if (DPAA_VERSION < 11)
#include <contrib/ncsw/inc/flib/fsl_fman_dtsec.h>
#endif
#include <contrib/ncsw/inc/xx_ext.h>

#include "fman.h"
#include "bman.h"

/* FM port internal header — needed for RCCB register access in stall diag */
#include <contrib/ncsw/Peripherals/FM/Port/fm_port.h>
#include "qman.h"
#include "if_dtsec.h"
#include "if_dtsec_im.h"
#include "if_dtsec_rm.h"

#include <dev/sff/sfp_fdt.h>

#define	DTSEC_MIN_FRAME_SIZE	64
#define	DTSEC_MAX_FRAME_SIZE	9600

#if (DPAA_VERSION < 11)
#define	DTSEC_REG_MAXFRM	0x110
#define	DTSEC_REG_GADDR(i)	(0x0a0 + 4*(i))
#else
#define	MEMAC_REG_MAXFRM	0x014
#endif

/**
 * @group dTSEC private defines.
 * @{
 */
/**
 * dTSEC FMan MAC exceptions info struct.
 */
struct dtsec_fm_mac_ex_str {
	const int num;
	const char *str;
};
/** @} */


/**
 * @group FMan MAC routines.
 * @{
 */
#define	DTSEC_MAC_EXCEPTIONS_END	(-1)

/**
 * FMan MAC exceptions.
 */
static const struct dtsec_fm_mac_ex_str dtsec_fm_mac_exceptions[] = {
	{ e_FM_MAC_EX_10G_MDIO_SCAN_EVENTMDIO, "MDIO scan event" },
	{ e_FM_MAC_EX_10G_MDIO_CMD_CMPL, "MDIO command completion" },
	{ e_FM_MAC_EX_10G_REM_FAULT, "Remote fault" },
	{ e_FM_MAC_EX_10G_LOC_FAULT, "Local fault" },
	{ e_FM_MAC_EX_10G_1TX_ECC_ER, "Transmit frame ECC error" },
	{ e_FM_MAC_EX_10G_TX_FIFO_UNFL, "Transmit FIFO underflow" },
	{ e_FM_MAC_EX_10G_TX_FIFO_OVFL, "Receive FIFO overflow" },
	{ e_FM_MAC_EX_10G_TX_ER, "Transmit frame error" },
	{ e_FM_MAC_EX_10G_RX_FIFO_OVFL, "Receive FIFO overflow" },
	{ e_FM_MAC_EX_10G_RX_ECC_ER, "Receive frame ECC error" },
	{ e_FM_MAC_EX_10G_RX_JAB_FRM, "Receive jabber frame" },
	{ e_FM_MAC_EX_10G_RX_OVRSZ_FRM, "Receive oversized frame" },
	{ e_FM_MAC_EX_10G_RX_RUNT_FRM, "Receive runt frame" },
	{ e_FM_MAC_EX_10G_RX_FRAG_FRM, "Receive fragment frame" },
	{ e_FM_MAC_EX_10G_RX_LEN_ER, "Receive payload length error" },
	{ e_FM_MAC_EX_10G_RX_CRC_ER, "Receive CRC error" },
	{ e_FM_MAC_EX_10G_RX_ALIGN_ER, "Receive alignment error" },
	{ e_FM_MAC_EX_1G_BAB_RX, "Babbling receive error" },
	{ e_FM_MAC_EX_1G_RX_CTL, "Receive control (pause frame) interrupt" },
	{ e_FM_MAC_EX_1G_GRATEFUL_TX_STP_COMPLET, "Graceful transmit stop "
	    "complete" },
	{ e_FM_MAC_EX_1G_BAB_TX, "Babbling transmit error" },
	{ e_FM_MAC_EX_1G_TX_CTL, "Transmit control (pause frame) interrupt" },
	{ e_FM_MAC_EX_1G_TX_ERR, "Transmit error" },
	{ e_FM_MAC_EX_1G_LATE_COL, "Late collision" },
	{ e_FM_MAC_EX_1G_COL_RET_LMT, "Collision retry limit" },
	{ e_FM_MAC_EX_1G_TX_FIFO_UNDRN, "Transmit FIFO underrun" },
	{ e_FM_MAC_EX_1G_MAG_PCKT, "Magic Packet detected when dTSEC is in "
	    "Magic Packet detection mode" },
	{ e_FM_MAC_EX_1G_MII_MNG_RD_COMPLET, "MII management read completion" },
	{ e_FM_MAC_EX_1G_MII_MNG_WR_COMPLET, "MII management write completion" },
	{ e_FM_MAC_EX_1G_GRATEFUL_RX_STP_COMPLET, "Graceful receive stop "
	    "complete" },
	{ e_FM_MAC_EX_1G_TX_DATA_ERR, "Internal data error on transmit" },
	{ e_FM_MAC_EX_1G_RX_DATA_ERR, "Internal data error on receive" },
	{ e_FM_MAC_EX_1G_1588_TS_RX_ERR, "Time-Stamp Receive Error" },
	{ e_FM_MAC_EX_1G_RX_MIB_CNT_OVFL, "MIB counter overflow" },
	{ DTSEC_MAC_EXCEPTIONS_END, "" }
};

static const char *
dtsec_fm_mac_ex_to_str(e_FmMacExceptions exception)
{
	int i;

	for (i = 0; dtsec_fm_mac_exceptions[i].num != exception &&
	    dtsec_fm_mac_exceptions[i].num != DTSEC_MAC_EXCEPTIONS_END; ++i)
		;

	if (dtsec_fm_mac_exceptions[i].num == DTSEC_MAC_EXCEPTIONS_END)
		return ("<Unknown Exception>");

	return (dtsec_fm_mac_exceptions[i].str);
}

static void
dtsec_fm_mac_mdio_event_callback(t_Handle h_App,
    e_FmMacExceptions exception)
{
	struct dtsec_softc *sc;

	sc = h_App;
	if_printf(sc->sc_ifnet, "MDIO event %i: %s.\n", exception,
	    dtsec_fm_mac_ex_to_str(exception));
}

static void
dtsec_fm_mac_exception_callback(t_Handle app, e_FmMacExceptions exception)
{
	struct dtsec_softc *sc;

	sc = app;
	if_printf(sc->sc_ifnet, "MAC exception %i: %s.\n", exception,
	    dtsec_fm_mac_ex_to_str(exception));
}

static void
dtsec_fm_mac_free(struct dtsec_softc *sc)
{
	if (sc->sc_mach == NULL)
		return;

	FM_MAC_Disable(sc->sc_mach, e_COMM_MODE_RX_AND_TX);
	FM_MAC_Free(sc->sc_mach);
	sc->sc_mach = NULL;
}

static int
dtsec_fm_mac_init(struct dtsec_softc *sc, uint8_t *mac)
{
	t_FmMacParams params;
	t_Error error;

	memset(&params, 0, sizeof(params));
	memcpy(&params.addr, mac, sizeof(params.addr));

	params.baseAddr = rman_get_bushandle(sc->sc_mem);
	params.enetMode = sc->sc_mac_enet_mode;
	params.macId = sc->sc_eth_id;
	params.mdioIrq = sc->sc_mac_mdio_irq;
	params.f_Event = dtsec_fm_mac_mdio_event_callback;
	params.f_Exception = dtsec_fm_mac_exception_callback;
	params.h_App = sc;
	params.h_Fm = sc->sc_fmh;

	sc->sc_mach = FM_MAC_Config(&params);
	if (sc->sc_mach == NULL) {
		device_printf(sc->sc_dev, "couldn't configure FM_MAC module.\n"
		    );
		return (ENXIO);
	}

	error = FM_MAC_ConfigResetOnInit(sc->sc_mach, TRUE);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "couldn't enable reset on init "
		    "feature.\n");
		dtsec_fm_mac_free(sc);
		return (ENXIO);
	}

#if (DPAA_VERSION < 11)
	/* Do not inform about pause frames (dTSEC only; MEMAC has no
	 * equivalent interrupt — it simply never generates this event). */
	error = FM_MAC_ConfigException(sc->sc_mach, e_FM_MAC_EX_1G_RX_CTL,
	    FALSE);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "couldn't disable pause frames "
			"exception.\n");
		dtsec_fm_mac_free(sc);
		return (ENXIO);
	}
#endif

	error = FM_MAC_Init(sc->sc_mach);
	if (error != E_OK) {
		device_printf(sc->sc_dev, "couldn't initialize FM_MAC module."
		    "\n");
		dtsec_fm_mac_free(sc);
		return (ENXIO);
	}

	/* Diagnostic: dump MEMAC IF_MODE and IF_STATUS registers */
	{
		uint32_t ifmode = be32toh(bus_read_4(sc->sc_mem, 0x300));
		uint32_t ifstat = be32toh(bus_read_4(sc->sc_mem, 0x304));
		device_printf(sc->sc_dev,
		    "MEMAC IF_MODE=0x%08x IF_STATUS=0x%08x mode=%s\n",
		    ifmode, ifstat,
		    (ifmode & 0x3) == 0 ? "XGMII(10G)" :
		    (ifmode & 0x3) == 2 ? "GMII(1G)" :
		    (ifmode & 0x3) == 4 ? "RGMII" : "unknown");
	}

	return (0);
}
/** @} */


/**
 * @group FMan PORT routines.
 * @{
 */
static const char *
dtsec_fm_port_ex_to_str(e_FmPortExceptions exception)
{

	switch (exception) {
	case e_FM_PORT_EXCEPTION_IM_BUSY:
		return ("IM: RX busy");
	default:
		return ("<Unknown Exception>");
	}
}

void
dtsec_fm_port_rx_exception_callback(t_Handle app,
    e_FmPortExceptions exception)
{
	struct dtsec_softc *sc;

	sc = app;
	if_printf(sc->sc_ifnet, "RX exception: %i: %s.\n", exception,
	    dtsec_fm_port_ex_to_str(exception));
}

void
dtsec_fm_port_tx_exception_callback(t_Handle app,
    e_FmPortExceptions exception)
{
	struct dtsec_softc *sc;

	sc = app;
	if_printf(sc->sc_ifnet, "TX exception: %i: %s.\n", exception,
	    dtsec_fm_port_ex_to_str(exception));
}

e_FmPortType
dtsec_fm_port_rx_type(enum eth_dev_type type)
{
	switch (type) {
	case ETH_DTSEC:
		return (e_FM_PORT_TYPE_RX);
	case ETH_10GSEC:
		return (e_FM_PORT_TYPE_RX_10G);
	default:
		return (e_FM_PORT_TYPE_DUMMY);
	}
}

e_FmPortType
dtsec_fm_port_tx_type(enum eth_dev_type type)
{

	switch (type) {
	case ETH_DTSEC:
		return (e_FM_PORT_TYPE_TX);
	case ETH_10GSEC:
		return (e_FM_PORT_TYPE_TX_10G);
	default:
		return (e_FM_PORT_TYPE_DUMMY);
	}
}

static void
dtsec_fm_port_free_both(struct dtsec_softc *sc)
{
	if (sc->sc_vsph) {
		FM_VSP_Free(sc->sc_vsph);
		sc->sc_vsph = NULL;
	}

	if (sc->sc_rxph) {
		FM_PORT_Free(sc->sc_rxph);
		sc->sc_rxph = NULL;
	}

	if (sc->sc_txph) {
		FM_PORT_Free(sc->sc_txph);
		sc->sc_txph = NULL;
	}
}
/** @} */


/**
 * @group IFnet routines.
 * @{
 */
static int
dtsec_set_mtu(struct dtsec_softc *sc, unsigned int mtu)
{

	mtu += ETHER_HDR_LEN + ETHER_VLAN_ENCAP_LEN + ETHER_CRC_LEN;

	DTSEC_LOCK_ASSERT(sc);

	if (mtu >= DTSEC_MIN_FRAME_SIZE && mtu <= DTSEC_MAX_FRAME_SIZE) {
#if (DPAA_VERSION < 11)
		bus_write_4(sc->sc_mem, DTSEC_REG_MAXFRM, mtu);
#else
		bus_write_4(sc->sc_mem, MEMAC_REG_MAXFRM,
		    htobe32((uint32_t)mtu));
#endif
		return (mtu);
	}

	return (0);
}

static u_int
dtsec_hash_maddr(void *arg, struct sockaddr_dl *sdl, u_int cnt)
{
	struct dtsec_softc *sc = arg;

	FM_MAC_AddHashMacAddr(sc->sc_mach, (t_EnetAddr *)LLADDR(sdl));

	return (1);
}

static void
dtsec_setup_multicast(struct dtsec_softc *sc)
{
#if (DPAA_VERSION < 11)
	int i;

	if (if_getflags(sc->sc_ifnet) & IFF_ALLMULTI) {
		for (i = 0; i < 8; i++)
			bus_write_4(sc->sc_mem, DTSEC_REG_GADDR(i), 0xFFFFFFFF);

		return;
	}

	fman_dtsec_reset_filter_table(rman_get_virtual(sc->sc_mem),
	    true, false);
#else
	/* mEMAC: no hash reset API — SetPromiscuous(false) clears filter */
	FM_MAC_SetPromiscuous(sc->sc_mach, false);
#endif
	if_foreach_llmaddr(sc->sc_ifnet, dtsec_hash_maddr, sc);
}

static void	dtsec_if_tick(void *arg);

static int
dtsec_if_enable_locked(struct dtsec_softc *sc)
{
	int error;

	DTSEC_LOCK_ASSERT(sc);

	error = FM_MAC_Enable(sc->sc_mach, e_COMM_MODE_RX_AND_TX);
	if (error != E_OK)
		return (EIO);

	error = FM_PORT_Enable(sc->sc_rxph);
	if (error != E_OK)
		return (EIO);

	error = FM_PORT_Enable(sc->sc_txph);
	if (error != E_OK)
		return (EIO);

	/* Reprogram hardware MAC address from current stack address.
	 * LAGG changes the stack MAC via if_setlladdr() then cycles
	 * SIOCSIFFLAGS, which reaches here — not dtsec_if_init_locked().
	 * Safe to call unconditionally; no-op if address hasn't changed. */
	error = FM_MAC_ModifyMacAddr(sc->sc_mach,
	    (t_EnetAddr *)if_getlladdr(sc->sc_ifnet));
	if (error != E_OK)
		return (EIO);

	dtsec_setup_multicast(sc);

	if_setdrvflagbits(sc->sc_ifnet, IFF_DRV_RUNNING, 0);

	/* Start MII tick if not already running.
	 * dtsec_if_init_locked() starts this, but LAGG member ports
	 * are enabled via SIOCSIFFLAGS → dtsec_if_enable_locked()
	 * without going through if_init(), so the tick never starts.
	 * callout_reset is idempotent — safe to call if already running. */
	if (sc->sc_mii != NULL)
		callout_reset(&sc->sc_tick_callout, hz, dtsec_if_tick, sc);

	/* Refresh link state */
	dtsec_miibus_statchg(sc->sc_dev);

	return (0);
}

static int
dtsec_if_disable_locked(struct dtsec_softc *sc)
{
	int error;

	DTSEC_LOCK_ASSERT(sc);

	error = FM_MAC_Disable(sc->sc_mach, e_COMM_MODE_RX_AND_TX);
	if (error != E_OK)
		return (EIO);

	error = FM_PORT_Disable(sc->sc_rxph);
	if (error != E_OK)
		return (EIO);

	error = FM_PORT_Disable(sc->sc_txph);
	if (error != E_OK)
		return (EIO);

	if_setdrvflagbits(sc->sc_ifnet, 0, IFF_DRV_RUNNING);

	return (0);
}

static int
dtsec_if_ioctl(if_t ifp, u_long command, caddr_t data)
{
	struct dtsec_softc *sc;
	struct ifreq *ifr;
	int error;

	sc = if_getsoftc(ifp);
	ifr = (struct ifreq *)data;
	error = 0;

	/* Basic functionality to achieve media status reports */
	switch (command) {
	case SIOCSIFMTU:
		DTSEC_LOCK(sc);
		if (dtsec_set_mtu(sc, ifr->ifr_mtu))
			if_setmtu(ifp, ifr->ifr_mtu);
		else
			error = EINVAL;
		DTSEC_UNLOCK(sc);
		break;
	case SIOCSIFFLAGS:
		DTSEC_LOCK(sc);

		if (if_getflags(sc->sc_ifnet) & IFF_UP)
			error = dtsec_if_enable_locked(sc);
		else
			error = dtsec_if_disable_locked(sc);

		DTSEC_UNLOCK(sc);
		break;

	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		if (sc->sc_mii != NULL)
			error = ifmedia_ioctl(ifp, ifr,
			    &sc->sc_mii->mii_media, command);
		else
			error = ENOTTY;
		break;

	case SIOCSIFCAP: {
		int mask;

		mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);

		if (mask & IFCAP_TXCSUM) {
			if_togglecapenable(ifp, IFCAP_TXCSUM);
			if (if_getcapenable(ifp) & IFCAP_TXCSUM)
				if_sethwassist(ifp, if_gethwassist(ifp) |
				    (CSUM_IP | CSUM_IP_TCP | CSUM_IP_UDP));
			else
				if_sethwassist(ifp, if_gethwassist(ifp) &
				    ~(CSUM_IP | CSUM_IP_TCP | CSUM_IP_UDP));
		}
		if (mask & IFCAP_TXCSUM_IPV6) {
			if_togglecapenable(ifp, IFCAP_TXCSUM_IPV6);
			if (if_getcapenable(ifp) & IFCAP_TXCSUM_IPV6)
				if_sethwassist(ifp, if_gethwassist(ifp) |
				    (CSUM_IP6_TCP | CSUM_IP6_UDP));
			else
				if_sethwassist(ifp, if_gethwassist(ifp) &
				    ~(CSUM_IP6_TCP | CSUM_IP6_UDP));
		}
		break;
	}

	default:
		error = ether_ioctl(ifp, command, data);
	}

	return (error);
}

static void
dtsec_if_tick(void *arg)
{
	struct dtsec_softc *sc;

	sc = arg;

	DTSEC_LOCK(sc);

	if (sc->sc_mii != NULL)
		mii_tick(sc->sc_mii);

	callout_reset(&sc->sc_tick_callout, hz, dtsec_if_tick, sc);

	DTSEC_UNLOCK(sc);
}

static void
dtsec_if_deinit_locked(struct dtsec_softc *sc)
{

	DTSEC_LOCK_ASSERT(sc);

	DTSEC_UNLOCK(sc);
	callout_drain(&sc->sc_tick_callout);
	DTSEC_LOCK(sc);
}

static void
dtsec_if_init_locked(struct dtsec_softc *sc)
{
	int error;

	DTSEC_LOCK_ASSERT(sc);

	/* Set MAC address */
	error = FM_MAC_ModifyMacAddr(sc->sc_mach,
	    (t_EnetAddr *)if_getlladdr(sc->sc_ifnet));
	if (error != E_OK) {
		if_printf(sc->sc_ifnet, "couldn't set MAC address.\n");
		goto err;
	}

	/* Start MII polling / periodic diagnostics */
	callout_reset(&sc->sc_tick_callout, hz, dtsec_if_tick, sc);

	if (if_getflags(sc->sc_ifnet) & IFF_UP) {
		error = dtsec_if_enable_locked(sc);
		if (error != 0)
			goto err;
	} else {
		error = dtsec_if_disable_locked(sc);
		if (error != 0)
			goto err;
	}

	return;

err:
	dtsec_if_deinit_locked(sc);
	if_printf(sc->sc_ifnet, "initialization error.\n");
	return;
}

static void
dtsec_if_init(void *data)
{
	struct dtsec_softc *sc;

	sc = data;

	DTSEC_LOCK(sc);
	dtsec_if_init_locked(sc);
	DTSEC_UNLOCK(sc);
}

static void
dtsec_if_start(if_t ifp)
{
	struct dtsec_softc *sc;

	sc = if_getsoftc(ifp);
	DTSEC_LOCK(sc);
	sc->sc_start_locked(sc);
	DTSEC_UNLOCK(sc);
}

static void
dtsec_if_qflush(if_t ifp)
{
	/* No software queue — nothing to flush */
}

static void
dtsec_if_watchdog(if_t ifp)
{
	/* TODO */
}
/** @} */


/**
 * @group SFP upstream callbacks (from sfp_fdt state machine).
 * @{
 */

static void
dtsec_sfp_trim(char *dst, const uint8_t *src, int len)
{
	int i;

	memcpy(dst, src, len);
	dst[len] = '\0';
	for (i = len - 1; i >= 0 && dst[i] == ' '; i--)
		dst[i] = '\0';
}

static int
dtsec_sfp_module_insert(void *arg, const uint8_t *id, int id_len)
{
	struct dtsec_softc *sc = arg;

	memcpy(sc->sc_sfp_id, id, MIN((int)sizeof(sc->sc_sfp_id), id_len));
	sc->sc_sfp_modpresent = true;

	return (0);	/* accept all modules */
}

static void
dtsec_sfp_module_remove(void *arg)
{
	struct dtsec_softc *sc = arg;

	memset(sc->sc_sfp_id, 0, sizeof(sc->sc_sfp_id));
	sc->sc_sfp_modpresent = false;
	sc->sc_sfp_phy_link = false;
	sc->sc_sfp_phy_speed = 0;
	if_link_state_change(sc->sc_ifnet, LINK_STATE_UNKNOWN);
}

static void
dtsec_sfp_link_up(void *arg, int speed)
{
	struct dtsec_softc *sc = arg;

	sc->sc_sfp_phy_link = true;
	sc->sc_sfp_phy_speed = speed;
	if_link_state_change(sc->sc_ifnet, LINK_STATE_UP);
}

static void
dtsec_sfp_link_down(void *arg)
{
	struct dtsec_softc *sc = arg;

	sc->sc_sfp_phy_link = false;
	sc->sc_sfp_phy_speed = 0;
	if_link_state_change(sc->sc_ifnet, LINK_STATE_DOWN);
}

const struct sfp_upstream_ops dtsec_sfp_ops = {
	.module_insert	= dtsec_sfp_module_insert,
	.module_remove	= dtsec_sfp_module_remove,
	.link_up	= dtsec_sfp_link_up,
	.link_down	= dtsec_sfp_link_down,
};
/** @} */


/**
 * @group IFmedia routines.
 * @{
 */
static int
dtsec_ifmedia_upd(if_t ifp)
{
	struct dtsec_softc *sc = if_getsoftc(ifp);

	DTSEC_LOCK(sc);
	if (sc->sc_mii != NULL)
		mii_mediachg(sc->sc_mii);
	DTSEC_UNLOCK(sc);

	return (0);
}

static void
dtsec_ifmedia_sts(if_t ifp, struct ifmediareq *ifmr)
{
	struct dtsec_softc *sc = if_getsoftc(ifp);

	DTSEC_LOCK(sc);

	if (sc->sc_mii != NULL) {
		mii_pollstat(sc->sc_mii);
		ifmr->ifm_active = sc->sc_mii->mii_media_active;
		ifmr->ifm_status = sc->sc_mii->mii_media_status;
	} else if (sc->sc_sfp_dev != NULL) {
		/* SFP+ — link state from sfp_fdt callbacks */
		ifmr->ifm_active = IFM_ETHER | IFM_FDX;
		if (sc->sc_sfp_id[SFP_CONNECTOR_OFFSET] ==
		    SFP_CONNECTOR_RJ45) {
			/* Copper module — report speed */
			switch (sc->sc_sfp_phy_speed) {
			case 10000: ifmr->ifm_active |= IFM_10G_T; break;
			case 5000:  ifmr->ifm_active |= IFM_5000_T; break;
			case 2500:  ifmr->ifm_active |= IFM_2500_T; break;
			case 1000:  ifmr->ifm_active |= IFM_1000_T; break;
			default:    ifmr->ifm_active |= IFM_10G_T; break;
			}
		} else {
			ifmr->ifm_active |= IFM_10G_SR;
		}
		if (sc->sc_sfp_phy_link)
			ifmr->ifm_status = IFM_AVALID | IFM_ACTIVE;
		else
			ifmr->ifm_status = IFM_AVALID;
	} else {
		/* 10G port with no SFP framework — legacy always-up */
		ifmr->ifm_active = IFM_ETHER | IFM_10G_SR | IFM_FDX;
		ifmr->ifm_status = IFM_AVALID | IFM_ACTIVE;
	}

	DTSEC_UNLOCK(sc);
}
/** @} */


/**
 * @group dTSEC bus interface.
 * @{
 */
static void
dtsec_configure_mode(struct dtsec_softc *sc)
{
	char tunable[64];

	snprintf(tunable, sizeof(tunable), "%s.independent_mode",
	    device_get_nameunit(sc->sc_dev));

	sc->sc_mode = DTSEC_MODE_REGULAR;
	TUNABLE_INT_FETCH(tunable, &sc->sc_mode);

	if (sc->sc_mode == DTSEC_MODE_REGULAR) {
		sc->sc_port_rx_init = dtsec_rm_fm_port_rx_init;
		sc->sc_port_tx_init = dtsec_rm_fm_port_tx_init;
		/* RM uses if_transmit, no sc_start_locked needed */
	} else {
		sc->sc_port_rx_init = dtsec_im_fm_port_rx_init;
		sc->sc_port_tx_init = dtsec_im_fm_port_tx_init;
		sc->sc_start_locked = dtsec_im_if_start_locked;
	}

	device_printf(sc->sc_dev, "Configured for %s mode.\n",
	    (sc->sc_mode == DTSEC_MODE_REGULAR) ? "regular" : "independent");
}

static int
dtsec_sysctl_bman_free(SYSCTL_HANDLER_ARGS)
{
	struct dtsec_softc *sc = (struct dtsec_softc *)arg1;
	uint32_t count;

	if (sc->sc_rx_pool == NULL)
		count = 0;
	else
		count = bman_count(sc->sc_rx_pool);

	return (sysctl_handle_int(oidp, &count, 0, req));
}

static int
dtsec_sysctl_diag(SYSCTL_HANDLER_ARGS)
{
	struct dtsec_softc *sc = (struct dtsec_softc *)arg1;
	int val = 0;
	int error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	if (val != 0) {
		uint32_t bcount = sc->sc_rx_pool ?
		    bman_count(sc->sc_rx_pool) : 0;
		printf("%s: DIAG bman_free=%u bman_total=%u bpid=%u "
		    "rx_fqid=%u\n",
		    if_name(sc->sc_ifnet), bcount,
		    sc->sc_rx_buf_total, sc->sc_rx_bpid,
		    sc->sc_rx_fqr[0] ? qman_fqr_get_base_fqid(sc->sc_rx_fqr[0]) : 0);
		qman_portal_dqrr_diag();
		if (sc->sc_rxph != NULL) {
			printf("%s: FMan RX: frame=%u discard=%u "
			    "bad=%u filter=%u oobd=%u enq=%u\n",
			    if_name(sc->sc_ifnet),
			    FM_PORT_GetCounter(sc->sc_rxph,
			        e_FM_PORT_COUNTERS_FRAME),
			    FM_PORT_GetCounter(sc->sc_rxph,
			        e_FM_PORT_COUNTERS_DISCARD_FRAME),
			    FM_PORT_GetCounter(sc->sc_rxph,
			        e_FM_PORT_COUNTERS_RX_BAD_FRAME),
			    FM_PORT_GetCounter(sc->sc_rxph,
			        e_FM_PORT_COUNTERS_RX_FILTER_FRAME),
			    FM_PORT_GetCounter(sc->sc_rxph,
			        e_FM_PORT_COUNTERS_RX_OUT_OF_BUFFERS_DISCARD),
			    FM_PORT_GetCounter(sc->sc_rxph,
			        e_FM_PORT_COUNTERS_ENQ_TOTAL));
		}
		if (sc->sc_txph != NULL) {
			printf("%s: FMan TX: frame=%u discard=%u "
			    "len_err=%u unsup=%u deq=%u deq_dflt=%u "
			    "deq_conf=%u\n",
			    if_name(sc->sc_ifnet),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_FRAME),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_DISCARD_FRAME),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_LENGTH_ERR),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_UNSUPPRTED_FORMAT),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_DEQ_TOTAL),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_DEQ_FROM_DEFAULT),
			    FM_PORT_GetCounter(sc->sc_txph,
			        e_FM_PORT_COUNTERS_DEQ_CONFIRM));
		}
		if (sc->sc_mach != NULL) {
			t_FmMacStatistics ms;
			if (FM_MAC_GetStatistics(sc->sc_mach, &ms) == E_OK) {
				printf("%s: MAC: rxPkts=%llu rxBytes=%llu "
				    "rxDiscard=%llu rxErr=%llu "
				    "txPkts=%llu txBytes=%llu "
				    "txErr=%llu\n",
				    if_name(sc->sc_ifnet),
				    (unsigned long long)ms.ifInPkts,
				    (unsigned long long)ms.ifInOctets,
				    (unsigned long long)ms.ifInDiscards,
				    (unsigned long long)ms.ifInErrors,
				    (unsigned long long)ms.ifOutPkts,
				    (unsigned long long)ms.ifOutOctets,
				    (unsigned long long)ms.ifOutErrors);
			}
		}
		/* MEMAC register dump for link debugging */
		if (sc->sc_mem != NULL) {
			uint32_t cmd_cfg = be32toh(bus_read_4(sc->sc_mem, 0x008));
			uint32_t ifmode = be32toh(bus_read_4(sc->sc_mem, 0x300));
			uint32_t ifstat = be32toh(bus_read_4(sc->sc_mem, 0x304));
			uint32_t ievent = be32toh(bus_read_4(sc->sc_mem, 0x040));
			printf("%s: MEMAC cmd_cfg=0x%08x [TX_%s RX_%s] "
			    "IF_MODE=0x%08x [%s] IF_STATUS=0x%08x "
			    "IEVENT=0x%08x\n",
			    if_name(sc->sc_ifnet), cmd_cfg,
			    (cmd_cfg & 0x01) ? "EN" : "DIS",
			    (cmd_cfg & 0x02) ? "EN" : "DIS",
			    ifmode,
			    (ifmode & 0x3) == 0 ? "XGMII/10G" :
			    (ifmode & 0x3) == 2 ? "GMII/1G" :
			    (ifmode & 0x3) == 4 ? "RGMII" : "unknown",
			    ifstat, ievent);
		}
		/* SFP state (managed by sfp_fdt) */
		if (sc->sc_sfp_dev != NULL) {
			printf("%s: SFP: module=%s link=%s speed=%d\n",
			    if_name(sc->sc_ifnet),
			    sc->sc_sfp_modpresent ? "present" : "absent",
			    sc->sc_sfp_phy_link ? "up" : "down",
			    sc->sc_sfp_phy_speed);
			/* Read DDM status byte (A2h byte 110 = 0x6E) */
			if (sc->sc_sfp_i2c != NULL &&
			    sc->sc_sfp_modpresent) {
				struct iic_msg msgs[2];
				uint8_t reg = 0x6E;
				uint8_t ddm_status;
				int err;
				msgs[0].slave = 0x51 << 1;
				msgs[0].flags = IIC_M_WR;
				msgs[0].len = 1;
				msgs[0].buf = &reg;
				msgs[1].slave = 0x51 << 1;
				msgs[1].flags = IIC_M_RD;
				msgs[1].len = 1;
				msgs[1].buf = &ddm_status;
				err = iicbus_request_bus(sc->sc_sfp_i2c,
				    sc->sc_dev, IIC_INTRWAIT);
				if (err == 0) {
					err = iicbus_transfer(
					    sc->sc_sfp_i2c, msgs, 2);
					iicbus_release_bus(sc->sc_sfp_i2c,
					    sc->sc_dev);
				}
				if (err == 0) {
					printf("%s: DDM byte110=0x%02x "
					    "[TX_DIS_STATE=%d SOFT_TXDIS=%d "
					    "TX_FAULT=%d RX_LOS=%d]\n",
					    if_name(sc->sc_ifnet),
					    ddm_status,
					    (ddm_status >> 7) & 1,
					    (ddm_status >> 6) & 1,
					    (ddm_status >> 2) & 1,
					    (ddm_status >> 1) & 1);
				}
			}
		}
	}

	return (0);
}

static int
dtsec_sysctl_sfp_info(SYSCTL_HANDLER_ARGS)
{
	struct dtsec_softc *sc = (struct dtsec_softc *)arg1;
	char buf[48];

	if (!sc->sc_sfp_modpresent) {
		strlcpy(buf, "empty", sizeof(buf));
	} else {
		char vendor[SFP_VENDOR_LEN + 1];
		char partnum[SFP_PARTNUM_LEN + 1];

		dtsec_sfp_trim(vendor, &sc->sc_sfp_id[SFP_VENDOR_OFFSET],
		    SFP_VENDOR_LEN);
		dtsec_sfp_trim(partnum, &sc->sc_sfp_id[SFP_PARTNUM_OFFSET],
		    SFP_PARTNUM_LEN);
		snprintf(buf, sizeof(buf), "%s %s", vendor, partnum);
	}

	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static uint64_t
dtsec_get_counter(if_t ifp, ift_counter cnt)
{
	struct dtsec_softc *sc = if_getsoftc(ifp);
	t_FmMacStatistics ms;

	if (sc->sc_mach != NULL &&
	    FM_MAC_GetStatistics(sc->sc_mach, &ms) == E_OK) {
		switch (cnt) {
		case IFCOUNTER_IPACKETS:
			return (ms.ifInPkts);
		case IFCOUNTER_OPACKETS:
			return (ms.ifOutPkts);
		case IFCOUNTER_IBYTES:
			return (ms.ifInOctets);
		case IFCOUNTER_OBYTES:
			return (ms.ifOutOctets);
		case IFCOUNTER_IERRORS:
			return (ms.ifInErrors);
		case IFCOUNTER_OERRORS:
			return (ms.ifOutErrors);
		case IFCOUNTER_IMCASTS:
			return (ms.ifInMcastPkts);
		default:
			break;
		}
	}
	return (if_get_counter_default(ifp, cnt));
}

int
dtsec_attach(device_t dev)
{
	struct dtsec_softc *sc;
	device_t parent;
	int error;
	if_t ifp;

	sc = device_get_softc(dev);

	parent = device_get_parent(dev);
	sc->sc_dev = dev;
	sc->sc_mac_mdio_irq = NO_IRQ;

	/* Check if MallocSmart allocator is ready */
	if (XX_MallocSmartInit() != E_OK)
		return (ENXIO);

	/* Init locks */
	mtx_init(&sc->sc_lock, device_get_nameunit(dev),
	    "DTSEC Global Lock", MTX_DEF);

	mtx_init(&sc->sc_mii_lock, device_get_nameunit(dev),
	    "DTSEC MII Lock", MTX_DEF);

	/* Init callouts */
	callout_init(&sc->sc_tick_callout, CALLOUT_MPSAFE);

	/* Read configuraton */
	if ((error = fman_get_handle(parent, &sc->sc_fmh)) != 0)
		return (error);

	if ((error = fman_get_muram_handle(parent, &sc->sc_muramh)) != 0)
		return (error);

	if ((error = fman_get_bushandle(parent, &sc->sc_fm_base)) != 0)
		return (error);

	fman_get_pcd_handle(parent, &sc->sc_pcdh);
	fman_get_netenv_handle(parent, &sc->sc_netenvh);

	/* Configure working mode */
	dtsec_configure_mode(sc);

	/* If we are working in regular mode configure BMAN and QMAN */
	if (sc->sc_mode == DTSEC_MODE_REGULAR) {
		/* Create RX buffer pool */
		error = dtsec_rm_pool_rx_init(sc);
		if (error != 0)
			return (EIO);

		/* Create RX frame queue range */
		error = dtsec_rm_fqr_rx_init(sc);
		if (error != 0)
			return (EIO);

		/* Create frame info pool */
		error = dtsec_rm_fi_pool_init(sc);
		if (error != 0)
			return (EIO);

		/* Create TX frame queue range */
		error = dtsec_rm_fqr_tx_init(sc);
		if (error != 0)
			return (EIO);
	}

	/* Init FMan MAC module. */
	error = dtsec_fm_mac_init(sc, sc->sc_mac_addr);
	if (error != 0) {
		dtsec_detach(dev);
		return (ENXIO);
	}

	/* Init FMan TX port */
	error = sc->sc_port_tx_init(sc, device_get_unit(sc->sc_dev));
	if (error != 0) {
		dtsec_detach(dev);
		return (ENXIO);
	}

	/* Init FMan RX port */
	error = sc->sc_port_rx_init(sc, device_get_unit(sc->sc_dev));
	if (error != 0) {
		dtsec_detach(dev);
		return (ENXIO);
	}

	if (sc->sc_mode == DTSEC_MODE_REGULAR) {
		error = dtsec_rm_pcd_init(sc);
		if (error != 0)
			device_printf(dev,
			    "PCD init failed, continuing without RSS\n");
	}

	/* Create network interface for upper layers */
	ifp = sc->sc_ifnet = if_alloc(IFT_ETHER);
	if_setsoftc(ifp, sc);

	if_setflags(ifp, IFF_SIMPLEX | IFF_BROADCAST | IFF_MULTICAST);
	if_setinitfn(ifp, dtsec_if_init);
	if_setioctlfn(ifp, dtsec_if_ioctl);
	if_setgetcounterfn(ifp, dtsec_get_counter);

	if (sc->sc_mode == DTSEC_MODE_REGULAR) {
		/* Modern if_transmit: per-CPU TX FQs, no locks, no sendq */
		if_settransmitfn(ifp, dtsec_rm_if_transmit);
		if_setqflushfn(ifp, dtsec_if_qflush);
	} else {
		/* Legacy if_start for Independent Mode */
		if_setstartfn(ifp, dtsec_if_start);
		if_setsendqlen(ifp, IFQ_MAXLEN);
	}

	if_initname(ifp, "dtsec", device_get_unit(sc->sc_dev));

	/* TODO */
#if 0
	if_setsendqlen(ifp, TSEC_TX_NUM_DESC - 1);
	if_setsendqready(ifp);
#endif

	if_setcapabilities(ifp, IFCAP_JUMBO_MTU | IFCAP_VLAN_MTU
	    | IFCAP_TXCSUM | IFCAP_TXCSUM_IPV6
	    | IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6);
	if_setcapenable(ifp, if_getcapabilities(ifp));
	if_sethwassist(ifp, CSUM_IP | CSUM_IP_TCP | CSUM_IP_UDP |
	    CSUM_IP6_TCP | CSUM_IP6_UDP);

	/* Attach PHY(s) — skip for 10G ports with no external PHY
	 * (SFP+ uses in-band status, sc_phy_addr set to -1). */
	if (sc->sc_phy_addr >= 0) {
		error = mii_attach(sc->sc_dev, &sc->sc_mii_dev, ifp,
		    dtsec_ifmedia_upd, dtsec_ifmedia_sts, BMSR_DEFCAPMASK,
		    sc->sc_phy_addr, MII_OFFSET_ANY, 0);
		if (error) {
			if_printf(sc->sc_ifnet, "attaching PHYs failed: "
			    "%d\n", error);
			dtsec_detach(sc->sc_dev);
			return (error);
		}
		sc->sc_mii = device_get_softc(sc->sc_mii_dev);
	}

	/* Attach to stack */
	ether_ifattach(ifp, sc->sc_mac_addr);

	/* Set baudrate */
	if (sc->sc_eth_dev_type == ETH_10GSEC)
		if_setbaudrate(ifp, IF_Gbps(10ULL));
	else
		if_setbaudrate(ifp, IF_Gbps(1ULL));

	/* 10G ports: SFP leaves default LINK_STATE_UNKNOWN (no module),
	 * non-SFP assumes always up (legacy) */
	if (sc->sc_phy_addr < 0 && sc->sc_sfp_dev == NULL)
		if_link_state_change(ifp, LINK_STATE_UP);

	/* Add diagnostic sysctls */
	if (sc->sc_mode == DTSEC_MODE_REGULAR) {
		struct sysctl_ctx_list *ctx;
		struct sysctl_oid *tree;

		ctx = device_get_sysctl_ctx(dev);
		tree = device_get_sysctl_tree(dev);

		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "bman_free", CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    sc, 0, dtsec_sysctl_bman_free, "IU",
		    "BMan free buffer count");
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
		    "diag", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    sc, 0, dtsec_sysctl_diag, "I",
		    "Write 1 to dump DQRR + BMan state to dmesg");

		if (sc->sc_sfp_dev != NULL) {
			SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree),
			    OID_AUTO, "sfp_info",
			    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
			    sc, 0, dtsec_sysctl_sfp_info, "A",
			    "SFP+ module vendor and part number");
		}
	}

	return (0);
}

int
dtsec_detach(device_t dev)
{
	struct dtsec_softc *sc;
	if_t ifp;

	sc = device_get_softc(dev);
	ifp = sc->sc_ifnet;

	if (device_is_attached(dev)) {
		ether_ifdetach(ifp);
		/* Shutdown interface */
		DTSEC_LOCK(sc);
		dtsec_if_deinit_locked(sc);
		DTSEC_UNLOCK(sc);
	}

	if (sc->sc_ifnet) {
		if_free(sc->sc_ifnet);
		sc->sc_ifnet = NULL;
	}

	if (sc->sc_mode == DTSEC_MODE_REGULAR) {
		/* Free PCD (must happen before port free) */
		dtsec_rm_pcd_free(sc);

		/* Free RX/TX FQRs */
		dtsec_rm_fqr_rx_free(sc);
		dtsec_rm_fqr_tx_free(sc);

		/* Free frame info pool */
		dtsec_rm_fi_pool_free(sc);

		/* Free RX buffer pool */
		dtsec_rm_pool_rx_free(sc);
	}

	dtsec_fm_mac_free(sc);
	dtsec_fm_port_free_both(sc);

	/* Destroy lock */
	mtx_destroy(&sc->sc_lock);

	return (0);
}

int
dtsec_suspend(device_t dev)
{

	return (0);
}

int
dtsec_resume(device_t dev)
{

	return (0);
}

int
dtsec_shutdown(device_t dev)
{

	return (0);
}
/** @} */


/**
 * @group MII bus interface.
 * @{
 */
int
dtsec_miibus_readreg(device_t dev, int phy, int reg)
{
	struct dtsec_softc *sc;

	sc = device_get_softc(dev);

	return (MIIBUS_READREG(sc->sc_mdio, phy, reg));
}

int
dtsec_miibus_writereg(device_t dev, int phy, int reg, int value)
{

	struct dtsec_softc *sc;

	sc = device_get_softc(dev);

	return (MIIBUS_WRITEREG(sc->sc_mdio, phy, reg, value));
}

void
dtsec_miibus_statchg(device_t dev)
{
	struct dtsec_softc *sc;
	e_EnetSpeed speed;
	bool duplex;
	int error;

	sc = device_get_softc(dev);

	DTSEC_LOCK_ASSERT(sc);

	if (sc->sc_mii == NULL)
		return;		/* 10G port, no PHY — nothing to adjust */

	duplex = ((sc->sc_mii->mii_media_active & IFM_GMASK) == IFM_FDX);

	switch (IFM_SUBTYPE(sc->sc_mii->mii_media_active)) {
	case IFM_1000_T:
	case IFM_1000_SX:
		speed = e_ENET_SPEED_1000;
		break;

        case IFM_100_TX:
		speed = e_ENET_SPEED_100;
		break;

        case IFM_10_T:
		speed = e_ENET_SPEED_10;
		break;

	default:
		speed = e_ENET_SPEED_10;
	}

	error = FM_MAC_AdjustLink(sc->sc_mach, speed, duplex);
	if (error != E_OK)
		if_printf(sc->sc_ifnet, "error while adjusting MAC speed.\n");
}
/** @} */
