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
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>

#include <machine/bus.h>

#ifdef __powerpc__
#include <powerpc/mpc85xx/mpc85xx.h>
#endif

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include "miibus_if.h"
#include "sff_if.h"

#include <contrib/ncsw/inc/Peripherals/fm_port_ext.h>
#include <contrib/ncsw/inc/xx_ext.h>

#include "if_dtsec.h"
#include "fman.h"


static int	dtsec_fdt_probe(device_t dev);
static int	dtsec_fdt_attach(device_t dev);

static device_method_t dtsec_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		dtsec_fdt_probe),
	DEVMETHOD(device_attach,	dtsec_fdt_attach),
	DEVMETHOD(device_detach,	dtsec_detach),

	DEVMETHOD(device_shutdown,	dtsec_shutdown),
	DEVMETHOD(device_suspend,	dtsec_suspend),
	DEVMETHOD(device_resume,	dtsec_resume),

	/* Bus interface */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	dtsec_miibus_readreg),
	DEVMETHOD(miibus_writereg,	dtsec_miibus_writereg),
	DEVMETHOD(miibus_statchg,	dtsec_miibus_statchg),

	{ 0, 0 }
};

static driver_t dtsec_driver = {
	"dtsec",
	dtsec_methods,
	sizeof(struct dtsec_softc),
};

DRIVER_MODULE(dtsec, fman, dtsec_driver, 0, 0);
DRIVER_MODULE(miibus, dtsec, miibus_driver, 0, 0);
MODULE_DEPEND(dtsec, ether, 1, 1, 1);
MODULE_DEPEND(dtsec, miibus, 1, 1, 1);
MODULE_DEPEND(dtsec, sff, 1, 1, 1);

static int
dtsec_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,fman-dtsec") &&
	    !ofw_bus_is_compatible(dev, "fsl,fman-xgec") &&
	    !ofw_bus_is_compatible(dev, "fsl,fman-memac"))
		return (ENXIO);

	device_set_desc(dev, "Freescale Data Path Ethernet Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
dtsec_fdt_attach(device_t dev)
{
	struct dtsec_softc *sc;
	device_t phy_dev;
	phandle_t enet_node, phy_node;
	phandle_t fman_rxtx_node[2];
	char phy_type[16];
	pcell_t fman_tx_cell, mac_id;
	int rid;

	sc = device_get_softc(dev);
	enet_node = ofw_bus_get_node(dev);

	if (OF_getprop(enet_node, "local-mac-address",
	    (void *)sc->sc_mac_addr, 6) == -1) {
		device_printf(dev,
		    "Could not load local-mac-addr property from DTS\n");
		return (ENXIO);
	}

	/* Determine MAC type / link speed */
	if (ofw_bus_is_compatible(dev, "fsl,fman-memac") != 0) {
		/* mEMAC: detect 10G from phy-connection-type */
		if (OF_getprop(enet_node, "phy-connection-type",
		    (void *)phy_type, sizeof(phy_type)) > 0 &&
		    (!strcmp(phy_type, "xgmii") ||
		     !strcmp(phy_type, "10gbase-r")))
			sc->sc_eth_dev_type = ETH_10GSEC;
		else
			sc->sc_eth_dev_type = ETH_DTSEC;
	} else if (ofw_bus_is_compatible(dev, "fsl,fman-dtsec") != 0)
		sc->sc_eth_dev_type = ETH_DTSEC;
	else if (ofw_bus_is_compatible(dev, "fsl,fman-xgec") != 0)
		sc->sc_eth_dev_type = ETH_10GSEC;
	else {
		device_printf(dev, "unknown MAC compatible\n");
		return(ENXIO);
	}

	/* Get PHY address */
	if (OF_getencprop(enet_node, "phy-handle", (void *)&phy_node,
	    sizeof(phy_node)) <= 0) {
		/* 10G ports may use managed="in-band-status" with no PHY */
		if (sc->sc_eth_dev_type == ETH_10GSEC) {
			sc->sc_phy_addr = -1;
			sc->sc_mdio = NULL;
			goto skip_phy;
		}
		device_printf(dev, "missing phy-handle\n");
		return (ENXIO);
	}

	phy_node = OF_node_from_xref(phy_node);

	if (OF_getencprop(phy_node, "reg", (void *)&sc->sc_phy_addr,
	    sizeof(sc->sc_phy_addr)) <= 0) {
		device_printf(dev, "missing phy reg\n");
		return (ENXIO);
	}

	phy_dev = OF_device_from_xref(OF_xref_from_node(OF_parent(phy_node)));

	if (phy_dev == NULL) {
		device_printf(dev, "No PHY found.\n");
		return (ENXIO);
	}

	sc->sc_mdio = phy_dev;
skip_phy:

	/* Parse SFP phandle for 10G ports */
	if (sc->sc_eth_dev_type == ETH_10GSEC) {
		phandle_t sfp_xref, sfp_node;
		device_t sfp_dev;

		if (OF_getencprop(enet_node, "sfp", &sfp_xref,
		    sizeof(sfp_xref)) > 0) {
			sfp_node = OF_node_from_xref(sfp_xref);
			sfp_dev = OF_device_from_xref(sfp_xref);

			if (sfp_dev != NULL) {
				sc->sc_sfp_dev = sfp_dev;

				/* Get I2C bus from sff driver */
				if (SFF_GET_I2C_BUS(sfp_dev,
				    &sc->sc_sfp_i2c) != 0) {
					device_printf(dev,
					    "SFP: failed to get I2C bus\n");
					sc->sc_sfp_i2c = NULL;
				}

				/* Acquire GPIOs from SFP DT node */
				gpio_pin_get_by_ofw_property(dev, sfp_node,
				    "mod-def0-gpios", &sc->sc_sfp_moddef0);
				gpio_pin_get_by_ofw_property(dev, sfp_node,
				    "los-gpios", &sc->sc_sfp_los);
				gpio_pin_get_by_ofw_property(dev, sfp_node,
				    "tx-disable-gpios", &sc->sc_sfp_txdis);

				/* Hold TX disabled until module detected */
				if (sc->sc_sfp_txdis != NULL)
					gpio_pin_set_active(sc->sc_sfp_txdis,
					    true);

				if (sc->sc_sfp_moddef0 != NULL)
					device_printf(dev,
					    "SFP+ cage detected (GPIOs OK)\n");
				else
					device_printf(dev,
					    "SFP: no mod-def0 GPIO\n");
			} else {
				device_printf(dev,
				    "SFP: sff driver not found\n");
			}
		}
	}

	/* Get MAC memory offset in SoC */
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "could not alloc memory resource\n");
		return (ENXIO);
	}

	/* Get PHY connection type */
	if (OF_getprop(enet_node, "phy-connection-type", (void *)phy_type,
	    sizeof(phy_type)) <= 0) {
		device_printf(dev, "missing phy-connection-type\n");
		return (ENXIO);
	}

	if (!strcmp(phy_type, "sgmii"))
		sc->sc_mac_enet_mode = e_ENET_MODE_SGMII_1000;
	else if (!strcmp(phy_type, "rgmii"))
		sc->sc_mac_enet_mode = e_ENET_MODE_RGMII_1000;
	else if (!strcmp(phy_type, "xgmii"))
		sc->sc_mac_enet_mode = e_ENET_MODE_XGMII_10000;
	else if (!strcmp(phy_type, "10gbase-r"))
		sc->sc_mac_enet_mode = e_ENET_MODE_XGMII_10000;
	else {
		device_printf(dev, "unsupported phy-connection-type: %s\n",
		    phy_type);
		return (ENXIO);
	}

	if (OF_getencprop(enet_node, "cell-index",
	    (void *)&mac_id, sizeof(mac_id)) <= 0) {
		device_printf(dev, "missing cell-index\n");
		return (ENXIO);
	}
	sc->sc_eth_id = mac_id;
	sc->sc_mac_cell_index = mac_id;
	/* ncsw expects 0-based per-type MAC IDs: 0..5 for 1G, 0..1 for 10G.
	 * FManV3 device trees use cell-index 0-5 for 1G MACs but 8-9 for
	 * 10G MACs (hardware MEMAC slot numbering).  Remap to 0-based. */
	if (sc->sc_eth_dev_type == ETH_10GSEC && mac_id >= 8)
		sc->sc_eth_id = mac_id - 8;

	/* Get RX/TX port handles */
	if (OF_getencprop(enet_node, "fsl,fman-ports", (void *)fman_rxtx_node,
	    sizeof(fman_rxtx_node)) <= 0) {
		device_printf(dev, "missing fsl,fman-ports\n");
		return (ENXIO);
	}

	if (fman_rxtx_node[0] == 0) {
		device_printf(dev, "fsl,fman-ports[0] is zero\n");
		return (ENXIO);
	}

	if (fman_rxtx_node[1] == 0) {
		device_printf(dev, "fsl,fman-ports[1] is zero\n");
		return (ENXIO);
	}

	/* fsl,fman-ports are phandle references on ARM64, ihandles on PPC */
	fman_rxtx_node[0] = OF_node_from_xref(fman_rxtx_node[0]);
	fman_rxtx_node[1] = OF_node_from_xref(fman_rxtx_node[1]);

	if (ofw_bus_node_is_compatible(fman_rxtx_node[0],
	    "fsl,fman-v2-port-rx") == 0 &&
	    ofw_bus_node_is_compatible(fman_rxtx_node[0],
	    "fsl,fman-v3-port-rx") == 0) {
		device_printf(dev, "RX port incompatible\n");
		return (ENXIO);
	}

	if (ofw_bus_node_is_compatible(fman_rxtx_node[1],
	    "fsl,fman-v2-port-tx") == 0 &&
	    ofw_bus_node_is_compatible(fman_rxtx_node[1],
	    "fsl,fman-v3-port-tx") == 0) {
		device_printf(dev, "TX port incompatible\n");
		return (ENXIO);
	}

	/* Get RX port HW id */
	if (OF_getencprop(fman_rxtx_node[0], "reg", (void *)&sc->sc_port_rx_hw_id,
	    sizeof(sc->sc_port_rx_hw_id)) <= 0) {
		device_printf(dev, "missing RX port reg\n");
		return (ENXIO);
	}

	/* Get TX port HW id */
	if (OF_getencprop(fman_rxtx_node[1], "reg", (void *)&sc->sc_port_tx_hw_id,
	    sizeof(sc->sc_port_tx_hw_id)) <= 0) {
		device_printf(dev, "missing TX port reg\n");
		return (ENXIO);
	}

	if (OF_getencprop(fman_rxtx_node[1], "cell-index", &fman_tx_cell,
	    sizeof(fman_tx_cell)) <= 0) {
		device_printf(dev, "missing TX port cell-index\n");
		return (ENXIO);
	}
	/* Get QMan channel */
	sc->sc_port_tx_qman_chan = fman_qman_channel_id(device_get_parent(dev),
	    fman_tx_cell);

	return (dtsec_attach(dev));
}
