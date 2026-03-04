/*-
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
 * mEMAC external MDIO controller driver for FManv3 (fsl,fman-memac-mdio).
 *
 * On LS1046A, external PHYs are accessed through dedicated MDIO controllers
 * (mdio@fc000, mdio@fd000) rather than through each MAC's internal MDIO.
 * Register layout matches NXP's "xgmac_mdio" Linux driver.
 *
 * Modeled after fman_mdio.c (pqmdio) which handles the dTSEC MDIO layout.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "miibus_if.h"

/*
 * MDIO registers at offset 0x30 from the MDIO controller base.
 * See struct memac_mii_access_mem_map in fsl_fman_memac_mii_acc.h.
 */
#define	MEMAC_MDIO_CFG		0x030
#define	MEMAC_MDIO_CTRL		0x034
#define	MEMAC_MDIO_DATA		0x038
#define	MEMAC_MDIO_ADDR		0x03c

/* MDIO_CFG bits */
#define	MDIO_CFG_BSY		0x80000000
#define	MDIO_CFG_READ_ERR	0x00000002
#define	MDIO_CFG_ENC45		0x00000040
#define	MDIO_CFG_CLK_DIV_MASK	0x0080ff80
#define	MDIO_CFG_HOLD_MASK	0x0000001c

/* MDIO_CTRL bits */
#define	MDIO_CTL_READ		0x00008000
#define	MDIO_CTL_PHY_ADDR_SHIFT	5

/* MDIO_DATA bits */
#define	MDIO_DATA_BSY		0x80000000

#define	MDIO_TIMEOUT		1000	/* iterations */

struct memac_mdio_softc {
	device_t		sc_dev;
	struct mtx		sc_lock;
	struct resource		*sc_mem;
};

/*
 * FMan registers are big-endian.  bus_read_4/write_4 on ARM64 give
 * native (little-endian) access, so we byte-swap.
 */
static inline uint32_t
memac_mdio_read(struct memac_mdio_softc *sc, bus_size_t off)
{

	return (be32toh(bus_read_4(sc->sc_mem, off)));
}

static inline void
memac_mdio_write(struct memac_mdio_softc *sc, bus_size_t off, uint32_t val)
{

	bus_write_4(sc->sc_mem, off, htobe32(val));
}

#define	MDIO_LOCK()	mtx_lock(&sc->sc_lock)
#define	MDIO_UNLOCK()	mtx_unlock(&sc->sc_lock)

static int	memac_mdio_probe(device_t dev);
static int	memac_mdio_attach(device_t dev);
static int	memac_mdio_detach(device_t dev);
static int	memac_mdio_readreg(device_t dev, int phy, int reg);
static int	memac_mdio_writereg(device_t dev, int phy, int reg, int value);

static device_method_t memac_mdio_methods[] = {
	DEVMETHOD(device_probe,		memac_mdio_probe),
	DEVMETHOD(device_attach,	memac_mdio_attach),
	DEVMETHOD(device_detach,	memac_mdio_detach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	memac_mdio_readreg),
	DEVMETHOD(miibus_writereg,	memac_mdio_writereg),

	DEVMETHOD_END
};

static struct ofw_compat_data memac_mdio_compat[] = {
	{"fsl,fman-memac-mdio",	1},
	{NULL,			0}
};

static driver_t memac_mdio_driver = {
	"memac_mdio",
	memac_mdio_methods,
	sizeof(struct memac_mdio_softc),
};

static int
memac_mdio_wait_free(struct memac_mdio_softc *sc)
{
	int timeout;

	for (timeout = MDIO_TIMEOUT; timeout > 0; timeout--) {
		if (!(memac_mdio_read(sc, MEMAC_MDIO_CFG) & MDIO_CFG_BSY))
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

static int
memac_mdio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, memac_mdio_compat)->ocd_str)
		return (ENXIO);

	device_set_desc(dev, "Freescale mEMAC MDIO Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
memac_mdio_attach(device_t dev)
{
	struct memac_mdio_softc *sc;
	int rid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);

	mtx_init(&sc->sc_lock, device_get_nameunit(dev),
	    "mEMAC MDIO lock", MTX_DEF);

	return (0);
}

static int
memac_mdio_detach(device_t dev)
{
	struct memac_mdio_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem);

	mtx_destroy(&sc->sc_lock);

	return (0);
}

/*
 * Clause 22 (1G) MDIO read.
 * Register protocol matches fman_memac_mii_acc.c:read_phy_reg_1g().
 */
static int
memac_mdio_readreg(device_t dev, int phy, int reg)
{
	struct memac_mdio_softc *sc;
	uint32_t cfg, ctl, data;

	sc = device_get_softc(dev);

	MDIO_LOCK();

	/* Clause 22: clear ENC45, preserve clock divider and hold */
	cfg = memac_mdio_read(sc, MEMAC_MDIO_CFG);
	cfg &= (MDIO_CFG_CLK_DIV_MASK | MDIO_CFG_HOLD_MASK);
	memac_mdio_write(sc, MEMAC_MDIO_CFG, cfg);

	if (memac_mdio_wait_free(sc) != 0) {
		MDIO_UNLOCK();
		return (0xffff);
	}

	/* Initiate read: phy in bits [9:5], register in bits [4:0] */
	ctl = ((phy & 0x1f) << MDIO_CTL_PHY_ADDR_SHIFT) | (reg & 0x1f);
	ctl |= MDIO_CTL_READ;
	memac_mdio_write(sc, MEMAC_MDIO_CTRL, ctl);

	if (memac_mdio_wait_free(sc) != 0) {
		MDIO_UNLOCK();
		return (0xffff);
	}

	/* Wait for data valid */
	{
		int timeout;
		for (timeout = MDIO_TIMEOUT; timeout > 0; timeout--) {
			data = memac_mdio_read(sc, MEMAC_MDIO_DATA);
			if (!(data & MDIO_DATA_BSY))
				break;
			DELAY(1);
		}
		if (timeout <= 0) {
			MDIO_UNLOCK();
			return (0xffff);
		}
	}

	/* Check for read error */
	if (memac_mdio_read(sc, MEMAC_MDIO_CFG) & MDIO_CFG_READ_ERR) {
		MDIO_UNLOCK();
		return (0xffff);
	}

	MDIO_UNLOCK();

	return (data & 0xffff);
}

/*
 * Clause 22 (1G) MDIO write.
 * Register protocol matches fman_memac_mii_acc.c:write_phy_reg_1g().
 */
static int
memac_mdio_writereg(device_t dev, int phy, int reg, int value)
{
	struct memac_mdio_softc *sc;
	uint32_t cfg, ctl;

	sc = device_get_softc(dev);

	MDIO_LOCK();

	/* Clause 22: clear ENC45, preserve clock divider and hold */
	cfg = memac_mdio_read(sc, MEMAC_MDIO_CFG);
	cfg &= (MDIO_CFG_CLK_DIV_MASK | MDIO_CFG_HOLD_MASK);
	memac_mdio_write(sc, MEMAC_MDIO_CFG, cfg);

	if (memac_mdio_wait_free(sc) != 0) {
		MDIO_UNLOCK();
		return (EIO);
	}

	/* Set phy address and register */
	ctl = ((phy & 0x1f) << MDIO_CTL_PHY_ADDR_SHIFT) | (reg & 0x1f);
	memac_mdio_write(sc, MEMAC_MDIO_CTRL, ctl);

	if (memac_mdio_wait_free(sc) != 0) {
		MDIO_UNLOCK();
		return (EIO);
	}

	/* Write data */
	memac_mdio_write(sc, MEMAC_MDIO_DATA, (uint32_t)(value & 0xffff));

	/* Wait for write to complete */
	{
		int timeout;
		for (timeout = MDIO_TIMEOUT; timeout > 0; timeout--) {
			if (!(memac_mdio_read(sc, MEMAC_MDIO_DATA) &
			    MDIO_DATA_BSY))
				break;
			DELAY(1);
		}
	}

	MDIO_UNLOCK();

	return (0);
}

/* Probe before dtsec (BUS_PASS_SUPPORTDEV) so sc_mdio lookup succeeds */
EARLY_DRIVER_MODULE(memac_mdio, fman, memac_mdio_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
DRIVER_MODULE(miibus, memac_mdio, miibus_driver, 0, 0);
MODULE_DEPEND(memac_mdio, miibus, 1, 1, 1);
