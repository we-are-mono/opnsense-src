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
 * MaxLinear GPY115/GPY211/GPY212/GPY215/GPY241/GPY245 PHY driver.
 *
 * These PHYs require explicit SGMII auto-negotiation enable via a
 * vendor-specific MMD register (VSPEC1_SGMII_CTRL).  Without this,
 * the MAC-side PCS never synchronizes and the link stays down.
 *
 * Reference: Linux drivers/net/phy/mxl-gpy.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/module.h>
#include <sys/bus.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include "miidevs.h"

#include "miibus_if.h"

/* Vendor-specific Clause 22 registers */
#define	GPY_MIISTAT		0x18	/* MII state */
#define	GPY_IMASK		0x19	/* Interrupt mask */
#define	GPY_ISTAT		0x1A	/* Interrupt status */
#define	GPY_FWV			0x1E	/* Firmware version */

/* GPY_MIISTAT bits */
#define	GPY_MIISTAT_LS		(1 << 10)	/* Link status */
#define	GPY_MIISTAT_DPX		(1 << 3)	/* Duplex: 1=full */
#define	GPY_MIISTAT_SPD_MASK	0x0007
#define	GPY_MIISTAT_SPD_10	0
#define	GPY_MIISTAT_SPD_100	1
#define	GPY_MIISTAT_SPD_1000	2
#define	GPY_MIISTAT_SPD_2500	4

/* GPY_FWV bits */
#define	GPY_FWV_REL_MASK	(1 << 15)
#define	GPY_FWV_MAJOR_MASK	0x0F00
#define	GPY_FWV_MAJOR_SHIFT	8
#define	GPY_FWV_MINOR_MASK	0x00FF

/* MMD device address for vendor-specific registers */
#define	MDIO_MMD_VEND1		30

/* VSPEC1 (MMD30) registers */
#define	VSPEC1_SGMII_CTRL	0x08
#define	VSPEC1_SGMII_CTRL_ANEN	(1 << 12)	/* SGMII AN enable */
#define	VSPEC1_SGMII_CTRL_ANRS	(1 << 9)	/* SGMII AN restart */

struct gpyphy_softc {
	struct mii_softc mii_sc;
	uint8_t		fw_major;
	uint8_t		fw_minor;
	int		prev_link;
};

static int	gpyphy_probe(device_t);
static int	gpyphy_attach(device_t);
static int	gpyphy_service(struct mii_softc *, struct mii_data *, int);
static void	gpyphy_status(struct mii_softc *);
static void	gpyphy_reset(struct mii_softc *);

static device_method_t gpyphy_methods[] = {
	DEVMETHOD(device_probe,		gpyphy_probe),
	DEVMETHOD(device_attach,	gpyphy_attach),
	DEVMETHOD(device_detach,	mii_phy_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),
	DEVMETHOD_END
};

static driver_t gpyphy_driver = {
	"gpyphy",
	gpyphy_methods,
	sizeof(struct gpyphy_softc)
};

DRIVER_MODULE(gpyphy, miibus, gpyphy_driver, 0, 0);

static const struct mii_phydesc gpyphys[] = {
	MII_PHY_DESC(MAXLINEAR, GPY2xx),
	MII_PHY_DESC(MAXLINEAR, GPY115),
	MII_PHY_DESC(MAXLINEAR, GPY115C),
	MII_PHY_DESC(MAXLINEAR, GPY211),
	MII_PHY_DESC(MAXLINEAR, GPY211C),
	MII_PHY_DESC(MAXLINEAR, GPY212C),
	MII_PHY_DESC(MAXLINEAR, GPY215C),
	MII_PHY_DESC(MAXLINEAR, GPY241),
	MII_PHY_DESC(MAXLINEAR, GPY241BM),
	MII_PHY_DESC(MAXLINEAR, GPY245),
	MII_PHY_END
};

static const struct mii_phy_funcs gpyphy_funcs = {
	gpyphy_service,
	gpyphy_status,
	gpyphy_reset
};

/*
 * MMD indirect access via Clause 22 registers 13/14 (MII_MMDACR/MII_MMDAADR).
 * Same pattern used by micphy.c (ksz9031_read/write) and rgephy.c.
 */
static int
gpyphy_mmd_read(struct mii_softc *sc, int devaddr, int reg)
{

	/* Set MMD device address */
	PHY_WRITE(sc, MII_MMDACR, devaddr & MMDACR_DADDRMASK);
	/* Set MMD register address */
	PHY_WRITE(sc, MII_MMDAADR, reg);
	/* Select data, no post-increment */
	PHY_WRITE(sc, MII_MMDACR,
	    MMDACR_FN_DATANPI | (devaddr & MMDACR_DADDRMASK));
	/* Read data */
	return (PHY_READ(sc, MII_MMDAADR));
}

static void
gpyphy_mmd_write(struct mii_softc *sc, int devaddr, int reg, int val)
{

	/* Set MMD device address */
	PHY_WRITE(sc, MII_MMDACR, devaddr & MMDACR_DADDRMASK);
	/* Set MMD register address */
	PHY_WRITE(sc, MII_MMDAADR, reg);
	/* Select data, no post-increment */
	PHY_WRITE(sc, MII_MMDACR,
	    MMDACR_FN_DATANPI | (devaddr & MMDACR_DADDRMASK));
	/* Write data */
	PHY_WRITE(sc, MII_MMDAADR, val);
}

/*
 * Enable and restart SGMII auto-negotiation.
 *
 * The GPY PHY uses SGMII to communicate with the MAC's PCS.
 * Without SGMII AN enabled, the PCS never synchronizes and the
 * MAC reports no carrier even when the copper link is up.
 */
static void
gpyphy_sgmii_aneg(struct mii_softc *sc)
{
	int val;

	val = gpyphy_mmd_read(sc, MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL);
	val |= VSPEC1_SGMII_CTRL_ANEN | VSPEC1_SGMII_CTRL_ANRS;
	gpyphy_mmd_write(sc, MDIO_MMD_VEND1, VSPEC1_SGMII_CTRL, val);
}

static int
gpyphy_probe(device_t dev)
{

	return (mii_phy_dev_probe(dev, gpyphys, BUS_PROBE_DEFAULT));
}

static int
gpyphy_attach(device_t dev)
{
	struct gpyphy_softc *gsc;
	struct mii_softc *sc;
	int fwv;

	gsc = device_get_softc(dev);
	sc = &gsc->mii_sc;
	gsc->prev_link = 0;

	/* Must initialize mii_softc before any PHY register access */
	mii_phy_dev_attach(dev, MIIF_NOMANPAUSE, &gpyphy_funcs, 1);

	/* Read firmware version */
	fwv = PHY_READ(sc, GPY_FWV);
	gsc->fw_major = (fwv & GPY_FWV_MAJOR_MASK) >> GPY_FWV_MAJOR_SHIFT;
	gsc->fw_minor = fwv & GPY_FWV_MINOR_MASK;

	device_printf(dev, "MaxLinear GPY Firmware: %d.%d (0x%04x%s)\n",
	    gsc->fw_major, gsc->fw_minor, fwv,
	    (fwv & GPY_FWV_REL_MASK) ? "" : " test");

	/* Enable SGMII auto-negotiation — critical for MAC-side link */
	gpyphy_sgmii_aneg(sc);

	mii_phy_setmedia(sc);

	return (0);
}

static int
gpyphy_service(struct mii_softc *sc, struct mii_data *mii, int cmd)
{
	struct gpyphy_softc *gsc = (struct gpyphy_softc *)sc;

	switch (cmd) {
	case MII_POLLSTAT:
		break;

	case MII_MEDIACHG:
		mii_phy_setmedia(sc);
		/* Re-trigger SGMII AN after media change */
		gpyphy_sgmii_aneg(sc);
		break;

	case MII_TICK:
		if (mii_phy_tick(sc) == EJUSTRETURN)
			return (0);
		break;
	}

	/* Update the media status */
	PHY_STATUS(sc);

	/*
	 * Re-trigger SGMII AN on link-up transitions.
	 *
	 * Some GPY firmware versions don't re-trigger SGMII AN when the
	 * copper link comes back at the same speed.  Always re-trigger
	 * on down→up transitions to be safe — SGMII AN restart is
	 * harmless if already negotiated.
	 */
	if ((mii->mii_media_status & IFM_ACTIVE) && !gsc->prev_link) {
		gpyphy_sgmii_aneg(sc);
	}
	gsc->prev_link = (mii->mii_media_status & IFM_ACTIVE) ? 1 : 0;

	/* Callback if something changed */
	mii_phy_update(sc, cmd);
	return (0);
}

static void
gpyphy_status(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	int bmsr, miistat;

	mii->mii_media_status = IFM_AVALID;
	mii->mii_media_active = IFM_ETHER;

	/* Read BMSR twice — link status bit latches low */
	bmsr = PHY_READ(sc, MII_BMSR) | PHY_READ(sc, MII_BMSR);

	/*
	 * Read vendor-specific MIISTAT for link, speed, and duplex.
	 * PHY_MIISTAT reflects the actual resolved link state and is
	 * more reliable than decoding ANLPAR for GPY PHYs.
	 */
	miistat = PHY_READ(sc, GPY_MIISTAT);

	if (!(miistat & GPY_MIISTAT_LS)) {
		/* No link — check BMSR too for consistency */
		if (bmsr & BMSR_LINK) {
			/*
			 * BMSR shows link but MIISTAT doesn't — this can
			 * happen briefly during SGMII negotiation.  Report
			 * no link; the next tick will re-check.
			 */
		}
		mii->mii_media_active |= IFM_NONE;
		return;
	}

	mii->mii_media_status |= IFM_ACTIVE;

	/* Duplex */
	if (miistat & GPY_MIISTAT_DPX)
		mii->mii_media_active |= IFM_FDX;
	else
		mii->mii_media_active |= IFM_HDX;

	/* Speed */
	switch (miistat & GPY_MIISTAT_SPD_MASK) {
	case GPY_MIISTAT_SPD_1000:
		mii->mii_media_active |= IFM_1000_T;
		break;
	case GPY_MIISTAT_SPD_100:
		mii->mii_media_active |= IFM_100_TX;
		break;
	case GPY_MIISTAT_SPD_10:
		mii->mii_media_active |= IFM_10_T;
		break;
	case GPY_MIISTAT_SPD_2500:
		mii->mii_media_active |= IFM_2500_T;
		break;
	default:
		mii->mii_media_active |= IFM_NONE;
		break;
	}

	if ((mii->mii_media_active & IFM_FDX) != 0)
		mii->mii_media_active |= mii_phy_flowstatus(sc);
}

static void
gpyphy_reset(struct mii_softc *sc)
{

	/*
	 * GPY is a firmware-based PHY.  BMCR software reset disrupts
	 * firmware operation (copper link never establishes, LEDs stay
	 * dark).  Linux's GPY driver does not define .soft_reset either.
	 *
	 * Skip mii_phy_reset() and just ensure SGMII AN is enabled.
	 * mii_phy_setmedia() calls PHY_RESET before mii_phy_auto(),
	 * so this path is hit on every media change.
	 */
	gpyphy_sgmii_aneg(sc);
}
