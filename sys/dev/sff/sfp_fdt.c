/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Dmitry Salychev
 * Copyright (c) 2026 Mono Technologies Inc.
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
 * SFP/SFP+ transceiver management driver (FDT-based).
 *
 * Implements a state machine for SFP module hot-plug, EEPROM identification,
 * embedded PHY management (RollBall and standard I2C-MDIO), and link
 * monitoring.  MAC drivers register upstream callbacks to receive module
 * insert/remove and link up/down events.
 *
 * Inspired by Linux drivers/net/phy/sfp.c but simplified to a single
 * state machine without phylink dependency.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/taskqueue.h>
#include <sys/callout.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/simplebus.h>
#include <sys/gpio.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <dev/sff/sfp_fdt.h>

#include "sff_if.h"

/* State machine states */
#define	SFP_ST_EMPTY		0	/* No module */
#define	SFP_ST_PROBE		1	/* Module detected, reading EEPROM */
#define	SFP_ST_PRESENT		2	/* EEPROM read, classifying */
#define	SFP_ST_PHY_PROBE	3	/* Probing embedded PHY (copper) */
#define	SFP_ST_PHY_AN		4	/* PHY found, polling link */
#define	SFP_ST_DDM_POLL		5	/* No PHY, polling DDM RX_LOS */
#define	SFP_ST_WAIT_LOS		6	/* Fiber/DAC: monitoring LOS GPIO */
#define	SFP_ST_LINK_UP		7	/* Link established */

/* IEEE 802.3 Clause 45 MMD device numbers */
#define	MDIO_MMD_PMAPMD		1
#define	MDIO_MMD_AN		7

/* Standard C45 register offsets */
#define	MDIO_CTRL1		0
#define	MDIO_STAT1		1
#define	MDIO_DEVID1		2
#define	MDIO_DEVID2		3
#define	MDIO_PMA_EXTABLE	11
#define	MDIO_AN_10GBT_CTRL	32
#define	MDIO_AN_10GBT_STAT	33

/* STAT1 bits */
#define	MDIO_STAT1_LSTATUS		(1 << 2)

/* AN CTRL1 bits */
#define	MDIO_AN_CTRL1_XNP		(1 << 13)
#define	MDIO_AN_CTRL1_ENABLE		(1 << 12)
#define	MDIO_AN_CTRL1_RESTART		(1 << 9)

/* AN STAT1 bits */
#define	MDIO_AN_STAT1_COMPLETE		(1 << 5)

/* PMA EXTABLE bits */
#define	MDIO_PMA_EXTABLE_10GBT		(1 << 2)
#define	MDIO_PMA_EXTABLE_1000BT	(1 << 5)
#define	MDIO_PMA_EXTABLE_NBT		(1 << 14)

/* AN 10GBT_CTRL advertisement bits */
#define	MDIO_AN_10GBT_CTRL_ADV10G	(1 << 12)
#define	MDIO_AN_10GBT_CTRL_ADV5G	(1 << 8)
#define	MDIO_AN_10GBT_CTRL_ADV2_5G	(1 << 7)

/* AN 10GBT_STAT link partner bits */
#define	MDIO_AN_10GBT_STAT_LP10G	(1 << 11)
#define	MDIO_AN_10GBT_STAT_LP5G		(1 << 6)
#define	MDIO_AN_10GBT_STAT_LP2_5G	(1 << 5)

/* Standard I2C-MDIO bridge (SFF-8472) */
#define	MDIOI2C_I2C_ADDR	0x56

/* RollBall I2C-to-MDIO bridge */
#define	ROLLBALL_I2C_ADDR	0x51
#define	ROLLBALL_PAGE_REG	0x7F
#define	ROLLBALL_PAGE		3
#define	ROLLBALL_PASSWORD_REG	0x7B
#define	ROLLBALL_CMD_ADDR	0x80
#define	ROLLBALL_DATA_ADDR	0x81
#define	ROLLBALL_CMD_READ	0x02
#define	ROLLBALL_CMD_WRITE	0x01
#define	ROLLBALL_CMD_DONE	0x04
#define	ROLLBALL_POLL_RETRIES	20
#define	ROLLBALL_POLL_MS	20

/* PHY access protocol */
#define	SFP_PHY_ROLLBALL	1
#define	SFP_PHY_MDIOI2C	2

/* PHY boot retries (500ms each, up to 4.5s total) */
#define	SFP_PHY_BOOT_RETRIES	9
#define	SFP_PHY_BOOT_MS		500

struct sfp_fdt_softc {
	device_t	sc_dev;
	phandle_t	sc_node;

	/* I2C */
	phandle_t	sc_i2c_phandle;
	device_t	sc_i2c;		/* resolved iicbus device */

	/* GPIOs */
	gpio_pin_t	sc_moddef0;	/* module detect (active low) */
	gpio_pin_t	sc_los;		/* loss of signal */
	gpio_pin_t	sc_txdis;	/* TX disable (output) */

	/* State machine */
	struct mtx	sc_mtx;
	int		sc_state;
	uint8_t		sc_id[64];	/* EEPROM A0h base ID cache */
	uint8_t		sc_connector;

	/* PHY (copper modules) */
	bool		sc_has_phy;
	int		sc_phy_proto;
	uint32_t	sc_phy_id;
	bool		sc_phy_link;
	int		sc_phy_speed;
	int		sc_phy_retries;

	/* Fiber/DAC LOS tracking */
	bool		sc_los_prev;

	/* Upstream (MAC driver) */
	const struct sfp_upstream_ops *sc_upstream_ops;
	void		*sc_upstream_arg;

	/* Polling */
	struct task	sc_sm_task;
	struct callout	sc_poll;
};

#define	SFP_LOCK(sc)		mtx_lock(&(sc)->sc_mtx)
#define	SFP_UNLOCK(sc)		mtx_unlock(&(sc)->sc_mtx)

/* Forward declarations */
static void sfp_fdt_sm_task(void *arg, int pending);
static void sfp_fdt_poll_callout(void *arg);

/*
 * Lazily resolve I2C bus device.  The I2C mux (pca954x) may attach
 * after sfp_fdt, so we defer resolution until first use.
 */
static bool
sfp_fdt_resolve_i2c(struct sfp_fdt_softc *sc)
{
	device_t xdev;

	if (sc->sc_i2c != NULL)
		return (true);

	xdev = OF_device_from_xref(OF_xref_from_node(sc->sc_i2c_phandle));
	if (xdev == NULL)
		return (false);

	sc->sc_i2c = xdev;
	return (true);
}

/*
 * ============================================================
 * I2C helpers
 * ============================================================
 */

static int
sfp_fdt_i2c_read(struct sfp_fdt_softc *sc, uint8_t addr, uint8_t offset,
    uint8_t *buf, int len)
{
	struct iic_msg msgs[2];
	int error;

	msgs[0].slave = addr << 1;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = &offset;
	msgs[1].slave = addr << 1;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev, IIC_INTRWAIT);
	if (error != 0)
		return (error);
	error = iicbus_transfer(sc->sc_i2c, msgs, 2);
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

	return (error);
}

static int
sfp_fdt_i2c_write_raw(struct sfp_fdt_softc *sc, uint8_t addr,
    uint8_t *buf, int len)
{
	struct iic_msg msg;

	msg.slave = addr << 1;
	msg.flags = IIC_M_WR;
	msg.len = len;
	msg.buf = buf;

	return (iicbus_transfer(sc->sc_i2c, &msg, 1));
}

static int
sfp_fdt_i2c_read_raw(struct sfp_fdt_softc *sc, uint8_t addr,
    uint8_t reg, uint8_t *buf, int len)
{
	struct iic_msg msgs[2];

	msgs[0].slave = addr << 1;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	msgs[1].slave = addr << 1;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	return (iicbus_transfer(sc->sc_i2c, msgs, 2));
}

/*
 * ============================================================
 * EEPROM
 * ============================================================
 */

static int
sfp_fdt_read_eeprom(struct sfp_fdt_softc *sc)
{
	int error;

	error = sfp_fdt_i2c_read(sc, SFP_EEPROM_ADDR, 0,
	    sc->sc_id, sizeof(sc->sc_id));
	if (error != 0) {
		device_printf(sc->sc_dev, "SFP EEPROM read failed: %d\n",
		    error);
		memset(sc->sc_id, 0, sizeof(sc->sc_id));
	}
	return (error);
}

static void
sfp_fdt_trim(char *dst, const uint8_t *src, int len)
{
	int i;

	memcpy(dst, src, len);
	dst[len] = '\0';
	for (i = len - 1; i >= 0 && dst[i] == ' '; i--)
		dst[i] = '\0';
}

static void
sfp_fdt_log_module(struct sfp_fdt_softc *sc)
{
	char vendor[SFP_VENDOR_LEN + 1];
	char partnum[SFP_PARTNUM_LEN + 1];
	uint8_t connector;

	sfp_fdt_trim(vendor, &sc->sc_id[SFP_VENDOR_OFFSET], SFP_VENDOR_LEN);
	sfp_fdt_trim(partnum, &sc->sc_id[SFP_PARTNUM_OFFSET], SFP_PARTNUM_LEN);
	connector = sc->sc_id[SFP_CONNECTOR_OFFSET];

	device_printf(sc->sc_dev,
	    "SFP+ module: %s %s (connector 0x%02x%s)\n",
	    vendor, partnum, connector,
	    connector == SFP_CONNECTOR_RJ45 ? " RJ45" : "");
}

/*
 * ============================================================
 * Standard I2C-MDIO (SFF-8472, address 0x56)
 * ============================================================
 */

static int
mdioi2c_read(struct sfp_fdt_softc *sc, int devad, int reg)
{
	uint8_t addr[3], data[2];
	int error;

	addr[0] = 0x20 | (devad & 0x1F);
	addr[1] = (reg >> 8) & 0xFF;
	addr[2] = reg & 0xFF;

	error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev, IIC_INTRWAIT);
	if (error != 0)
		return (-1);

	error = sfp_fdt_i2c_write_raw(sc, MDIOI2C_I2C_ADDR, addr, 3);
	if (error == 0) {
		struct iic_msg msg;
		msg.slave = MDIOI2C_I2C_ADDR << 1;
		msg.flags = IIC_M_RD;
		msg.len = 2;
		msg.buf = data;
		error = iicbus_transfer(sc->sc_i2c, &msg, 1);
	}

	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

	if (error != 0)
		return (-1);
	return ((data[0] << 8) | data[1]);
}

static int
mdioi2c_write(struct sfp_fdt_softc *sc, int devad, int reg, int val)
{
	uint8_t buf[5];
	int error;

	buf[0] = devad & 0x1F;
	buf[1] = (reg >> 8) & 0xFF;
	buf[2] = reg & 0xFF;
	buf[3] = (val >> 8) & 0xFF;
	buf[4] = val & 0xFF;

	error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev, IIC_INTRWAIT);
	if (error != 0)
		return (error);
	error = sfp_fdt_i2c_write_raw(sc, MDIOI2C_I2C_ADDR, buf, 5);
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

	return (error);
}

/*
 * ============================================================
 * RollBall I2C-to-MDIO bridge (address 0x51)
 * ============================================================
 */

/* Bus must already be acquired */
static int
rollball_page_set(struct sfp_fdt_softc *sc, uint8_t *saved_page)
{
	uint8_t buf[2];
	int error;

	error = sfp_fdt_i2c_read_raw(sc, ROLLBALL_I2C_ADDR,
	    ROLLBALL_PAGE_REG, saved_page, 1);
	if (error != 0)
		return (error);

	buf[0] = ROLLBALL_PAGE_REG;
	buf[1] = ROLLBALL_PAGE;
	return (sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR, buf, 2));
}

/* Bus must already be acquired */
static int
rollball_page_restore(struct sfp_fdt_softc *sc, uint8_t saved_page)
{
	uint8_t buf[2] = { ROLLBALL_PAGE_REG, saved_page };

	return (sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR, buf, 2));
}

static int
rollball_init(struct sfp_fdt_softc *sc)
{
	uint8_t pw[] = { ROLLBALL_PASSWORD_REG, 0xFF, 0xFF, 0xFF, 0xFF };
	int error;

	error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev, IIC_INTRWAIT);
	if (error != 0)
		return (error);
	error = sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR, pw, sizeof(pw));
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

	return (error);
}

static int
rollball_phy_read(struct sfp_fdt_softc *sc, int devad, int reg)
{
	uint8_t data_buf[4], cmd_buf[2], res[6];
	uint8_t saved_page;
	int error, i;

	error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev, IIC_INTRWAIT);
	if (error != 0)
		return (-1);

	error = rollball_page_set(sc, &saved_page);
	if (error != 0)
		goto out;

	/* Write devad + register address */
	data_buf[0] = ROLLBALL_DATA_ADDR;
	data_buf[1] = devad;
	data_buf[2] = (reg >> 8) & 0xFF;
	data_buf[3] = reg & 0xFF;
	error = sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR,
	    data_buf, 4);
	if (error != 0)
		goto restore;

	/* Write read command */
	cmd_buf[0] = ROLLBALL_CMD_ADDR;
	cmd_buf[1] = ROLLBALL_CMD_READ;
	error = sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR,
	    cmd_buf, 2);
	if (error != 0)
		goto restore;

	/* Restore page before polling */
	rollball_page_restore(sc, saved_page);
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

	/* Poll for completion */
	for (i = 0; i < ROLLBALL_POLL_RETRIES; i++) {
		pause_sbt("rbpoll", SBT_1MS * ROLLBALL_POLL_MS, 0, C_PREL(2));

		error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev,
		    IIC_INTRWAIT);
		if (error != 0)
			return (-1);

		error = rollball_page_set(sc, &saved_page);
		if (error != 0) {
			iicbus_release_bus(sc->sc_i2c, sc->sc_dev);
			return (-1);
		}

		error = sfp_fdt_i2c_read_raw(sc, ROLLBALL_I2C_ADDR,
		    ROLLBALL_CMD_ADDR, res, 6);

		rollball_page_restore(sc, saved_page);
		iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

		if (error != 0)
			return (-1);

		if (res[0] == ROLLBALL_CMD_DONE)
			return ((res[4] << 8) | res[5]);
	}

	return (-1);	/* timeout */

restore:
	rollball_page_restore(sc, saved_page);
out:
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);
	return (-1);
}

static int
rollball_phy_write(struct sfp_fdt_softc *sc, int devad, int reg, int val)
{
	uint8_t data_buf[6], cmd_buf[2], status;
	uint8_t saved_page;
	int error, i;

	error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev, IIC_INTRWAIT);
	if (error != 0)
		return (error);

	error = rollball_page_set(sc, &saved_page);
	if (error != 0)
		goto out;

	data_buf[0] = ROLLBALL_DATA_ADDR;
	data_buf[1] = devad;
	data_buf[2] = (reg >> 8) & 0xFF;
	data_buf[3] = reg & 0xFF;
	data_buf[4] = (val >> 8) & 0xFF;
	data_buf[5] = val & 0xFF;
	error = sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR,
	    data_buf, 6);
	if (error != 0)
		goto restore;

	cmd_buf[0] = ROLLBALL_CMD_ADDR;
	cmd_buf[1] = ROLLBALL_CMD_WRITE;
	error = sfp_fdt_i2c_write_raw(sc, ROLLBALL_I2C_ADDR,
	    cmd_buf, 2);
	if (error != 0)
		goto restore;

	rollball_page_restore(sc, saved_page);
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

	/* Poll for completion */
	for (i = 0; i < ROLLBALL_POLL_RETRIES; i++) {
		pause_sbt("rbpoll", SBT_1MS * ROLLBALL_POLL_MS, 0, C_PREL(2));

		error = iicbus_request_bus(sc->sc_i2c, sc->sc_dev,
		    IIC_INTRWAIT);
		if (error != 0)
			return (error);

		error = rollball_page_set(sc, &saved_page);
		if (error != 0) {
			iicbus_release_bus(sc->sc_i2c, sc->sc_dev);
			return (error);
		}

		error = sfp_fdt_i2c_read_raw(sc, ROLLBALL_I2C_ADDR,
		    ROLLBALL_CMD_ADDR, &status, 1);

		rollball_page_restore(sc, saved_page);
		iicbus_release_bus(sc->sc_i2c, sc->sc_dev);

		if (error != 0)
			return (error);

		if (status == ROLLBALL_CMD_DONE)
			return (0);
	}

	return (ETIMEDOUT);

restore:
	rollball_page_restore(sc, saved_page);
out:
	iicbus_release_bus(sc->sc_i2c, sc->sc_dev);
	return (error);
}

/*
 * ============================================================
 * PHY register access — protocol dispatch
 * ============================================================
 */

static int
sfp_phy_read(struct sfp_fdt_softc *sc, int devad, int reg)
{

	if (sc->sc_phy_proto == SFP_PHY_MDIOI2C)
		return (mdioi2c_read(sc, devad, reg));
	return (rollball_phy_read(sc, devad, reg));
}

static int
sfp_phy_write(struct sfp_fdt_softc *sc, int devad, int reg, int val)
{

	if (sc->sc_phy_proto == SFP_PHY_MDIOI2C)
		return (mdioi2c_write(sc, devad, reg, val));
	return (rollball_phy_write(sc, devad, reg, val));
}

/*
 * ============================================================
 * PHY auto-negotiation and speed detection
 * ============================================================
 */

static void
sfp_phy_config_aneg(struct sfp_fdt_softc *sc)
{
	int adv10g;

	/* Advertise 10G/5G/2.5G in register 7.32 */
	adv10g = MDIO_AN_10GBT_CTRL_ADV10G |
	    MDIO_AN_10GBT_CTRL_ADV5G | MDIO_AN_10GBT_CTRL_ADV2_5G;
	sfp_phy_write(sc, MDIO_MMD_AN, MDIO_AN_10GBT_CTRL, adv10g);

	/* Enable and restart AN with XNP (Extended Next Page).
	 * 10GBASE-T capability is exchanged via extended next pages —
	 * without XNP, only base-page speeds (1G and below) negotiate. */
	sfp_phy_write(sc, MDIO_MMD_AN, MDIO_CTRL1,
	    MDIO_AN_CTRL1_XNP | MDIO_AN_CTRL1_ENABLE |
	    MDIO_AN_CTRL1_RESTART);

	device_printf(sc->sc_dev, "SFP+ PHY: auto-negotiation started\n");
}

static int
sfp_phy_read_speed(struct sfp_fdt_softc *sc)
{
	int stat, lpa10g, ctrl1;

	stat = sfp_phy_read(sc, MDIO_MMD_AN, MDIO_STAT1);
	if (stat < 0 || !(stat & MDIO_AN_STAT1_COMPLETE))
		return (0);

	lpa10g = sfp_phy_read(sc, MDIO_MMD_AN, MDIO_AN_10GBT_STAT);
	if (lpa10g >= 0) {
		if (lpa10g & MDIO_AN_10GBT_STAT_LP10G)
			return (10000);
		if (lpa10g & MDIO_AN_10GBT_STAT_LP5G)
			return (5000);
		if (lpa10g & MDIO_AN_10GBT_STAT_LP2_5G)
			return (2500);
	}

	/* PMA/PMD CTRL1 speed select: bit 13=10G, bit 6=1G */
	ctrl1 = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_CTRL1);
	if (ctrl1 >= 0) {
		if (ctrl1 & (1 << 13))
			return (10000);
		if (ctrl1 & (1 << 6))
			return (1000);
		return (100);
	}

	return (0);
}

/*
 * ============================================================
 * Upstream callback helpers
 * ============================================================
 */

static void
sfp_fdt_notify_insert(struct sfp_fdt_softc *sc)
{

	if (sc->sc_upstream_ops != NULL &&
	    sc->sc_upstream_ops->module_insert != NULL)
		sc->sc_upstream_ops->module_insert(sc->sc_upstream_arg,
		    sc->sc_id, sizeof(sc->sc_id));
}

static void
sfp_fdt_notify_remove(struct sfp_fdt_softc *sc)
{

	if (sc->sc_upstream_ops != NULL &&
	    sc->sc_upstream_ops->module_remove != NULL)
		sc->sc_upstream_ops->module_remove(sc->sc_upstream_arg);
}

static void
sfp_fdt_notify_link_up(struct sfp_fdt_softc *sc, int speed)
{

	if (sc->sc_upstream_ops != NULL &&
	    sc->sc_upstream_ops->link_up != NULL)
		sc->sc_upstream_ops->link_up(sc->sc_upstream_arg, speed);
}

static void
sfp_fdt_notify_link_down(struct sfp_fdt_softc *sc)
{

	if (sc->sc_upstream_ops != NULL &&
	    sc->sc_upstream_ops->link_down != NULL)
		sc->sc_upstream_ops->link_down(sc->sc_upstream_arg);
}

/*
 * ============================================================
 * State machine transitions
 * ============================================================
 */

/* EMPTY → PROBE: module just detected */
static void
sfp_fdt_sm_insert(struct sfp_fdt_softc *sc)
{

	device_printf(sc->sc_dev, "SFP+ module detected, probing...\n");

	/* De-assert TX disable so module MCU can fully boot */
	if (sc->sc_txdis != NULL)
		gpio_pin_set_active(sc->sc_txdis, false);

	sc->sc_state = SFP_ST_PROBE;
}

/* PROBE: wait T_serial (300ms) then read EEPROM */
static void
sfp_fdt_sm_probe(struct sfp_fdt_softc *sc)
{
	bool present;

	/* Ensure I2C bus is resolved (may have deferred from attach) */
	if (!sfp_fdt_resolve_i2c(sc)) {
		device_printf(sc->sc_dev,
		    "SFP: I2C bus not available yet\n");
		sc->sc_state = SFP_ST_EMPTY;
		if (sc->sc_txdis != NULL)
			gpio_pin_set_active(sc->sc_txdis, true);
		return;
	}

	/* SFF-8472 T_serial: 300ms after power-up before EEPROM accessible */
	pause_sbt("sfpwait", SBT_1MS * 300, 0, C_PREL(2));

	/* Re-check module presence after sleep */
	if (sc->sc_moddef0 != NULL) {
		gpio_pin_is_active(sc->sc_moddef0, &present);
		if (!present) {
			sc->sc_state = SFP_ST_EMPTY;
			return;
		}
	}

	if (sfp_fdt_read_eeprom(sc) != 0) {
		/* EEPROM read failed — back to EMPTY */
		sc->sc_state = SFP_ST_EMPTY;
		if (sc->sc_txdis != NULL)
			gpio_pin_set_active(sc->sc_txdis, true);
		return;
	}

	sc->sc_connector = sc->sc_id[SFP_CONNECTOR_OFFSET];
	sfp_fdt_log_module(sc);
	sfp_fdt_notify_insert(sc);

	sc->sc_state = SFP_ST_PRESENT;
}

/* PRESENT: classify module → PHY_PROBE (copper) or WAIT_LOS (fiber) */
static void
sfp_fdt_sm_present(struct sfp_fdt_softc *sc)
{

	if (sc->sc_connector == SFP_CONNECTOR_RJ45) {
		sc->sc_phy_retries = 0;
		sc->sc_state = SFP_ST_PHY_PROBE;
	} else {
		/* Fiber or DAC — monitor LOS */
		sc->sc_los_prev = true;	/* assume no signal initially */
		sc->sc_state = SFP_ST_WAIT_LOS;
	}
}

/* PHY_PROBE: probe embedded PHY via RollBall then I2C-MDIO */
static void
sfp_fdt_sm_phy_probe(struct sfp_fdt_softc *sc)
{
	int id1, id2, retry;
	uint32_t phy_id;
	int extable;

	id1 = -1;

	/* Try RollBall first */
	sc->sc_phy_proto = SFP_PHY_ROLLBALL;
	if (rollball_init(sc) == 0) {
		for (retry = 0; retry < SFP_PHY_BOOT_RETRIES; retry++) {
			pause_sbt("phyboot", SBT_1MS * SFP_PHY_BOOT_MS,
			    0, C_PREL(2));

			id1 = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_DEVID1);
			if (id1 > 0 && id1 != 0xFFFF)
				break;
		}
	}

	/* Fallback to standard I2C-MDIO */
	if (id1 <= 0 || id1 == 0xFFFF) {
		sc->sc_phy_proto = SFP_PHY_MDIOI2C;
		id1 = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_DEVID1);
	}

	if (id1 <= 0 || id1 == 0xFFFF) {
		device_printf(sc->sc_dev,
		    "SFP+ PHY: no C45 PHY detected "
		    "(tried RollBall and I2C-MDIO)\n");
		sc->sc_phy_proto = 0;
		sc->sc_has_phy = false;
		/* Fall back to DDM polling for copper without PHY */
		sc->sc_state = SFP_ST_DDM_POLL;
		return;
	}

	id2 = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_DEVID2);
	if (id2 < 0 || id2 == 0xFFFF) {
		device_printf(sc->sc_dev,
		    "SFP+ PHY: DEVID1=0x%04x but DEVID2 read failed\n",
		    id1 & 0xFFFF);
		sc->sc_phy_proto = 0;
		sc->sc_has_phy = false;
		sc->sc_state = SFP_ST_DDM_POLL;
		return;
	}

	phy_id = ((uint32_t)id1 << 16) | id2;

	device_printf(sc->sc_dev,
	    "SFP+ PHY detected via %s: ID %08x "
	    "(OUI %06x model %02x rev %01x)\n",
	    sc->sc_phy_proto == SFP_PHY_ROLLBALL ? "RollBall" : "I2C-MDIO",
	    phy_id,
	    (id1 << 6) | (id2 >> 10),
	    (id2 >> 4) & 0x3F,
	    id2 & 0x0F);

	extable = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_PMA_EXTABLE);
	if (extable >= 0) {
		device_printf(sc->sc_dev,
		    "SFP+ PHY abilities: %s%s%s\n",
		    (extable & MDIO_PMA_EXTABLE_10GBT) ? "10GBASE-T " : "",
		    (extable & MDIO_PMA_EXTABLE_1000BT) ? "1000BASE-T " : "",
		    (extable & MDIO_PMA_EXTABLE_NBT) ? "NBASE-T" : "");
	}

	sc->sc_phy_id = phy_id;
	sc->sc_has_phy = true;
	sc->sc_phy_link = false;
	sc->sc_phy_speed = 0;

	sfp_phy_config_aneg(sc);

	sc->sc_state = SFP_ST_PHY_AN;
}

/* PHY_AN: poll PHY link status */
static void
sfp_fdt_sm_phy_poll(struct sfp_fdt_softc *sc)
{
	int stat, speed;
	bool link_up;

	if (!sc->sc_has_phy)
		return;

	stat = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_STAT1);
	if (stat < 0)
		return;

	link_up = (stat & MDIO_STAT1_LSTATUS) != 0;

	if (link_up != sc->sc_phy_link) {
		sc->sc_phy_link = link_up;

		if (link_up) {
			speed = sfp_phy_read_speed(sc);
			sc->sc_phy_speed = speed;
			if (speed > 0)
				device_printf(sc->sc_dev,
				    "SFP+ PHY: link up at %d Mbps\n", speed);
			else
				device_printf(sc->sc_dev,
				    "SFP+ PHY: link up (speed unknown)\n");
			sfp_fdt_notify_link_up(sc, speed);
			sc->sc_state = SFP_ST_LINK_UP;
		} else {
			sc->sc_phy_speed = 0;
			device_printf(sc->sc_dev, "SFP+ PHY: link down\n");
			sfp_fdt_notify_link_down(sc);
		}
	}

	/* Keep polling for speed if link up but speed not yet known */
	if (link_up && sc->sc_phy_speed == 0) {
		speed = sfp_phy_read_speed(sc);
		if (speed > 0) {
			sc->sc_phy_speed = speed;
			device_printf(sc->sc_dev,
			    "SFP+ PHY: negotiated %d Mbps\n", speed);
		}
	}
}

/* DDM_POLL: poll DDM RX_LOS for copper modules without PHY */
static void
sfp_fdt_sm_ddm_poll(struct sfp_fdt_softc *sc)
{
	uint8_t ddm_status;
	int error;
	bool link_up;

	/* Read DDM status byte (A2h offset 0x6E, bit 1 = RX_LOS) */
	error = sfp_fdt_i2c_read(sc, SFP_DIAG_ADDR, 0x6E,
	    &ddm_status, 1);
	if (error != 0)
		return;

	link_up = (ddm_status & 0x02) == 0;

	if (link_up && sc->sc_state == SFP_ST_DDM_POLL) {
		device_printf(sc->sc_dev, "SFP+ DDM: link up\n");
		sfp_fdt_notify_link_up(sc, 0);
		sc->sc_state = SFP_ST_LINK_UP;
	}
}

/* WAIT_LOS: poll LOS GPIO for fiber/DAC modules */
static void
sfp_fdt_sm_wait_los(struct sfp_fdt_softc *sc)
{
	bool los;

	if (sc->sc_los != NULL) {
		gpio_pin_is_active(sc->sc_los, &los);
		if (!los) {
			/* LOS clear = signal present = link up */
			sc->sc_los_prev = false;
			device_printf(sc->sc_dev, "SFP+ fiber: link up\n");
			sfp_fdt_notify_link_up(sc, 0);
			sc->sc_state = SFP_ST_LINK_UP;
		}
	} else {
		/* No LOS GPIO — assume link up when module present */
		sfp_fdt_notify_link_up(sc, 0);
		sc->sc_state = SFP_ST_LINK_UP;
	}
}

/* LINK_UP: monitor for link loss */
static void
sfp_fdt_sm_link_up(struct sfp_fdt_softc *sc)
{

	if (sc->sc_has_phy) {
		/* PHY-based: check PMA/PMD link status */
		int stat = sfp_phy_read(sc, MDIO_MMD_PMAPMD, MDIO_STAT1);
		if (stat >= 0 && !(stat & MDIO_STAT1_LSTATUS)) {
			sc->sc_phy_link = false;
			sc->sc_phy_speed = 0;
			device_printf(sc->sc_dev, "SFP+ PHY: link down\n");
			sfp_fdt_notify_link_down(sc);
			sc->sc_state = SFP_ST_PHY_AN;
		} else if (stat >= 0 && sc->sc_phy_speed == 0) {
			/* Link up but speed unknown — keep trying */
			int speed = sfp_phy_read_speed(sc);
			if (speed > 0) {
				sc->sc_phy_speed = speed;
				device_printf(sc->sc_dev,
				    "SFP+ PHY: negotiated %d Mbps\n", speed);
			}
		}
	} else if (sc->sc_connector == SFP_CONNECTOR_RJ45) {
		/* DDM-based copper: check RX_LOS */
		uint8_t ddm_status;
		int error = sfp_fdt_i2c_read(sc, SFP_DIAG_ADDR, 0x6E,
		    &ddm_status, 1);
		if (error == 0 && (ddm_status & 0x02)) {
			device_printf(sc->sc_dev, "SFP+ DDM: link down\n");
			sfp_fdt_notify_link_down(sc);
			sc->sc_state = SFP_ST_DDM_POLL;
		}
	} else if (sc->sc_los != NULL) {
		/* Fiber/DAC: check LOS GPIO */
		bool los;
		gpio_pin_is_active(sc->sc_los, &los);
		if (los && !sc->sc_los_prev) {
			sc->sc_los_prev = true;
			device_printf(sc->sc_dev, "SFP+ fiber: link down\n");
			sfp_fdt_notify_link_down(sc);
			sc->sc_state = SFP_ST_WAIT_LOS;
		} else if (!los && sc->sc_los_prev) {
			sc->sc_los_prev = false;
		}
	}
}

/* Module removal — reset all state */
static void
sfp_fdt_sm_remove(struct sfp_fdt_softc *sc)
{
	int prev_state;

	prev_state = sc->sc_state;

	/* Assert TX disable */
	if (sc->sc_txdis != NULL)
		gpio_pin_set_active(sc->sc_txdis, true);

	/* Notify upstream if we had gotten past EEPROM read */
	if (prev_state >= SFP_ST_PRESENT)
		sfp_fdt_notify_remove(sc);

	/* Notify link down if we were up */
	if (prev_state == SFP_ST_LINK_UP)
		sfp_fdt_notify_link_down(sc);

	/* Reset state */
	sc->sc_state = SFP_ST_EMPTY;
	sc->sc_connector = 0;
	sc->sc_has_phy = false;
	sc->sc_phy_proto = 0;
	sc->sc_phy_id = 0;
	sc->sc_phy_link = false;
	sc->sc_phy_speed = 0;
	sc->sc_los_prev = true;
	memset(sc->sc_id, 0, sizeof(sc->sc_id));

	device_printf(sc->sc_dev, "SFP+ module removed\n");
}

/*
 * ============================================================
 * State machine task — runs in taskqueue_thread (sleepable)
 * ============================================================
 */

static void
sfp_fdt_sm_task(void *arg, int pending)
{
	struct sfp_fdt_softc *sc = arg;
	bool present;

	if (sc->sc_moddef0 == NULL)
		return;

	/* Check module presence */
	gpio_pin_is_active(sc->sc_moddef0, &present);

	/* Handle removal from any state */
	if (!present && sc->sc_state != SFP_ST_EMPTY) {
		sfp_fdt_sm_remove(sc);
		return;
	}

	switch (sc->sc_state) {
	case SFP_ST_EMPTY:
		if (present)
			sfp_fdt_sm_insert(sc);
		break;
	case SFP_ST_PROBE:
		sfp_fdt_sm_probe(sc);
		break;
	case SFP_ST_PRESENT:
		sfp_fdt_sm_present(sc);
		break;
	case SFP_ST_PHY_PROBE:
		sfp_fdt_sm_phy_probe(sc);
		break;
	case SFP_ST_PHY_AN:
		sfp_fdt_sm_phy_poll(sc);
		break;
	case SFP_ST_DDM_POLL:
		sfp_fdt_sm_ddm_poll(sc);
		break;
	case SFP_ST_WAIT_LOS:
		sfp_fdt_sm_wait_los(sc);
		break;
	case SFP_ST_LINK_UP:
		sfp_fdt_sm_link_up(sc);
		break;
	}
}

/*
 * 1Hz callout — enqueues state machine task to taskqueue_thread
 * so all I2C and GPIO work happens in sleepable context.
 */
static void
sfp_fdt_poll_callout(void *arg)
{
	struct sfp_fdt_softc *sc = arg;

	taskqueue_enqueue(taskqueue_thread, &sc->sc_sm_task);
	callout_reset(&sc->sc_poll, hz, sfp_fdt_poll_callout, sc);
}

/*
 * ============================================================
 * Device methods
 * ============================================================
 */

static int
sfp_fdt_probe(device_t dev)
{
	phandle_t node;
	ssize_t s;

	node = ofw_bus_get_node(dev);
	if (!ofw_bus_node_is_compatible(node, "sff,sfp"))
		return (ENXIO);

	s = device_get_property(dev, "i2c-bus", &node, sizeof(node),
	    DEVICE_PROP_HANDLE);
	if (s == -1)
		return (ENXIO);

	device_set_desc(dev, "Small Form-factor Pluggable Transceiver");
	return (BUS_PROBE_DEFAULT);
}

static int
sfp_fdt_attach(device_t dev)
{
	struct sfp_fdt_softc *sc;
	device_t xdev;
	ssize_t s;
	int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_node = ofw_bus_get_node(dev);
	sc->sc_state = SFP_ST_EMPTY;

	/* Get I2C bus phandle (resolved lazily on first use) */
	s = device_get_property(dev, "i2c-bus", &sc->sc_i2c_phandle,
	    sizeof(sc->sc_i2c_phandle), DEVICE_PROP_HANDLE);
	if (s == -1) {
		device_printf(dev, "cannot find 'i2c-bus' property\n");
		return (ENXIO);
	}

	/* Register xref FIRST so MAC drivers can find us even if
	 * I2C/GPIO aren't ready yet (they may attach later) */
	error = OF_device_register_xref(OF_xref_from_node(sc->sc_node), dev);
	if (error != 0) {
		device_printf(dev, "failed to register xref\n");
		return (error);
	}

	/* Try to resolve I2C bus now; if not ready, resolve lazily */
	xdev = OF_device_from_xref(OF_xref_from_node(sc->sc_i2c_phandle));
	if (xdev != NULL)
		sc->sc_i2c = xdev;

	/* Acquire GPIOs from DT (all optional) */
	gpio_pin_get_by_ofw_property(dev, sc->sc_node,
	    "mod-def0-gpios", &sc->sc_moddef0);
	gpio_pin_get_by_ofw_property(dev, sc->sc_node,
	    "los-gpios", &sc->sc_los);
	gpio_pin_get_by_ofw_property(dev, sc->sc_node,
	    "tx-disable-gpios", &sc->sc_txdis);

	/* TX disable must be output, initially asserted (no module) */
	if (sc->sc_txdis != NULL) {
		gpio_pin_setflags(sc->sc_txdis, GPIO_PIN_OUTPUT);
		gpio_pin_set_active(sc->sc_txdis, true);
	}

	/* Init mutex, task, and callout */
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev),
	    "SFP state", MTX_DEF);
	TASK_INIT(&sc->sc_sm_task, 0, sfp_fdt_sm_task, sc);
	callout_init(&sc->sc_poll, CALLOUT_MPSAFE);

	if (sc->sc_moddef0 != NULL) {
		device_printf(dev, "SFP+ cage ready (GPIOs OK), "
		    "polling started\n");
		/* Start 1Hz poll */
		callout_reset(&sc->sc_poll, hz, sfp_fdt_poll_callout, sc);
	} else {
		device_printf(dev, "SFP: no mod-def0 GPIO, "
		    "hot-plug disabled\n");
	}

	return (0);
}

static int
sfp_fdt_detach(device_t dev)
{
	struct sfp_fdt_softc *sc;

	sc = device_get_softc(dev);

	callout_drain(&sc->sc_poll);
	taskqueue_drain(taskqueue_thread, &sc->sc_sm_task);
	mtx_destroy(&sc->sc_mtx);

	if (sc->sc_txdis != NULL)
		gpio_pin_release(sc->sc_txdis);
	if (sc->sc_los != NULL)
		gpio_pin_release(sc->sc_los);
	if (sc->sc_moddef0 != NULL)
		gpio_pin_release(sc->sc_moddef0);

	return (0);
}

static int
sfp_fdt_get_i2c_bus(device_t dev, device_t *i2c_bus)
{
	struct sfp_fdt_softc *sc;

	KASSERT(i2c_bus != NULL, ("%s: i2c_bus is NULL", __func__));

	sc = device_get_softc(dev);
	if (!sfp_fdt_resolve_i2c(sc))
		return (ENXIO);

	*i2c_bus = sc->sc_i2c;
	return (0);
}

static int
sfp_fdt_register_upstream(device_t dev, const struct sfp_upstream_ops *ops,
    void *arg)
{
	struct sfp_fdt_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_upstream_ops != NULL) {
		device_printf(dev, "upstream already registered\n");
		return (EBUSY);
	}

	sc->sc_upstream_ops = ops;
	sc->sc_upstream_arg = arg;

	device_printf(dev, "upstream MAC driver registered\n");

	return (0);
}

static device_method_t sfp_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		sfp_fdt_probe),
	DEVMETHOD(device_attach,	sfp_fdt_attach),
	DEVMETHOD(device_detach,	sfp_fdt_detach),

	/* SFF interface */
	DEVMETHOD(sff_get_i2c_bus,	sfp_fdt_get_i2c_bus),
	DEVMETHOD(sff_register_upstream, sfp_fdt_register_upstream),

	DEVMETHOD_END
};

DEFINE_CLASS_0(sfp_fdt, sfp_fdt_driver, sfp_fdt_methods,
    sizeof(struct sfp_fdt_softc));

EARLY_DRIVER_MODULE(sfp_fdt, simplebus, sfp_fdt_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
EARLY_DRIVER_MODULE(sfp_fdt, ofwbus, sfp_fdt_driver, 0, 0,
    BUS_PASS_SUPPORTDEV);
