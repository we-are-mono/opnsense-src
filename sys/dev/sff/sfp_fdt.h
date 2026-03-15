/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
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

#ifndef _SFP_FDT_H_
#define _SFP_FDT_H_

/*
 * SFP upstream operations — callbacks from sfp_fdt to MAC driver.
 * MAC driver registers these via SFF_REGISTER_UPSTREAM().
 * All callbacks run in taskqueue_thread context (sleepable).
 */
struct sfp_upstream_ops {
	/*
	 * Module inserted.  Called after EEPROM read succeeds.
	 * id points to 64-byte SFP EEPROM A0h base ID data.
	 * Return 0 to accept, non-zero to reject the module.
	 */
	int	(*module_insert)(void *arg, const uint8_t *id, int id_len);

	/*
	 * Module removed.  Clean up any per-module state.
	 */
	void	(*module_remove)(void *arg);

	/*
	 * Link came up.  speed is in Mbps (10000, 5000, 2500, 1000).
	 * For fiber/DAC modules speed is 0 (unknown/fixed).
	 */
	void	(*link_up)(void *arg, int speed);

	/*
	 * Link went down.
	 */
	void	(*link_down)(void *arg);
};

/* SFF-8024 connector types */
#define	SFP_CONNECTOR_LC	0x07
#define	SFP_CONNECTOR_RJ45	0x22

/* SFF-8472 EEPROM A0h field offsets */
#define	SFP_ID_OFFSET		0
#define	SFP_CONNECTOR_OFFSET	2
#define	SFP_VENDOR_OFFSET	20
#define	SFP_VENDOR_LEN		16
#define	SFP_PARTNUM_OFFSET	40
#define	SFP_PARTNUM_LEN		16

/* I2C addresses */
#define	SFP_EEPROM_ADDR		0x50	/* EEPROM A0h page (7-bit) */
#define	SFP_DIAG_ADDR		0x51	/* A2h / RollBall (7-bit) */
#define	SFP_PHY_ADDR		0x56	/* Standard I2C-MDIO (7-bit) */

#endif /* _SFP_FDT_H_ */
