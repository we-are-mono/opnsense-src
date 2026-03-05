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

#ifndef DPAA_OH_H_
#define DPAA_OH_H_

#include <contrib/ncsw/inc/ncsw_ext.h>
#include <contrib/ncsw/inc/integrations/dpaa_integration_ext.h>
#include <contrib/ncsw/inc/Peripherals/dpaa_ext.h>
#include <contrib/ncsw/inc/Peripherals/qm_ext.h>

/**
 * Find an OH port device by its cell-index (hardware port ID).
 * Cell-index values are 2-7 for FManv3 OH ports.
 *
 * @param cell_index	The cell-index from device tree.
 * @return		The device_t, or NULL if not found.
 */
device_t	dpaa_oh_find_port(int cell_index);

/**
 * Get the FM_PORT handle for PCD attachment.
 * Consumers (e.g., CDX) use this to call FM_PORT_SetPCD().
 */
t_Handle	dpaa_oh_get_fm_port(device_t dev);

/**
 * Get the QMan channel ID for this OH port.
 * Consumers create FQs with this channel as destination to send
 * frames to the OH port for processing.
 */
uint32_t	dpaa_oh_get_qman_channel(device_t dev);

/**
 * Get the default (egress) FQID.
 * Frames that pass through the OH port without PCD classification
 * are enqueued to this FQID.
 */
uint32_t	dpaa_oh_get_default_fqid(device_t dev);

/**
 * Get the buffer data offset (prefix size).
 * When building frames to send to the OH port, data must start
 * at this offset within the buffer to leave room for the prefix.
 */
uint32_t	dpaa_oh_get_data_offset(device_t dev);

/**
 * Enqueue a frame descriptor to the OH port for processing.
 *
 * @param dev	The OH port device.
 * @param fd	The frame descriptor to enqueue.
 * @return	0 on success, errno on failure.
 */
int		dpaa_oh_enqueue(device_t dev, t_DpaaFD *fd);

/**
 * Register a callback for frames exiting the OH port's default FQ.
 * These are "miss" frames that PCD did not classify into a flow.
 *
 * @param dev		The OH port device.
 * @param callback	The callback function (same signature as QMan RX cb).
 * @param app		Opaque argument passed to callback.
 * @return		0 on success, errno on failure.
 */
int		dpaa_oh_register_cb(device_t dev,
		    t_QmReceivedFrameCallback *callback, t_Handle app);

/**
 * Distribution FQ callback type.
 * Same signature as t_QmReceivedFrameCallback.
 */
typedef e_RxStoreResponse (*dpaa_oh_dist_cb_t)(t_Handle app,
    t_Handle fqr, t_Handle portal, uint32_t fqid_off, t_DpaaFD *fd);

/**
 * Register a distribution FQ callback keyed by BMan pool ID.
 *
 * When CDX takes over PCD, frames matching KeyGen distribution
 * schemes on OH ports arrive at CDX distribution FQs.  OH port
 * consumers (WiFi, IPsec) register here so CDX can dispatch
 * these frames instead of dropping them.
 *
 * @param bpid	BMan pool ID that identifies the consumer's frames.
 * @param fn	Callback function for frame delivery.
 * @param app	Opaque argument passed to callback.
 * @return	0 on success, ENOSPC if registry is full.
 */
int		dpaa_oh_register_dist_cb(uint8_t bpid,
		    dpaa_oh_dist_cb_t fn, t_Handle app);
void		dpaa_oh_unregister_dist_cb(uint8_t bpid);
dpaa_oh_dist_cb_t dpaa_oh_lookup_dist_cb(uint8_t bpid, t_Handle *app);

#endif /* DPAA_OH_H_ */
