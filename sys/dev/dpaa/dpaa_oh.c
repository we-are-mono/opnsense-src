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
 * DPAA1 Offline / Header Manipulation port driver.
 *
 * OH ports are FMan port instances without a MAC.  Software enqueues frames
 * via QMan, FMan processes them (parse, classify, manipulate), and the
 * results come back via QMan.  They serve as "on-ramps" to inject
 * software-originated traffic (WiFi, IPsec) into the FMan pipeline.
 *
 * Each OH port probes a "fsl,fman-v3-port-oh" device tree node (child of
 * the fman bus) and creates:
 *   - An FMan offline-parsing port (FM_PORT)
 *   - A default egress FQR (frames out of OH port after processing)
 *   - An error FQR (frames that hit errors during processing)
 *   - An ingress FQR (software enqueues here → FMan OH port dequeues)
 *
 * Consumers (CDX, WiFi, IPsec) use the public API in dpaa_oh.h to:
 *   - Get the FM_PORT handle for PCD attachment
 *   - Enqueue frames for offline processing
 *   - Register callbacks for processed frames
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "opt_dpaa.h"

#include <contrib/ncsw/inc/integrations/dpaa_integration_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_port_ext.h>
#include <contrib/ncsw/inc/xx_ext.h>

#include "fman.h"
#include "fman_chardev.h"
#include "qman.h"
#include "dpaa_oh.h"


/**
 * @group OH port private defines.
 * @{
 */

/* Frame queue parameters — same WQ as dtsec RX/TX */
#define	DPAA_OH_FQR_DFLT_WQ	1
#define	DPAA_OH_FQR_ERR_WQ	1
#define	DPAA_OH_FQR_TX_WQ	1

/* Pool channel for egress FQs (frames back to software) */
#define	DPAA_OH_FQR_DFLT_CHANNEL	e_QM_FQ_CHANNEL_POOL1
#define	DPAA_OH_FQR_ERR_CHANNEL		e_QM_FQ_CHANNEL_POOL1

/* LIODN base (same as dtsec) */
#define	DPAA_OH_LIODN_BASE	0

/*
 * FreeBSD's upstream DTS (qoriq-fman3-0.dtsi) uses hardware port IDs as
 * cell-index for OH ports: port@82000 has cell-index 2, port@83000 has
 * cell-index 3, etc.  NCSW FM_PORT_Config expects a 0-based relative port
 * ID and adds BASE_OH_PORTID internally via SwPortIdToHwPortId().
 *
 * The NXP Linux SDK DTS uses 0-based cell-indices (0, 1, 2, ...) instead,
 * so the Linux wrapper passes cell-index directly without adjustment.
 *
 * We subtract the first OH port cell-index to convert from FreeBSD DTS
 * convention to the 0-based relative ID that NCSW expects.
 */
#define	DPAA_OH_CELL_INDEX_BASE	2

struct dpaa_oh_softc {
	device_t	sc_dev;
	int		sc_port_id;	/* cell-index from DT (2-7) */
	uint32_t	sc_port_hw_id;	/* reg from DT (0x82000, etc.) */
	uint32_t	sc_qman_chan;	/* QMan channel for this port */

	/* FMan handles (from fman parent) */
	vm_offset_t	sc_fm_base;
	t_Handle	sc_fmh;
	t_Handle	sc_pcdh;
	t_Handle	sc_netenvh;

	/* FMan OH port */
	t_Handle	sc_portph;	/* FM_PORT handle */
	uint32_t	sc_data_offset;	/* buffer data offset (prefix size) */

	/* Frame queues */
	t_Handle	sc_dflt_fqr;	/* default egress FQR */
	uint32_t	sc_dflt_fqid;
	t_Handle	sc_err_fqr;	/* error FQR */
	uint32_t	sc_err_fqid;
	t_Handle	sc_tx_fqr;	/* ingress FQR (software → OH) */
};
/** @} */


/**
 * @group FQR callbacks.
 * @{
 */

static void
dpaa_oh_exception_callback(t_Handle app, e_FmPortExceptions exception)
{
	struct dpaa_oh_softc *sc;

	sc = app;
	device_printf(sc->sc_dev, "FMan port exception %d\n", exception);
}

static e_RxStoreResponse
dpaa_oh_dflt_cb(t_Handle app, t_Handle fqr, t_Handle portal,
    uint32_t fqid_off, t_DpaaFD *frame)
{
	struct dpaa_oh_softc *sc;
	void *buf;

	sc = app;

	/*
	 * Default FQ callback — frames that PCD didn't classify.
	 * For now, just drop and return the buffer.
	 * Consumers can override this via dpaa_oh_register_cb().
	 */
	buf = DPAA_FD_GET_ADDR(frame);
	if (buf != NULL) {
		/*
		 * We don't own a buffer pool — the buffer was allocated
		 * by whoever enqueued the frame.  We can't return it to
		 * BMan without knowing which pool it came from.  The
		 * consumer's callback is responsible for buffer lifecycle.
		 * If no consumer is registered, we leak.  This is a
		 * configuration error — OH ports should always have a
		 * consumer or PCD attached.
		 */
		if (bootverbose)
			device_printf(sc->sc_dev,
			    "default FQ: unhandled frame (no consumer)\n");
	}

	return (e_RX_STORE_RESPONSE_CONTINUE);
}

static e_RxStoreResponse
dpaa_oh_err_cb(t_Handle app, t_Handle fqr, t_Handle portal,
    uint32_t fqid_off, t_DpaaFD *frame)
{
	struct dpaa_oh_softc *sc;

	sc = app;
	device_printf(sc->sc_dev, "error frame: status 0x%08x\n",
	    DPAA_FD_GET_STATUS(frame));

	return (e_RX_STORE_RESPONSE_CONTINUE);
}
/** @} */


/**
 * @group Driver probe/attach/detach.
 * @{
 */

static int
dpaa_oh_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "fsl,fman-v3-port-oh"))
		return (ENXIO);

	device_set_desc(dev, "Freescale FMan Offline Parsing Port");

	return (BUS_PROBE_DEFAULT);
}

static int
dpaa_oh_attach(device_t dev)
{
	struct dpaa_oh_softc *sc;
	device_t parent;
	t_FmPortParams params;
	t_FmPortNonRxParams *oh_params;
	t_FmBufferPrefixContent prefix;
	t_Error error;
	phandle_t node;
	pcell_t cell_index;
	pcell_t port_reg;
	t_Handle fqr;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	parent = device_get_parent(dev);
	node = ofw_bus_get_node(dev);

	/* Read cell-index — the hardware port ID for OH ports (2-7) */
	if (OF_getencprop(node, "cell-index", &cell_index,
	    sizeof(cell_index)) <= 0) {
		device_printf(dev, "missing cell-index property\n");
		return (ENXIO);
	}
	sc->sc_port_id = cell_index;

	/* Read reg — hardware register offset within FMan */
	if (OF_getencprop(node, "reg", &port_reg, sizeof(port_reg)) <= 0) {
		device_printf(dev, "missing reg property\n");
		return (ENXIO);
	}
	sc->sc_port_hw_id = port_reg;

	/* Get FMan handles from parent */
	if (fman_get_handle(parent, &sc->sc_fmh) != 0 ||
	    sc->sc_fmh == NULL) {
		device_printf(dev, "couldn't get FMan handle\n");
		return (ENXIO);
	}

	if (fman_get_bushandle(parent, &sc->sc_fm_base) != 0) {
		device_printf(dev, "couldn't get FMan base address\n");
		return (ENXIO);
	}

	fman_get_pcd_handle(parent, &sc->sc_pcdh);
	fman_get_netenv_handle(parent, &sc->sc_netenvh);

	/* Get QMan channel for this OH port */
	sc->sc_qman_chan = fman_qman_channel_id(parent, sc->sc_port_id);
	if (sc->sc_qman_chan == 0) {
		device_printf(dev,
		    "couldn't get QMan channel for port %d\n",
		    sc->sc_port_id);
		return (ENXIO);
	}

	/*
	 * Create frame queues.
	 */

	/* Default egress FQR — processed frames come here */
	fqr = qman_fqr_create(1, DPAA_OH_FQR_DFLT_CHANNEL,
	    DPAA_OH_FQR_DFLT_WQ, false, 0, false, false, true, false,
	    0, 0, 0);
	if (fqr == NULL) {
		device_printf(dev, "couldn't create default FQR\n");
		return (EIO);
	}
	sc->sc_dflt_fqr = fqr;
	sc->sc_dflt_fqid = qman_fqr_get_base_fqid(fqr);

	error = qman_fqr_register_cb(fqr, dpaa_oh_dflt_cb, sc);
	if (error != E_OK) {
		device_printf(dev, "couldn't register default FQ callback\n");
		goto fail_dflt;
	}

	/* Error FQR */
	fqr = qman_fqr_create(1, DPAA_OH_FQR_ERR_CHANNEL,
	    DPAA_OH_FQR_ERR_WQ, false, 0, false, false, true, false,
	    0, 0, 0);
	if (fqr == NULL) {
		device_printf(dev, "couldn't create error FQR\n");
		goto fail_dflt;
	}
	sc->sc_err_fqr = fqr;
	sc->sc_err_fqid = qman_fqr_get_base_fqid(fqr);

	error = qman_fqr_register_cb(fqr, dpaa_oh_err_cb, sc);
	if (error != E_OK) {
		device_printf(dev, "couldn't register error FQ callback\n");
		goto fail_err;
	}

	/* Ingress FQR — software enqueues here, FMan OH port dequeues */
	fqr = qman_fqr_create(1, sc->sc_qman_chan,
	    DPAA_OH_FQR_TX_WQ, false, 0, false, false, true, false,
	    0, 0, 0);
	if (fqr == NULL) {
		device_printf(dev, "couldn't create ingress FQR\n");
		goto fail_err;
	}
	sc->sc_tx_fqr = fqr;

	/*
	 * Configure FMan OH port.
	 *
	 * OH ports use nonRxParams (like TX ports) because they receive
	 * frames from QMan (not from a MAC).  But they also need dfltFqid
	 * (like RX ports) for the egress path.
	 */
	memset(&params, 0, sizeof(params));
	params.baseAddr = sc->sc_fm_base + sc->sc_port_hw_id;
	params.h_Fm = sc->sc_fmh;
	params.portType = e_FM_PORT_TYPE_OH_OFFLINE_PARSING;
	params.portId = sc->sc_port_id - DPAA_OH_CELL_INDEX_BASE;
	params.independentModeEnable = false;
	params.liodnBase = DPAA_OH_LIODN_BASE;
	params.f_Exception = dpaa_oh_exception_callback;
	params.h_App = sc;

	oh_params = &params.specificParams.nonRxParams;
	oh_params->errFqid = sc->sc_err_fqid;
	oh_params->dfltFqid = sc->sc_dflt_fqid;
	oh_params->qmChannel = sc->sc_qman_chan;
#ifdef FM_OP_PARTITION_ERRATA_FMANx8
	oh_params->opLiodnOffset = 0;
#endif

	sc->sc_portph = FM_PORT_Config(&params);
	if (sc->sc_portph == NULL) {
		device_printf(dev, "couldn't configure FM OH port\n");
		goto fail_tx;
	}

	/*
	 * Configure buffer prefix: parse result + hash result.
	 * This enables PCD (parser writes results into prefix) and
	 * allows consumers to read parse results for classification.
	 *
	 * manipExtraSpace = 96 matches the Linux SDK DTS
	 * buffer-layout = <0x60 0x40> for OH port extended-args.
	 */
	memset(&prefix, 0, sizeof(prefix));
	prefix.privDataSize = 16;
	prefix.passPrsResult = TRUE;
	prefix.passHashResult = TRUE;
	prefix.dataAlign = 64;
	prefix.manipExtraSpace = 96;

	error = FM_PORT_ConfigBufferPrefixContent(sc->sc_portph, &prefix);
	if (error != E_OK) {
		device_printf(dev, "couldn't configure buffer prefix\n");
		goto fail_port;
	}

	error = FM_PORT_Init(sc->sc_portph);
	if (error != E_OK) {
		device_printf(dev, "couldn't initialize FM OH port: %d\n",
		    error);
		goto fail_port;
	}

	sc->sc_data_offset = FM_PORT_GetBufferDataOffset(sc->sc_portph);

	/* Register OH port with FMD chardev for userspace PCD control */
	{
		struct fmcd_softc *fmcd;

		fmcd = fman_get_fmcd(device_get_parent(dev));
		if (fmcd != NULL)
			fmcd_register_oh_port(fmcd,
			    sc->sc_port_id - DPAA_OH_CELL_INDEX_BASE,
			    sc->sc_portph);
	}

	device_printf(dev,
	    "OH port %d: QMan chan 0x%x, dflt FQID %u, err FQID %u, "
	    "data offset %u\n",
	    sc->sc_port_id, sc->sc_qman_chan, sc->sc_dflt_fqid,
	    sc->sc_err_fqid, sc->sc_data_offset);

	return (0);

fail_port:
	FM_PORT_Free(sc->sc_portph);
	sc->sc_portph = NULL;
fail_tx:
	qman_fqr_free(sc->sc_tx_fqr);
	sc->sc_tx_fqr = NULL;
fail_err:
	qman_fqr_free(sc->sc_err_fqr);
	sc->sc_err_fqr = NULL;
fail_dflt:
	qman_fqr_free(sc->sc_dflt_fqr);
	sc->sc_dflt_fqr = NULL;
	return (ENXIO);
}

static int
dpaa_oh_detach(device_t dev)
{
	struct dpaa_oh_softc *sc;

	sc = device_get_softc(dev);

	if (sc->sc_portph != NULL) {
		FM_PORT_Free(sc->sc_portph);
		sc->sc_portph = NULL;
	}

	if (sc->sc_tx_fqr != NULL)
		qman_fqr_free(sc->sc_tx_fqr);
	if (sc->sc_err_fqr != NULL)
		qman_fqr_free(sc->sc_err_fqr);
	if (sc->sc_dflt_fqr != NULL)
		qman_fqr_free(sc->sc_dflt_fqr);

	return (0);
}
/** @} */


/**
 * @group Distribution FQ callback registry.
 *
 * BPID-keyed table for OH port consumers to register distribution FQ
 * handlers.  CDX calls dpaa_oh_lookup_dist_cb() in its distribution
 * FQ callback to dispatch frames to the appropriate consumer.
 * @{
 */
#define	DPAA_OH_MAX_DIST_CBS	4

static struct {
	uint8_t		bpid;
	bool		active;
	dpaa_oh_dist_cb_t fn;
	t_Handle	app;
} dpaa_oh_dist_cbs[DPAA_OH_MAX_DIST_CBS];

int
dpaa_oh_register_dist_cb(uint8_t bpid, dpaa_oh_dist_cb_t fn, t_Handle app)
{
	int i;

	for (i = 0; i < DPAA_OH_MAX_DIST_CBS; i++) {
		if (!dpaa_oh_dist_cbs[i].active) {
			dpaa_oh_dist_cbs[i].bpid = bpid;
			dpaa_oh_dist_cbs[i].fn = fn;
			dpaa_oh_dist_cbs[i].app = app;
			atomic_thread_fence_rel();
			dpaa_oh_dist_cbs[i].active = true;
			printf("dpaa_oh: dist cb registered for bpid %u\n",
			    bpid);
			return (0);
		}
	}
	return (ENOSPC);
}

void
dpaa_oh_unregister_dist_cb(uint8_t bpid)
{
	int i;

	for (i = 0; i < DPAA_OH_MAX_DIST_CBS; i++) {
		if (dpaa_oh_dist_cbs[i].active &&
		    dpaa_oh_dist_cbs[i].bpid == bpid) {
			dpaa_oh_dist_cbs[i].active = false;
			atomic_thread_fence_rel();
			printf("dpaa_oh: dist cb unregistered for bpid %u\n",
			    bpid);
			return;
		}
	}
}

dpaa_oh_dist_cb_t
dpaa_oh_lookup_dist_cb(uint8_t bpid, t_Handle *app)
{
	int i;

	for (i = 0; i < DPAA_OH_MAX_DIST_CBS; i++) {
		if (dpaa_oh_dist_cbs[i].active &&
		    dpaa_oh_dist_cbs[i].bpid == bpid) {
			*app = dpaa_oh_dist_cbs[i].app;
			return (dpaa_oh_dist_cbs[i].fn);
		}
	}
	return (NULL);
}
/** @} */


/**
 * @group Public API.
 * @{
 */

device_t
dpaa_oh_find_port(int cell_index)
{
	devclass_t dc;
	device_t *devlist;
	int count, i;
	device_t result;

	dc = devclass_find("dpaa_oh");
	if (dc == NULL)
		return (NULL);

	if (devclass_get_devices(dc, &devlist, &count) != 0)
		return (NULL);

	result = NULL;
	for (i = 0; i < count; i++) {
		struct dpaa_oh_softc *sc;

		sc = device_get_softc(devlist[i]);
		if (sc->sc_port_id == cell_index) {
			result = devlist[i];
			break;
		}
	}

	free(devlist, M_TEMP);
	return (result);
}

t_Handle
dpaa_oh_get_fm_port(device_t dev)
{
	struct dpaa_oh_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_portph);
}

uint32_t
dpaa_oh_get_qman_channel(device_t dev)
{
	struct dpaa_oh_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_qman_chan);
}

uint32_t
dpaa_oh_get_default_fqid(device_t dev)
{
	struct dpaa_oh_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_dflt_fqid);
}

uint32_t
dpaa_oh_get_data_offset(device_t dev)
{
	struct dpaa_oh_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_data_offset);
}

int
dpaa_oh_enqueue(device_t dev, t_DpaaFD *fd)
{
	struct dpaa_oh_softc *sc;
	t_Error error;

	sc = device_get_softc(dev);
	if (sc->sc_tx_fqr == NULL)
		return (ENXIO);

	error = qman_fqr_enqueue(sc->sc_tx_fqr, 0, fd);
	if (error != E_OK)
		return (EIO);

	return (0);
}

int
dpaa_oh_register_cb(device_t dev, t_QmReceivedFrameCallback *callback,
    t_Handle app)
{
	struct dpaa_oh_softc *sc;
	t_Error error;

	sc = device_get_softc(dev);
	if (sc->sc_dflt_fqr == NULL)
		return (ENXIO);

	error = qman_fqr_register_cb(sc->sc_dflt_fqr, callback, app);
	if (error != E_OK)
		return (EIO);

	return (0);
}
/** @} */


/**
 * @group Device driver boilerplate.
 * @{
 */

static device_method_t dpaa_oh_methods[] = {
	DEVMETHOD(device_probe,		dpaa_oh_probe),
	DEVMETHOD(device_attach,	dpaa_oh_attach),
	DEVMETHOD(device_detach,	dpaa_oh_detach),
	{ 0, 0 }
};

static driver_t dpaa_oh_driver = {
	"dpaa_oh",
	dpaa_oh_methods,
	sizeof(struct dpaa_oh_softc),
};

DRIVER_MODULE(dpaa_oh, fman, dpaa_oh_driver, 0, 0);
MODULE_VERSION(dpaa_oh, 1);
/** @} */
