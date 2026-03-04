/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Mono Technologies Inc.
 * All rights reserved.
 *
 * FMan character device interface — exposes NCSW PCD API to userspace.
 *
 * Ioctl command numbers and data structures are ABI-compatible with the
 * Linux NXP SDK FMD driver so that the same fmlib/FMC userspace tools
 * work unchanged.
 *
 * Handle management: NCSW returns opaque kernel pointers (t_Handle).
 * Userspace receives opaque integer handle IDs via a kernel handle
 * table — kernel pointers are never exposed.  The ioc structs have
 * identical binary layouts to the NCSW structs (with a trailing
 * void *id field); the kernel translates handle IDs to/from kernel
 * pointers at the ioctl boundary.
 *
 * The ioctl struct definitions come from the fmlib ioctl headers
 * (fm_pcd_ioctls.h, fm_port_ioctls.h) which are included directly
 * by fman_chardev.c via the FMCD_COMPILE_CMD include paths.
 * This header only exposes kernel-private definitions.
 */

#ifndef FMAN_CHARDEV_H_
#define FMAN_CHARDEV_H_

/*
 * Device minor number layout.
 * For FMan instance 0:
 *   minor 0  = /dev/fm0              (FM management)
 *   minor 1  = /dev/fm0-pcd          (PCD operations)
 *   minor 2  = /dev/fm0-port-oh0     (OH port, cell-index 3)
 *   minor 3  = /dev/fm0-port-oh1     (OH port, cell-index 4)
 *   minor 8-15 = /dev/fm0-port-rx0..rx7 (RX ports, fmlib convention:
 *                 0-5 = 1G, 6-7 = 10G)
 */
#define FMD_MINOR_FM			0
#define FMD_MINOR_PCD			1
#define FMD_MINOR_OH_BASE		2
#define FMD_MINOR_RX_BASE		8	/* after 6 OH slots */
#define FMD_MINOR_MAX			16	/* 8 + 8 RX port slots */

/* ------------------------------------------------------------------ */
/*  Kernel-only: chardev state and functions                           */
/* ------------------------------------------------------------------ */

#ifdef _KERNEL

#include <sys/conf.h>
#include <sys/sx.h>
#include <contrib/ncsw/inc/ncsw_ext.h>

/*
 * PCD handle table — maps opaque integer IDs (exposed to userspace)
 * to kernel pointers (used internally by NCSW).
 *
 * Handle ID encoding (uint64_t, same width as void* on arm64):
 *   bits [6:0]   = slot index (0..FMCD_MAX_HANDLES-1)
 *   bits [31:7]  = generation counter (invalidates stale handles)
 *   bits [63:32] = 0 (ensures ID never resembles a kernel pointer)
 *
 * Value 0 is reserved as "invalid/NULL handle".  handle_gen starts
 * at 1 so slot 0's first ID is (1 << 7) | 0 = 128, not 0.
 */
enum fmcd_handle_type {
	FMCD_HDL_NONE = 0,
	FMCD_HDL_NET_ENV,
	FMCD_HDL_KG_SCHEME,
	FMCD_HDL_CC_ROOT,
	FMCD_HDL_MATCH_TABLE,
	FMCD_HDL_HASH_TABLE,
	FMCD_HDL_MANIP,
	FMCD_HDL_PLCR_PROFILE,
	FMCD_HDL_FRM_REPLIC,
};

struct fmcd_handle_entry {
	t_Handle		ptr;	/* kernel pointer (NULL = free slot) */
	enum fmcd_handle_type	type;
};

#define	FMCD_MAX_HANDLES	128
#define	FMCD_HANDLE_MASK	0x7f	/* slot index in lower 7 bits */
#define	FMCD_HANDLE_SHIFT	7	/* generation counter above */

/*
 * Per-FMan chardev state.
 */
struct fmcd_softc {
	device_t		fman_dev;	/* parent fman device */
	struct cdev		*cdev_fm;	/* /dev/fm0 */
	struct cdev		*cdev_pcd;	/* /dev/fm0-pcd */
	struct cdev		*cdev_port_rx[8]; /* /dev/fm0-port-rx[0-7] */
	struct cdev		*cdev_port_oh[6]; /* /dev/fm0-port-oh[0-5] */

	t_Handle		fm_handle;	/* NCSW FM handle */
	t_Handle		pcd_handle;	/* NCSW PCD handle */

	/* Port handle registry — indexed by fmlib convention:
	 * 0-5 = 1G MACs (cell-index 0-5), 6-7 = 10G MACs (cell-index 8-9).
	 * fmcd_cell_to_rxidx() in fman_chardev.c converts DT cell-index. */
	struct sx		port_lock;
	t_Handle		rx_port_handles[8];
	t_Handle		rx_port_schemes[8]; /* dtsec KG scheme handles */
	t_Handle		*rx_port_scheme_ptrs[8]; /* ptr to dtsec sc_scheme */
	uint32_t		rx_port_fqids[8]; /* dtsec default RX FQIDs */
	t_Handle		oh_port_handles[6];

	/* PCD handle table — protected by port_lock */
	struct fmcd_handle_entry handles[FMCD_MAX_HANDLES];
	uint32_t		handle_gen;	/* generation counter */
	bool			pcd_transferred; /* CDX has taken PCD ownership */
};

/*
 * Chardev lifecycle.
 */
int	fmcd_init(device_t fman_dev, struct fmcd_softc *sc);
void	fmcd_destroy(struct fmcd_softc *sc);

/*
 * Port registration (called by dtsec and dpaa_oh during attach).
 */
void	fmcd_register_rx_port(struct fmcd_softc *sc, int index,
	    t_Handle h_port, uint32_t dflt_fqid);
void	fmcd_register_oh_port(struct fmcd_softc *sc, int index,
	    t_Handle h_port);
uint32_t fmcd_get_rx_dflt_fqid(struct fmcd_softc *sc, int index);

/*
 * Register dtsec's KG scheme handle for PCD teardown.
 * p_scheme points to dtsec_softc->sc_scheme so the chardev can
 * clear it when userspace takes over PCD management.
 */
void	fmcd_register_rx_port_scheme(struct fmcd_softc *sc, int index,
	    t_Handle h_scheme, t_Handle *p_scheme);

/*
 * Mark PCD resources as transferred to CDX.  After this call,
 * fmcd_pcd_close() will not attempt to delete NCSW PCD objects.
 */
void	fmcd_pcd_mark_transferred(struct fmcd_softc *sc);

/*
 * Resolve a handle ID to a kernel pointer (read-only, does not free).
 * Used by CDX to translate userspace handle IDs to NCSW pointers.
 */
t_Handle fmcd_handle_resolve(struct fmcd_softc *sc, uint64_t id,
	    enum fmcd_handle_type expected_type);

#endif /* _KERNEL */

#endif /* FMAN_CHARDEV_H_ */
