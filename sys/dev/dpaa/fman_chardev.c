/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Mono Technologies Inc.
 * All rights reserved.
 *
 * FMan character device — exposes NCSW PCD API to userspace via ioctls.
 *
 * Creates /dev/fm0, /dev/fm0-pcd, /dev/fm0-port-rx*, /dev/fm0-port-oh*
 * character devices.  Userspace tools (fmlib, FMC, dpa_app) open these devices
 * and issue ioctls that map 1:1 to NCSW kernel functions:
 *
 *   FM_PCD_IOC_KG_SCHEME_SET    →  FM_PCD_KgSchemeSet()
 *   FM_PCD_IOC_HASH_TABLE_SET   →  FM_PCD_HashTableSet()
 *   FM_PCD_IOC_CC_ROOT_BUILD    →  FM_PCD_CcRootBuild()
 *   FM_PORT_IOC_SET_PCD         →  FM_PORT_SetPCD()
 *   ... etc.
 *
 * Handle management: NCSW returns opaque kernel pointers (t_Handle).
 * A per-device handle table maps these to opaque integer IDs that are
 * exposed to userspace — kernel pointers are never leaked.  The ioc
 * structs have identical binary layouts to the NCSW structs (plus a
 * trailing void *id for output handles); the kernel translates handle
 * IDs to/from kernel pointers at the ioctl boundary.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/sx.h>
#include <sys/uio.h>
#include <sys/ioccom.h>
#include <sys/priv.h>

#include "opt_dpaa.h"

#include <contrib/ncsw/inc/ncsw_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_pcd_ext.h>
#include <contrib/ncsw/inc/Peripherals/fm_port_ext.h>
#include <contrib/ncsw/Peripherals/FM/Port/fm_port.h>
#include <contrib/ncsw/inc/net_ext.h>

/*
 * fmlib ioctl ABI headers — authoritative struct/ioctl definitions.
 * Located in contrib/ncsw/inc/ioctls/ (included via DPAA_COMPILE_CMD).
 */
#include "fm_pcd_ioctls.h"

#include "fman.h"
#include "fman_chardev.h"

static MALLOC_DEFINE(M_FMCD, "fmcd", "FMan chardev data");

/*
 * Port PCD types and ioctl commands.
 *
 * These come from fm_port_ioctls.h in fmlib, but we cannot include that
 * header because it pulls in fmlib's "enet_ext.h" which conflicts with
 * the NCSW kernel copy (contrib/ncsw/inc/enet_ext.h).  The types below
 * are copied verbatim from fm_port_ioctls.h and must stay in sync.
 */
typedef enum ioc_fm_port_pcd_support {
	e_IOC_FM_PORT_PCD_SUPPORT_NONE = 0,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_ONLY,
	e_IOC_FM_PORT_PCD_SUPPORT_PLCR_ONLY,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_PLCR,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_CC_AND_PLCR,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_KG_AND_PLCR,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_CC,
	e_IOC_FM_PORT_PCD_SUPPORT_PRS_AND_CC_AND_PLCR,
	e_IOC_FM_PORT_PCD_SUPPORT_CC_ONLY,
} ioc_fm_port_pcd_support;

typedef union ioc_fm_pcd_hdr_prs_opts_u {
	struct {
		bool			label_interpretation_enable;
		ioc_net_header_type	next_parse;
	} mpls_prs_options;
	struct {
		uint16_t		tag_protocol_id1;
		uint16_t		tag_protocol_id2;
	} vlan_prs_options;
	struct {
		bool			enable_mtu_check;
	} pppoe_prs_options;
	struct {
		bool			routing_hdr_disable;
	} ipv6_prs_options;
	struct {
		bool			pad_ignore_checksum;
	} udp_prs_options;
	struct {
		bool			pad_ignore_checksum;
	} tcp_prs_options;
} ioc_fm_pcd_hdr_prs_opts_u;

typedef struct ioc_fm_pcd_prs_additional_hdr_params_t {
	ioc_net_header_type		hdr;
	bool				err_disable;
	bool				soft_prs_enable;
	uint8_t				index_per_hdr;
	bool				use_prs_opts;
	ioc_fm_pcd_hdr_prs_opts_u	prs_opts;
} ioc_fm_pcd_prs_additional_hdr_params_t;

typedef struct ioc_fm_port_pcd_prs_params_t {
	uint8_t				prs_res_priv_info;
	uint8_t				parsing_offset;
	ioc_net_header_type		first_prs_hdr;
	bool				include_in_prs_statistics;
	uint8_t				num_of_hdrs_with_additional_params;
	ioc_fm_pcd_prs_additional_hdr_params_t
	    additional_params[IOC_FM_PCD_PRS_NUM_OF_HDRS];
	bool				set_vlan_tpid1;
	uint16_t			vlan_tpid1;
	bool				set_vlan_tpid2;
	uint16_t			vlan_tpid2;
} ioc_fm_port_pcd_prs_params_t;

typedef struct ioc_fm_port_pcd_cc_params_t {
	void	*cc_tree_id;
} ioc_fm_port_pcd_cc_params_t;

typedef struct ioc_fm_port_pcd_kg_params_t {
	uint8_t		num_of_schemes;
	void		*scheme_ids[FM_PCD_KG_NUM_OF_SCHEMES];
	bool		direct_scheme;
	void		*direct_scheme_id;
} ioc_fm_port_pcd_kg_params_t;

typedef struct ioc_fm_port_pcd_plcr_params_t {
	void	*plcr_profile_id;
} ioc_fm_port_pcd_plcr_params_t;

typedef struct ioc_fm_port_pcd_params_t {
	ioc_fm_port_pcd_support		pcd_support;
	void				*net_env_id;
	ioc_fm_port_pcd_prs_params_t	*p_prs_params;
	ioc_fm_port_pcd_cc_params_t	*p_cc_params;
	ioc_fm_port_pcd_kg_params_t	*p_kg_params;
	ioc_fm_port_pcd_plcr_params_t	*p_plcr_params;
	void				*p_ip_reassembly_manip;
	void				*p_capwap_reassembly_manip;
} ioc_fm_port_pcd_params_t;

#define FM_PORT_IOC_DISABLE	_IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(1))
#define FM_PORT_IOC_ENABLE	_IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(2))
#define FM_PORT_IOC_SET_PCD \
	_IOW(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(20), ioc_fm_port_pcd_params_t)
#define FM_PORT_IOC_DELETE_PCD	_IO(FM_IOC_TYPE_BASE, FM_PORT_IOC_NUM(21))

/* ------------------------------------------------------------------ */
/*  Struct layout assertions                                           */
/* ------------------------------------------------------------------ */

/*
 * The ioc structs must have identical binary layout to the corresponding
 * NCSW structs, with only a trailing void *id output field appended.
 * This is verified at compile time, matching Linux's ASSERT_COND pattern.
 *
 * roundup2() accounts for alignment padding the compiler may insert
 * before the trailing void *id pointer (e.g. NetEnvParams: NCSW=964,
 * ioc=976 because of 4 bytes padding to align void *id to 8 bytes).
 */
#define	FMCD_IOC_SIZE(ncsw_t) \
	roundup2(sizeof(ncsw_t) + sizeof(void *), _Alignof(void *))

_Static_assert(
    sizeof(ioc_fm_pcd_net_env_params_t) == FMCD_IOC_SIZE(t_FmPcdNetEnvParams),
    "NetEnvParams size mismatch — ioc and NCSW struct layouts diverged");

_Static_assert(
    sizeof(ioc_fm_pcd_kg_scheme_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdKgSchemeParams),
    "KgSchemeParams size mismatch — ioc and NCSW struct layouts diverged");

_Static_assert(
    sizeof(ioc_fm_pcd_cc_tree_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdCcTreeParams),
    "CcTreeParams size mismatch — ioc and NCSW struct layouts diverged");

_Static_assert(
    sizeof(ioc_fm_pcd_cc_node_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdCcNodeParams),
    "CcNodeParams size mismatch — ioc and NCSW struct layouts diverged");

_Static_assert(
    sizeof(ioc_fm_pcd_hash_table_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdHashTableParams),
    "HashTableParams size mismatch — ioc and NCSW struct layouts diverged");

_Static_assert(
    sizeof(ioc_fm_pcd_manip_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdManipParams),
    "ManipParams size mismatch — ioc and NCSW struct layouts diverged");

_Static_assert(
    sizeof(ioc_fm_pcd_plcr_profile_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdPlcrProfileParams),
    "PlcrProfileParams size mismatch — ioc and NCSW struct layouts diverged");

#if (DPAA_VERSION >= 11)
_Static_assert(
    sizeof(ioc_fm_pcd_frm_replic_group_params_t) ==
    FMCD_IOC_SIZE(t_FmPcdFrmReplicGroupParams),
    "FrmReplicGroupParams size mismatch — ioc and NCSW struct layouts diverged");
#endif

/* Port PCD struct: no trailing id field, sizes must match exactly. */
_Static_assert(
    sizeof(ioc_fm_port_pcd_params_t) == sizeof(t_FmPortPcdParams),
    "PortPcdParams size mismatch — ioc and NCSW struct layouts diverged");

/* ------------------------------------------------------------------ */
/*  Port registration                                                  */
/* ------------------------------------------------------------------ */

/*
 * Convert DT cell-index to fmlib RX port index.
 *
 * fmlib convention (matching Linux SDK):
 *   1G MACs:  cell-index 0-5 → port index 0-5
 *   10G MACs: cell-index 8-9 → port index 6-7
 */
#define FMCD_1G_RX_PORTS	6

static inline int
fmcd_cell_to_rxidx(int cell_index)
{
	if (cell_index >= 8) /* 10G MAC */
		return (FMCD_1G_RX_PORTS + (cell_index - 8));
	return (cell_index); /* 1G MAC */
}

void
fmcd_register_rx_port(struct fmcd_softc *sc, int cell_index, t_Handle h_port,
    uint32_t dflt_fqid)
{
	int index = fmcd_cell_to_rxidx(cell_index);

	if (sc == NULL || index < 0 || index >= 8)
		return;

	sx_xlock(&sc->port_lock);
	sc->rx_port_handles[index] = h_port;
	sc->rx_port_fqids[index] = dflt_fqid;
	if (sc->cdev_port_rx[index] != NULL)
		sc->cdev_port_rx[index]->si_drv2 = h_port;
	sx_xunlock(&sc->port_lock);

	device_printf(sc->fman_dev,
	    "fmcd: rx port registered: cell=%d idx=%d fqid=%u\n",
	    cell_index, index, dflt_fqid);
}

uint32_t
fmcd_get_rx_dflt_fqid(struct fmcd_softc *sc, int index)
{

	if (sc == NULL || index < 0 || index >= 8)
		return (0);
	return (sc->rx_port_fqids[index]);
}

void
fmcd_register_rx_port_scheme(struct fmcd_softc *sc, int cell_index,
    t_Handle h_scheme, t_Handle *p_scheme)
{
	int index = fmcd_cell_to_rxidx(cell_index);

	if (sc == NULL || index < 0 || index >= 8)
		return;

	sx_xlock(&sc->port_lock);
	sc->rx_port_schemes[index] = h_scheme;
	sc->rx_port_scheme_ptrs[index] = p_scheme;
	sx_xunlock(&sc->port_lock);
}

void
fmcd_register_oh_port(struct fmcd_softc *sc, int index, t_Handle h_port)
{
	if (sc == NULL || index < 0 || index >= 6)
		return;

	sx_xlock(&sc->port_lock);
	sc->oh_port_handles[index] = h_port;
	if (sc->cdev_port_oh[index] != NULL)
		sc->cdev_port_oh[index]->si_drv2 = h_port;
	sx_xunlock(&sc->port_lock);
}

void
fmcd_pcd_mark_transferred(struct fmcd_softc *sc)
{

	sx_xlock(&sc->port_lock);
	sc->pcd_transferred = true;
	sx_xunlock(&sc->port_lock);
}

/* ------------------------------------------------------------------ */
/*  Handle table helpers                                               */
/* ------------------------------------------------------------------ */

/*
 * Allocate a handle slot, store the kernel pointer and type.
 * Returns the opaque handle ID, or 0 on failure (table full).
 * Caller must hold sc->port_lock.
 */
static uint64_t
fmcd_handle_alloc(struct fmcd_softc *sc, t_Handle ptr,
    enum fmcd_handle_type type)
{
	int i;

	KASSERT(ptr != NULL, ("fmcd_handle_alloc: NULL ptr"));
	KASSERT(type != FMCD_HDL_NONE, ("fmcd_handle_alloc: NONE type"));

	for (i = 0; i < FMCD_MAX_HANDLES; i++) {
		if (sc->handles[i].ptr == NULL) {
			sc->handles[i].ptr = ptr;
			sc->handles[i].type = type;
			return (((uint64_t)sc->handle_gen <<
			    FMCD_HANDLE_SHIFT) | (uint64_t)i);
		}
	}
	printf("fmcd: handle table full (%d entries)\n", FMCD_MAX_HANDLES);
	return (0);
}

/*
 * Look up a handle ID and return the kernel pointer.
 * If expected_type is not FMCD_HDL_NONE, validates the type matches.
 * Returns NULL if the handle is invalid, stale, or type mismatches.
 * Caller must hold sc->port_lock.
 */
static t_Handle
fmcd_handle_lookup(struct fmcd_softc *sc, uint64_t id,
    enum fmcd_handle_type expected_type)
{
	uint32_t slot, gen;

	if (id == 0)
		return (NULL);

	slot = (uint32_t)(id & FMCD_HANDLE_MASK);
	gen = (uint32_t)(id >> FMCD_HANDLE_SHIFT);

	if (slot >= FMCD_MAX_HANDLES)
		return (NULL);
	if (gen != sc->handle_gen)
		return (NULL);
	if (sc->handles[slot].ptr == NULL)
		return (NULL);
	if (expected_type != FMCD_HDL_NONE &&
	    sc->handles[slot].type != expected_type)
		return (NULL);

	return (sc->handles[slot].ptr);
}

/*
 * Free a handle slot.  Returns the kernel pointer for the caller
 * to pass to NCSW delete, or NULL if the handle is invalid.
 * Caller must hold sc->port_lock.
 */
static t_Handle
fmcd_handle_free(struct fmcd_softc *sc, uint64_t id,
    enum fmcd_handle_type expected_type)
{
	uint32_t slot, gen;
	t_Handle ptr;

	if (id == 0)
		return (NULL);

	slot = (uint32_t)(id & FMCD_HANDLE_MASK);
	gen = (uint32_t)(id >> FMCD_HANDLE_SHIFT);

	if (slot >= FMCD_MAX_HANDLES)
		return (NULL);
	if (gen != sc->handle_gen)
		return (NULL);
	if (sc->handles[slot].ptr == NULL)
		return (NULL);
	if (expected_type != FMCD_HDL_NONE &&
	    sc->handles[slot].type != expected_type)
		return (NULL);

	ptr = sc->handles[slot].ptr;
	sc->handles[slot].ptr = NULL;
	sc->handles[slot].type = FMCD_HDL_NONE;
	return (ptr);
}

/*
 * Resolve a handle ID to a kernel pointer without freeing the slot.
 * Used by the CDX module to translate hash table handles received
 * from userspace (dpa_app) into NCSW kernel pointers.
 * Returns NULL if the handle is invalid or stale.
 */
t_Handle
fmcd_handle_resolve(struct fmcd_softc *sc, uint64_t id,
    enum fmcd_handle_type expected_type)
{
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_lookup(sc, id, expected_type);
	sx_xunlock(&sc->port_lock);
	return (ptr);
}

/*
 * Translate a void* field that holds an opaque handle ID to a kernel
 * pointer.  Returns 0 on success (field updated in-place), EINVAL on
 * bad handle.  NULL fields are left alone (returns 0).
 * Caller must hold sc->port_lock.
 */
static int
fmcd_xlate(struct fmcd_softc *sc, void **field,
    enum fmcd_handle_type expected_type)
{
	t_Handle ptr;

	if (*field == NULL)
		return (0);

	ptr = fmcd_handle_lookup(sc, (uint64_t)(uintptr_t)*field,
	    expected_type);
	if (ptr == NULL)
		return (EINVAL);

	*field = ptr;
	return (0);
}

/*
 * Translate handle IDs embedded within a cc_next_engine_params_t
 * to kernel pointers.  Returns 0 on success, EINVAL on bad handle.
 * Caller must hold sc->port_lock.
 */
static int
fmcd_translate_next_engine(struct fmcd_softc *sc,
    ioc_fm_pcd_cc_next_engine_params_t *ne)
{
	int error;

	switch (ne->next_engine) {
	case e_IOC_FM_PCD_HASH:
	case e_IOC_FM_PCD_CC:
		error = fmcd_xlate(sc,
		    &ne->params.cc_params.cc_node_id, FMCD_HDL_NONE);
		if (error)
			return (error);
		break;
	case e_IOC_FM_PCD_KG:
		error = fmcd_xlate(sc,
		    &ne->params.kg_params.p_direct_scheme,
		    FMCD_HDL_KG_SCHEME);
		if (error)
			return (error);
		break;
#if (DPAA_VERSION >= 11)
	case e_IOC_FM_PCD_FR:
		error = fmcd_xlate(sc,
		    &ne->params.fr_params.frm_replic_id,
		    FMCD_HDL_FRM_REPLIC);
		if (error)
			return (error);
		break;
#endif
	default:
		break;
	}

	/* manip_id can appear regardless of next_engine */
	error = fmcd_xlate(sc, &ne->manip_id, FMCD_HDL_MANIP);
	return (error);
}

/*
 * Translate handle IDs in a policer profile's color action params.
 * Caller must hold sc->port_lock.
 */
static int
fmcd_translate_plcr_color(struct fmcd_softc *sc,
    ioc_fm_pcd_engine next_engine,
    ioc_fm_pcd_plcr_next_engine_params_u *p)
{
	int error;

	switch (next_engine) {
	case e_IOC_FM_PCD_PLCR:
		error = fmcd_xlate(sc, &p->p_profile,
		    FMCD_HDL_PLCR_PROFILE);
		return (error);
	case e_IOC_FM_PCD_KG:
		error = fmcd_xlate(sc, &p->p_direct_scheme,
		    FMCD_HDL_KG_SCHEME);
		return (error);
	default:
		return (0);
	}
}

/* ------------------------------------------------------------------ */
/*  PCD ioctl handlers                                                 */
/* ------------------------------------------------------------------ */

static int
fmcd_ioc_pcd_enable(struct fmcd_softc *sc)
{
	t_Error err;

	err = FM_PCD_Enable(sc->pcd_handle);
	return (err == E_OK ? 0 : EIO);
}

static int
fmcd_ioc_pcd_disable(struct fmcd_softc *sc)
{
	t_Error err;

	err = FM_PCD_Disable(sc->pcd_handle);
	return (err == E_OK ? 0 : EIO);
}

static int
fmcd_ioc_get_api_version(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_api_version_t *p = (ioc_fm_api_version_t *)data;

	p->version.major = FMD_API_VERSION_MAJOR;
	p->version.minor = FMD_API_VERSION_MINOR;
	p->version.respin = FMD_API_VERSION_RESPIN;
	p->version.reserved = 0;
	return (0);
}

/*
 * NetEnvCharacteristicsSet — _IOWR, framework copies in/out.
 * Direct cast: ioc struct layout == NCSW struct layout.
 */
static int
fmcd_ioc_net_env_set(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_net_env_params_t *param =
	    (ioc_fm_pcd_net_env_params_t *)data;
	t_Handle ptr;
	uint64_t id;

	ptr = FM_PCD_NetEnvCharacteristicsSet(sc->pcd_handle,
	    (t_FmPcdNetEnvParams *)param);
	if (ptr == NULL)
		return (EIO);

	sx_xlock(&sc->port_lock);
	id = fmcd_handle_alloc(sc, ptr, FMCD_HDL_NET_ENV);
	sx_xunlock(&sc->port_lock);
	if (id == 0) {
		FM_PCD_NetEnvCharacteristicsDelete(ptr);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)id;
	return (0);
}

static int
fmcd_ioc_net_env_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_NET_ENV);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_NetEnvCharacteristicsDelete(ptr)
	    == E_OK ? 0 : EIO);
}

/*
 * KgSchemeSet — _IOWR, framework copies in/out.
 * Direct cast: ioc struct layout == NCSW struct layout
 * (after adding bool shared to t_FmPcdKgSchemeParams).
 */
static int
fmcd_ioc_kg_scheme_set(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_kg_scheme_params_t *param =
	    (ioc_fm_pcd_kg_scheme_params_t *)data;
	t_FmPcdKgSchemeParams *ncsw = (t_FmPcdKgSchemeParams *)param;
	t_Handle ptr;
	uint64_t id;
	int error;

	/* Translate input handle references */
	sx_xlock(&sc->port_lock);
	if (param->modify) {
		error = fmcd_xlate(sc, &param->scm_id.scheme_id,
		    FMCD_HDL_KG_SCHEME);
		if (error)
			goto unlock;
	}
	error = fmcd_xlate(sc, &param->net_env_params.net_env_id,
	    FMCD_HDL_NET_ENV);
	if (error)
		goto unlock;
	if (param->next_engine == e_IOC_FM_PCD_CC) {
		error = fmcd_xlate(sc,
		    &param->kg_next_engine_params.cc.tree_id,
		    FMCD_HDL_CC_ROOT);
		if (error)
			goto unlock;
	}
	/* PLCR profile in KG scheme context uses a numeric index
	 * (profile_select), not a handle — no translation needed. */
	sx_xunlock(&sc->port_lock);

	ptr = FM_PCD_KgSchemeSet(sc->pcd_handle, ncsw);
	if (ptr == NULL)
		return (EIO);

	sx_xlock(&sc->port_lock);
	id = fmcd_handle_alloc(sc, ptr, FMCD_HDL_KG_SCHEME);
	sx_xunlock(&sc->port_lock);
	if (id == 0) {
		FM_PCD_KgSchemeDelete(ptr);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)id;
	return (0);

unlock:
	sx_xunlock(&sc->port_lock);
	return (error);
}

static int
fmcd_ioc_kg_scheme_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_KG_SCHEME);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_KgSchemeDelete(ptr) == E_OK ? 0 : EIO);
}

/*
 * CcRootBuild — _IO on FreeBSD (struct exceeds IOCPARM_MAX).
 * Manual copyin/copyout, direct cast.
 */
static int
fmcd_ioc_cc_root_build(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_cc_tree_params_t *param;
	void *uptr;
	t_Handle ptr;
	uint64_t hid;
	int i, j, error;

	uptr = *(void **)data;
	if (uptr == NULL)
		return (EINVAL);

	param = malloc(sizeof(*param), M_FMCD, M_WAITOK);
	error = copyin(uptr, param, sizeof(*param));
	if (error) {
		free(param, M_FMCD);
		return (error);
	}

	/* Translate input handle references */
	sx_xlock(&sc->port_lock);
	error = fmcd_xlate(sc, &param->net_env_id, FMCD_HDL_NET_ENV);
	if (error)
		goto unlock;

	for (i = 0; i < param->num_of_groups &&
	    i < IOC_FM_PCD_MAX_NUM_OF_CC_GROUPS; i++) {
		uint8_t ndu = param->fm_pcd_cc_group_params[i].
		    num_of_distinction_units;
		int n_entries = (1 << ndu);	/* 2^ndu entries per group */
		for (j = 0; j < n_entries &&
		    j < IOC_FM_PCD_MAX_NUM_OF_CC_ENTRIES_IN_GRP; j++) {
			ioc_fm_pcd_cc_next_engine_params_t *ne =
			    &param->fm_pcd_cc_group_params[i].
			    next_engine_per_entries_in_grp[j];
			error = fmcd_translate_next_engine(sc, ne);
			if (error)
				goto unlock;
		}
	}
	sx_xunlock(&sc->port_lock);

	ptr = FM_PCD_CcRootBuild(sc->pcd_handle,
	    (t_FmPcdCcTreeParams *)param);
	if (ptr == NULL) {
		free(param, M_FMCD);
		return (EIO);
	}

	sx_xlock(&sc->port_lock);
	hid = fmcd_handle_alloc(sc, ptr, FMCD_HDL_CC_ROOT);
	sx_xunlock(&sc->port_lock);
	if (hid == 0) {
		FM_PCD_CcRootDelete(ptr);
		free(param, M_FMCD);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)hid;
	error = copyout(param, uptr, sizeof(*param));
	free(param, M_FMCD);
	return (error);

unlock:
	sx_xunlock(&sc->port_lock);
	free(param, M_FMCD);
	return (error);
}

static int
fmcd_ioc_cc_root_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_CC_ROOT);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_CcRootDelete(ptr) == E_OK ? 0 : EIO);
}

/*
 * HashTableSet — _IOWR, framework copies in/out.
 * Direct cast.
 */
static int
fmcd_ioc_hash_table_set(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_hash_table_params_t *param =
	    (ioc_fm_pcd_hash_table_params_t *)data;
	t_Handle ptr;
	uint64_t id;
	int error;

	/* Translate miss next-engine handle references */
	sx_xlock(&sc->port_lock);
	error = fmcd_translate_next_engine(sc,
	    &param->cc_next_engine_params_for_miss);
	sx_xunlock(&sc->port_lock);
	if (error)
		return (error);

	ptr = FM_PCD_HashTableSet(sc->pcd_handle,
	    (t_FmPcdHashTableParams *)param);
	if (ptr == NULL)
		return (EIO);

	sx_xlock(&sc->port_lock);
	id = fmcd_handle_alloc(sc, ptr, FMCD_HDL_HASH_TABLE);
	sx_xunlock(&sc->port_lock);
	if (id == 0) {
		FM_PCD_HashTableDelete(ptr);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)id;
	return (0);
}

static int
fmcd_ioc_hash_table_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_HASH_TABLE);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_HashTableDelete(ptr) == E_OK ? 0 : EIO);
}

/*
 * MatchTableSet — _IO on FreeBSD (struct exceeds IOCPARM_MAX).
 * Manual copyin/copyout, direct cast.
 *
 * Special handling: key_params[i].p_key and p_mask point to userspace
 * memory after copyin of the outer struct.  Must copyin each key/mask
 * buffer and redirect the pointers before calling NCSW.
 */
static int
fmcd_ioc_match_table_set(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_cc_node_params_t *param;
	void *uptr;
	uint8_t key_size;
	int i, error;
	int processed_keys = 0;

	uptr = *(void **)data;
	if (uptr == NULL)
		return (EINVAL);

	param = malloc(sizeof(*param), M_FMCD, M_WAITOK);
	error = copyin(uptr, param, sizeof(*param));
	if (error) {
		free(param, M_FMCD);
		return (error);
	}

	key_size = param->keys_params.key_size;

	/*
	 * Copyin key and mask buffers from userspace.
	 * After copyin of the outer struct, p_key/p_mask still reference
	 * userspace addresses.  Allocate kernel buffers, copyin, redirect.
	 *
	 * Track processed_keys so the cleanup path only frees keys whose
	 * pointers were replaced with kernel allocations.  Keys beyond
	 * processed_keys still hold raw userspace addresses from the
	 * outer copyin and must NOT be freed.
	 */
	for (i = 0; i < param->keys_params.num_of_keys &&
	    i < IOC_FM_PCD_MAX_NUM_OF_KEYS; i++) {
		ioc_fm_pcd_cc_key_params_t *kp =
		    &param->keys_params.key_params[i];
		void *umask = kp->p_mask;

		/*
		 * Clear p_mask before processing p_key so that if
		 * p_key copyin fails, the cleanup path won't call
		 * free() on the userspace address still in p_mask.
		 */
		kp->p_mask = NULL;

		if (kp->p_key != NULL && key_size > 0) {
			void *usrc = kp->p_key;
			kp->p_key = malloc(key_size, M_FMCD, M_WAITOK);
			processed_keys = i + 1;
			error = copyin(usrc, kp->p_key, key_size);
			if (error)
				goto fail;
		}

		if (umask != NULL && key_size > 0) {
			kp->p_mask = malloc(key_size, M_FMCD, M_WAITOK);
			processed_keys = i + 1;
			error = copyin(umask, kp->p_mask, key_size);
			if (error)
				goto fail;
		}

		processed_keys = i + 1;
	}

	/* Translate handle references in next-engine params */
	sx_xlock(&sc->port_lock);
	for (i = 0; i < param->keys_params.num_of_keys &&
	    i < IOC_FM_PCD_MAX_NUM_OF_KEYS; i++) {
		error = fmcd_translate_next_engine(sc,
		    &param->keys_params.key_params[i].cc_next_engine_params);
		if (error) {
			sx_xunlock(&sc->port_lock);
			goto fail;
		}
	}
	error = fmcd_translate_next_engine(sc,
	    &param->keys_params.cc_next_engine_params_for_miss);
	sx_xunlock(&sc->port_lock);
	if (error)
		goto fail;

	{
		t_Handle ptr;
		uint64_t hid;

		ptr = FM_PCD_MatchTableSet(sc->pcd_handle,
		    (t_FmPcdCcNodeParams *)param);
		if (ptr == NULL) {
			error = EIO;
			goto fail;
		}

		sx_xlock(&sc->port_lock);
		hid = fmcd_handle_alloc(sc, ptr, FMCD_HDL_MATCH_TABLE);
		sx_xunlock(&sc->port_lock);
		if (hid == 0) {
			FM_PCD_MatchTableDelete(ptr);
			error = ENOMEM;
			goto fail;
		}

		param->id = (void *)(uintptr_t)hid;
	}

	error = copyout(param, uptr, sizeof(*param));

fail:
	/*
	 * Free only key/mask buffers that were replaced with kernel
	 * allocations (indices 0..processed_keys-1).  Entries beyond
	 * processed_keys still hold userspace addresses and must not
	 * be passed to free().
	 */
	for (i = 0; i < processed_keys; i++) {
		ioc_fm_pcd_cc_key_params_t *kp =
		    &param->keys_params.key_params[i];
		if (kp->p_key != NULL)
			free(kp->p_key, M_FMCD);
		if (kp->p_mask != NULL)
			free(kp->p_mask, M_FMCD);
	}
	free(param, M_FMCD);
	return (error);
}

static int
fmcd_ioc_match_table_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_MATCH_TABLE);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_MatchTableDelete(ptr) == E_OK ? 0 : EIO);
}

/*
 * ManipNodeSet — _IOWR, framework copies in/out.
 * Direct cast.
 */
static int
fmcd_ioc_manip_node_set(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_manip_params_t *param =
	    (ioc_fm_pcd_manip_params_t *)data;
	t_Handle ptr;
	uint64_t id;
	int error;

	/* Translate chained manipulation handle */
	sx_xlock(&sc->port_lock);
	error = fmcd_xlate(sc, &param->p_next_manip, FMCD_HDL_MANIP);
	sx_xunlock(&sc->port_lock);
	if (error)
		return (error);

	ptr = FM_PCD_ManipNodeSet(sc->pcd_handle,
	    (t_FmPcdManipParams *)param);
	if (ptr == NULL)
		return (EIO);

	sx_xlock(&sc->port_lock);
	id = fmcd_handle_alloc(sc, ptr, FMCD_HDL_MANIP);
	sx_xunlock(&sc->port_lock);
	if (id == 0) {
		FM_PCD_ManipNodeDelete(ptr);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)id;
	return (0);
}

static int
fmcd_ioc_manip_node_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_MANIP);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_ManipNodeDelete(ptr) == E_OK ? 0 : EIO);
}

#if (DPAA_VERSION >= 11)
/*
 * FrmReplicSetGroup — _IO on FreeBSD (struct exceeds IOCPARM_MAX).
 * Manual copyin/copyout, direct cast.
 */
static int
fmcd_ioc_frm_replic_set_group(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_frm_replic_group_params_t *param;
	void *uptr;
	t_Handle ptr;
	uint64_t hid;
	int i, error;

	uptr = *(void **)data;
	if (uptr == NULL)
		return (EINVAL);

	param = malloc(sizeof(*param), M_FMCD, M_WAITOK);
	error = copyin(uptr, param, sizeof(*param));
	if (error) {
		free(param, M_FMCD);
		return (error);
	}

	/* Translate handle references in each entry's next-engine params */
	sx_xlock(&sc->port_lock);
	for (i = 0; i < param->num_of_entries &&
	    i < IOC_FM_PCD_FRM_REPLIC_MAX_NUM_OF_ENTRIES; i++) {
		error = fmcd_translate_next_engine(sc,
		    &param->next_engine_params[i]);
		if (error) {
			sx_xunlock(&sc->port_lock);
			free(param, M_FMCD);
			return (error);
		}
	}
	sx_xunlock(&sc->port_lock);

	ptr = FM_PCD_FrmReplicSetGroup(sc->pcd_handle,
	    (t_FmPcdFrmReplicGroupParams *)param);
	if (ptr == NULL) {
		free(param, M_FMCD);
		return (EIO);
	}

	sx_xlock(&sc->port_lock);
	hid = fmcd_handle_alloc(sc, ptr, FMCD_HDL_FRM_REPLIC);
	sx_xunlock(&sc->port_lock);
	if (hid == 0) {
		FM_PCD_FrmReplicDeleteGroup(ptr);
		free(param, M_FMCD);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)hid;
	error = copyout(param, uptr, sizeof(*param));
	free(param, M_FMCD);
	return (error);
}

static int
fmcd_ioc_frm_replic_delete_group(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_FRM_REPLIC);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_FrmReplicDeleteGroup(ptr) == E_OK ? 0 : EIO);
}
#endif /* DPAA_VERSION >= 11 */

/*
 * PrsLoadSw — _IOW, framework copies in.
 * Direct cast, but p_code pointer needs copyin + redirect.
 */
static int
fmcd_ioc_prs_load_sw(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_prs_sw_params_t *param =
	    (ioc_fm_pcd_prs_sw_params_t *)data;
	uint8_t *code_buf;
	t_Error err;

	if (param->size == 0 || param->p_code == NULL)
		return (EINVAL);
	if (param->size > FM_PCD_SW_PRS_SIZE)
		return (EINVAL);

	/* Copyin the parser microcode from userspace */
	code_buf = malloc(param->size, M_FMCD, M_WAITOK);
	if (copyin(param->p_code, code_buf, param->size) != 0) {
		free(code_buf, M_FMCD);
		return (EFAULT);
	}

	/* Redirect pointer to kernel buffer */
	param->p_code = code_buf;

	err = FM_PCD_PrsLoadSw(sc->pcd_handle, (t_FmPcdPrsSwParams *)param);
	free(code_buf, M_FMCD);

	return (err == E_OK ? 0 : EIO);
}

/*
 * PlcrProfileSet — _IOWR, framework copies in/out.
 * Direct cast.
 */
static int
fmcd_ioc_plcr_profile_set(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_pcd_plcr_profile_params_t *param =
	    (ioc_fm_pcd_plcr_profile_params_t *)data;
	t_Handle ptr;
	uint64_t id;
	int error;

	/* Translate input handle references */
	sx_xlock(&sc->port_lock);
	if (param->modify) {
		error = fmcd_xlate(sc, &param->profile_select.p_profile,
		    FMCD_HDL_PLCR_PROFILE);
		if (error)
			goto unlock;
	}
	error = fmcd_translate_plcr_color(sc, param->next_engine_on_green,
	    &param->params_on_green);
	if (error)
		goto unlock;
	error = fmcd_translate_plcr_color(sc, param->next_engine_on_yellow,
	    &param->params_on_yellow);
	if (error)
		goto unlock;
	error = fmcd_translate_plcr_color(sc, param->next_engine_on_red,
	    &param->params_on_red);
	if (error)
		goto unlock;
	sx_xunlock(&sc->port_lock);

	ptr = FM_PCD_PlcrProfileSet(sc->pcd_handle,
	    (t_FmPcdPlcrProfileParams *)param);
	if (ptr == NULL)
		return (EIO);

	sx_xlock(&sc->port_lock);
	id = fmcd_handle_alloc(sc, ptr, FMCD_HDL_PLCR_PROFILE);
	sx_xunlock(&sc->port_lock);
	if (id == 0) {
		FM_PCD_PlcrProfileDelete(ptr);
		return (ENOMEM);
	}

	param->id = (void *)(uintptr_t)id;
	return (0);

unlock:
	sx_xunlock(&sc->port_lock);
	return (error);
}

static int
fmcd_ioc_plcr_profile_delete(struct fmcd_softc *sc, caddr_t data)
{
	ioc_fm_obj_t *ioc = (ioc_fm_obj_t *)data;
	uint64_t id = (uint64_t)(uintptr_t)ioc->obj;
	t_Handle ptr;

	sx_xlock(&sc->port_lock);
	ptr = fmcd_handle_free(sc, id, FMCD_HDL_PLCR_PROFILE);
	sx_xunlock(&sc->port_lock);

	if (ptr == NULL)
		return (EINVAL);

	return (FM_PCD_PlcrProfileDelete(ptr) == E_OK ? 0 : EIO);
}

/* ------------------------------------------------------------------ */
/*  Port ioctl handlers                                                */
/* ------------------------------------------------------------------ */

/*
 * PortSetPCD — _IOW, framework copies in the outer struct.
 * The pointer members (p_prs_params, p_cc_params, p_kg_params,
 * p_plcr_params) still reference userspace sub-structs after copyin.
 * Copyin each and redirect, then direct-cast to NCSW type.
 *
 * Handle references (net_env_id, cc_tree_id, scheme_ids[], etc.)
 * are translated from opaque IDs to kernel pointers.
 */
static int
fmcd_ioc_port_set_pcd(struct fmcd_softc *sc, t_Handle h_port, caddr_t data)
{
	ioc_fm_port_pcd_params_t *param = (ioc_fm_port_pcd_params_t *)data;
	ioc_fm_port_pcd_prs_params_t *prs = NULL;
	ioc_fm_port_pcd_cc_params_t *cc_p = NULL;
	ioc_fm_port_pcd_kg_params_t *kg_p = NULL;
	ioc_fm_port_pcd_plcr_params_t *plcr = NULL;
	t_Error err;
	int i, error = 0;

	if (h_port == NULL)
		return (ENXIO);

	/* Copyin sub-structs from userspace pointers */
	if (param->p_prs_params != NULL) {
		prs = malloc(sizeof(*prs), M_FMCD, M_WAITOK);
		error = copyin(param->p_prs_params, prs, sizeof(*prs));
		if (error)
			goto out;
		param->p_prs_params = prs;
	}

	if (param->p_cc_params != NULL) {
		cc_p = malloc(sizeof(*cc_p), M_FMCD, M_WAITOK);
		error = copyin(param->p_cc_params, cc_p, sizeof(*cc_p));
		if (error)
			goto out;
		param->p_cc_params = cc_p;
	}

	if (param->p_kg_params != NULL) {
		kg_p = malloc(sizeof(*kg_p), M_FMCD, M_WAITOK);
		error = copyin(param->p_kg_params, kg_p, sizeof(*kg_p));
		if (error)
			goto out;
		param->p_kg_params = kg_p;
	}

	if (param->p_plcr_params != NULL) {
		plcr = malloc(sizeof(*plcr), M_FMCD, M_WAITOK);
		error = copyin(param->p_plcr_params, plcr, sizeof(*plcr));
		if (error)
			goto out;
		param->p_plcr_params = plcr;
	}

	/* Translate handle references in copyin'd sub-structs */
	sx_xlock(&sc->port_lock);
	error = fmcd_xlate(sc, &param->net_env_id, FMCD_HDL_NET_ENV);
	if (error)
		goto unlock;

	if (cc_p != NULL) {
		error = fmcd_xlate(sc, &cc_p->cc_tree_id, FMCD_HDL_CC_ROOT);
		if (error)
			goto unlock;
	}

	if (kg_p != NULL) {
		for (i = 0; i < kg_p->num_of_schemes &&
		    i < FM_PCD_KG_NUM_OF_SCHEMES; i++) {
			error = fmcd_xlate(sc, &kg_p->scheme_ids[i],
			    FMCD_HDL_KG_SCHEME);
			if (error)
				goto unlock;
		}
		if (kg_p->direct_scheme) {
			error = fmcd_xlate(sc, &kg_p->direct_scheme_id,
			    FMCD_HDL_KG_SCHEME);
			if (error)
				goto unlock;
		}
	}

	if (plcr != NULL) {
		error = fmcd_xlate(sc, &plcr->plcr_profile_id,
		    FMCD_HDL_PLCR_PROFILE);
		if (error)
			goto unlock;
	}

	error = fmcd_xlate(sc, &param->p_ip_reassembly_manip,
	    FMCD_HDL_MANIP);
	if (error)
		goto unlock;
	error = fmcd_xlate(sc, &param->p_capwap_reassembly_manip,
	    FMCD_HDL_MANIP);
	if (error)
		goto unlock;
	sx_xunlock(&sc->port_lock);

	err = FM_PORT_SetPCD(h_port, (t_FmPortPcdParams *)param);
	if (err != E_OK) {
		printf("fmcd: SetPCD FAILED err=%d\n", (int)err);
		error = EIO;
		goto out;
	}

out:
	free(prs, M_FMCD);
	free(cc_p, M_FMCD);
	free(kg_p, M_FMCD);
	free(plcr, M_FMCD);
	return (error);

unlock:
	sx_xunlock(&sc->port_lock);
	goto out;
}

static int
fmcd_ioc_port_delete_pcd(struct fmcd_softc *sc, t_Handle h_port)
{

	if (h_port == NULL)
		return (ENXIO);

	{
		t_Error err = FM_PORT_DeletePCD(h_port);
		return (err == E_OK ? 0 : EIO);
	}
}

static int
fmcd_ioc_port_enable(struct fmcd_softc *sc, t_Handle h_port)
{
	t_Error err;

	if (h_port == NULL) {
		printf("fmcd: port_enable: h_port=NULL!\n");
		return (ENXIO);
	}

	err = FM_PORT_Enable(h_port);
	return (err == E_OK ? 0 : EIO);
}

static int
fmcd_ioc_port_disable(struct fmcd_softc *sc, t_Handle h_port)
{
	t_Error err;

	if (h_port == NULL) {
		printf("fmcd: port_disable: h_port=NULL!\n");
		return (ENXIO);
	}

	err = FM_PORT_Disable(h_port);
	return (err == E_OK ? 0 : EIO);
}

/* ------------------------------------------------------------------ */
/*  Chardev operations                                                 */
/* ------------------------------------------------------------------ */

static int
fmcd_open(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	return (0);
}

/*
 * PCD device open handler.
 *
 * When userspace opens /dev/fm0-pcd, it is taking over PCD management
 * (e.g., FMC/dpa_app is starting).  Tear down any existing PCD set up
 * by the dtsec driver (RSS KG schemes) to free KG scheme IDs for FMC.
 * Also reset the handle table and bump the generation counter to
 * invalidate any stale handles from a previous session.
 */
static int
fmcd_pcd_open(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	struct fmcd_softc *sc = dev->si_drv1;
	int i;

	sx_xlock(&sc->port_lock);

	for (i = 0; i < 8; i++) {
		if (sc->rx_port_schemes[i] != NULL) {
			if (sc->rx_port_handles[i] != NULL)
				FM_PORT_DeletePCD(sc->rx_port_handles[i]);
			FM_PCD_KgSchemeDelete(sc->rx_port_schemes[i]);
			sc->rx_port_schemes[i] = NULL;
			/* Clear dtsec's handle so pcd_free() won't
			 * double-free */
			if (sc->rx_port_scheme_ptrs[i] != NULL) {
				*sc->rx_port_scheme_ptrs[i] = NULL;
				sc->rx_port_scheme_ptrs[i] = NULL;
			}
		}
	}

	/*
	 * Clear handle table and bump generation.  We don't delete
	 * NCSW objects here — the previous session should have cleaned
	 * up via close, and the PCD enable/disable cycle that
	 * fmc/dpa_app does on startup resets PCD state.
	 */
	for (i = 0; i < FMCD_MAX_HANDLES; i++) {
		sc->handles[i].ptr = NULL;
		sc->handles[i].type = FMCD_HDL_NONE;
	}
	sc->handle_gen++;
	if (sc->handle_gen == 0)
		sc->handle_gen = 1;

	sx_xunlock(&sc->port_lock);

	return (0);
}

/*
 * PCD device close handler.
 *
 * Delete all PCD objects created during this session in dependency
 * order (reverse of creation).  This prevents kernel memory leaks
 * if userspace crashes without explicit cleanup.
 */
static int
fmcd_pcd_close(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	struct fmcd_softc *sc = dev->si_drv1;
	int i;

	sx_xlock(&sc->port_lock);

	if (sc->pcd_transferred) {
		/*
		 * CDX has taken ownership of all PCD resources via
		 * CDX_CTRL_DPA_SET_PARAMS.  The NCSW objects (hash tables,
		 * CC trees, KG schemes, net envs) are bound to FMan ports
		 * and must not be deleted.  Just clear the handle table.
		 */
		for (i = 0; i < FMCD_MAX_HANDLES; i++) {
			sc->handles[i].ptr = NULL;
			sc->handles[i].type = FMCD_HDL_NONE;
		}
	} else {
		/*
		 * No CDX takeover — dpa_app exited before CDX claimed
		 * the PCD resources.  Delete all NCSW objects in
		 * dependency order to prevent kernel memory leaks.
		 */
		static const enum fmcd_handle_type teardown_order[] = {
			FMCD_HDL_FRM_REPLIC,
			FMCD_HDL_PLCR_PROFILE,
			FMCD_HDL_MATCH_TABLE,
			FMCD_HDL_HASH_TABLE,
			FMCD_HDL_CC_ROOT,
			FMCD_HDL_KG_SCHEME,
			FMCD_HDL_MANIP,
			FMCD_HDL_NET_ENV,
		};
		int phase;

		for (phase = 0; phase < nitems(teardown_order); phase++) {
			for (i = 0; i < FMCD_MAX_HANDLES; i++) {
				t_Handle ptr = sc->handles[i].ptr;
				t_Error err;

				if (ptr == NULL ||
				    sc->handles[i].type !=
				    teardown_order[phase])
					continue;

				sc->handles[i].ptr = NULL;
				sc->handles[i].type = FMCD_HDL_NONE;

				switch (teardown_order[phase]) {
				case FMCD_HDL_NET_ENV:
					err = FM_PCD_NetEnvCharacteristicsDelete(ptr);
					break;
				case FMCD_HDL_KG_SCHEME:
					err = FM_PCD_KgSchemeDelete(ptr);
					break;
				case FMCD_HDL_CC_ROOT:
					err = FM_PCD_CcRootDelete(ptr);
					break;
				case FMCD_HDL_MATCH_TABLE:
					err = FM_PCD_MatchTableDelete(ptr);
					break;
				case FMCD_HDL_HASH_TABLE:
					err = FM_PCD_HashTableDelete(ptr);
					break;
				case FMCD_HDL_MANIP:
					err = FM_PCD_ManipNodeDelete(ptr);
					break;
				case FMCD_HDL_PLCR_PROFILE:
					err = FM_PCD_PlcrProfileDelete(ptr);
					break;
				case FMCD_HDL_FRM_REPLIC:
					err = FM_PCD_FrmReplicDeleteGroup(ptr);
					break;
				default:
					err = E_OK;
					break;
				}

				if (err != E_OK)
					printf("fmcd: close: failed to "
					    "delete slot %d type %d\n",
					    i, teardown_order[phase]);
			}
		}
	}

	/* Bump generation to invalidate any lingering handle IDs */
	sc->handle_gen++;
	if (sc->handle_gen == 0)
		sc->handle_gen = 1;

	sc->pcd_transferred = false;
	sx_xunlock(&sc->port_lock);
	return (0);
}

static int
fmcd_close(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	return (0);
}

/*
 * PCD device ioctl handler (/dev/fm0-pcd).
 */
static int
fmcd_pcd_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct fmcd_softc *sc = dev->si_drv1;
	int error;

	/* PCD operations require root */
	error = priv_check(td, PRIV_DRIVER);
	if (error)
		return (error);

	if (sc->pcd_handle == NULL)
		return (ENXIO);

	switch (cmd) {
	case FM_PCD_IOC_ENABLE:
		return (fmcd_ioc_pcd_enable(sc));
	case FM_PCD_IOC_DISABLE:
		return (fmcd_ioc_pcd_disable(sc));
	case FM_PCD_IOC_NET_ENV_CHARACTERISTICS_SET:
		return (fmcd_ioc_net_env_set(sc, data));
	case FM_PCD_IOC_NET_ENV_CHARACTERISTICS_DELETE:
		return (fmcd_ioc_net_env_delete(sc, data));
	case FM_PCD_IOC_KG_SCHEME_SET:
		return (fmcd_ioc_kg_scheme_set(sc, data));
	case FM_PCD_IOC_KG_SCHEME_DELETE:
		return (fmcd_ioc_kg_scheme_delete(sc, data));
	case FM_PCD_IOC_CC_ROOT_BUILD:
		return (fmcd_ioc_cc_root_build(sc, data));
	case FM_PCD_IOC_CC_ROOT_DELETE:
		return (fmcd_ioc_cc_root_delete(sc, data));
	case FM_PCD_IOC_HASH_TABLE_SET:
		return (fmcd_ioc_hash_table_set(sc, data));
	case FM_PCD_IOC_HASH_TABLE_DELETE:
		return (fmcd_ioc_hash_table_delete(sc, data));
	case FM_PCD_IOC_MATCH_TABLE_SET:
		return (fmcd_ioc_match_table_set(sc, data));
	case FM_PCD_IOC_MATCH_TABLE_DELETE:
		return (fmcd_ioc_match_table_delete(sc, data));
	case FM_PCD_IOC_MANIP_NODE_SET:
		return (fmcd_ioc_manip_node_set(sc, data));
	case FM_PCD_IOC_MANIP_NODE_DELETE:
		return (fmcd_ioc_manip_node_delete(sc, data));
#if (DPAA_VERSION >= 11)
	case FM_PCD_IOC_FRM_REPLIC_GROUP_SET:
		return (fmcd_ioc_frm_replic_set_group(sc, data));
	case FM_PCD_IOC_FRM_REPLIC_GROUP_DELETE:
		return (fmcd_ioc_frm_replic_delete_group(sc, data));
#endif
	case FM_PCD_IOC_PRS_LOAD_SW:
		return (fmcd_ioc_prs_load_sw(sc, data));
	case FM_PCD_IOC_PLCR_PROFILE_SET:
		return (fmcd_ioc_plcr_profile_set(sc, data));
	case FM_PCD_IOC_PLCR_PROFILE_DELETE:
		return (fmcd_ioc_plcr_profile_delete(sc, data));
	case FM_PCD_IOC_SET_ADVANCED_OFFLOAD_SUPPORT:
		return (FM_PCD_SetAdvancedOffloadSupport(sc->pcd_handle)
		    == E_OK ? 0 : EIO);
	case FM_PCD_IOC_ALLOW_HC_USAGE:
		/* HC is already configured by fman.c — just accept */
		return (0);
	default:
		return (ENOTTY);
	}
}

/*
 * FM device ioctl handler (/dev/fm0).
 */
static int
fmcd_fm_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct fmcd_softc *sc = dev->si_drv1;

	switch (cmd) {
	case FM_IOC_GET_API_VERSION:
		return (fmcd_ioc_get_api_version(sc, data));
	default:
		return (ENOTTY);
	}
}

/*
 * Port device ioctl handler (/dev/fm0-port-rx*, /dev/fm0-port-oh*).
 */
static int
fmcd_port_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct fmcd_softc *sc = dev->si_drv1;
	t_Handle h_port = dev->si_drv2;
	int error;

	error = priv_check(td, PRIV_DRIVER);
	if (error)
		return (error);

	switch (cmd) {
	case FM_PORT_IOC_SET_PCD:
		return (fmcd_ioc_port_set_pcd(sc, h_port, data));
	case FM_PORT_IOC_DELETE_PCD:
		return (fmcd_ioc_port_delete_pcd(sc, h_port));
	case FM_PORT_IOC_ENABLE:
		return (fmcd_ioc_port_enable(sc, h_port));
	case FM_PORT_IOC_DISABLE:
		return (fmcd_ioc_port_disable(sc, h_port));
	default:
		return (ENOTTY);
	}
}

static struct cdevsw fmcd_fm_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	fmcd_open,
	.d_close =	fmcd_close,
	.d_ioctl =	fmcd_fm_ioctl,
	.d_name =	"fm",
};

static struct cdevsw fmcd_pcd_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	fmcd_pcd_open,
	.d_close =	fmcd_pcd_close,
	.d_ioctl =	fmcd_pcd_ioctl,
	.d_name =	"fm-pcd",
};

static struct cdevsw fmcd_port_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	fmcd_open,
	.d_close =	fmcd_close,
	.d_ioctl =	fmcd_port_ioctl,
	.d_name =	"fm-port",
};

/* ------------------------------------------------------------------ */
/*  Init / Destroy                                                     */
/* ------------------------------------------------------------------ */

int
fmcd_init(device_t fman_dev, struct fmcd_softc *sc)
{
	struct fman_softc *fsc;
	int i;

	fsc = device_get_softc(fman_dev);

	memset(sc, 0, sizeof(*sc));
	sc->fman_dev = fman_dev;
	sc->fm_handle = fsc->fm_handle;
	sc->pcd_handle = fsc->pcd_handle;
	sc->handle_gen = 1;	/* start at 1 so slot 0's first ID != 0 */

	sx_init(&sc->port_lock, "fmcd_port");

	/* /dev/fman0 */
	sc->cdev_fm = make_dev(&fmcd_fm_cdevsw, FMD_MINOR_FM,
	    UID_ROOT, GID_WHEEL, 0600, "fm%d", device_get_unit(fman_dev));
	if (sc->cdev_fm == NULL) {
		device_printf(fman_dev, "fmcd: couldn't create /dev/fm\n");
		goto fail;
	}
	sc->cdev_fm->si_drv1 = sc;

	/* /dev/fman0-pcd */
	sc->cdev_pcd = make_dev(&fmcd_pcd_cdevsw, FMD_MINOR_PCD,
	    UID_ROOT, GID_WHEEL, 0600, "fm%d-pcd",
	    device_get_unit(fman_dev));
	if (sc->cdev_pcd == NULL) {
		device_printf(fman_dev,
		    "fmcd: couldn't create /dev/fm-pcd\n");
		goto fail;
	}
	sc->cdev_pcd->si_drv1 = sc;

	/*
	 * Port devices are created eagerly — the handles are filled in
	 * when dtsec/oh drivers attach via fmcd_register_rx_port/oh_port.
	 */
	for (i = 0; i < 8; i++) {
		sc->cdev_port_rx[i] = make_dev(&fmcd_port_cdevsw,
		    FMD_MINOR_RX_BASE + i, UID_ROOT, GID_WHEEL, 0600,
		    "fm%d-port-rx%d", device_get_unit(fman_dev), i);
		if (sc->cdev_port_rx[i]) {
			sc->cdev_port_rx[i]->si_drv1 = sc;
			sc->cdev_port_rx[i]->si_drv2 = NULL;
		}
	}

	for (i = 0; i < 6; i++) {
		sc->cdev_port_oh[i] = make_dev(&fmcd_port_cdevsw,
		    FMD_MINOR_OH_BASE + i, UID_ROOT, GID_WHEEL, 0600,
		    "fm%d-port-oh%d", device_get_unit(fman_dev), i);
		if (sc->cdev_port_oh[i]) {
			sc->cdev_port_oh[i]->si_drv1 = sc;
			sc->cdev_port_oh[i]->si_drv2 = NULL;
		}
	}

	device_printf(fman_dev, "fmcd: chardev interface ready\n");
	return (0);

fail:
	fmcd_destroy(sc);
	return (ENXIO);
}

void
fmcd_destroy(struct fmcd_softc *sc)
{
	int i;

	for (i = 0; i < 6; i++) {
		if (sc->cdev_port_oh[i])
			destroy_dev(sc->cdev_port_oh[i]);
	}
	for (i = 0; i < 8; i++) {
		if (sc->cdev_port_rx[i])
			destroy_dev(sc->cdev_port_rx[i]);
	}
	if (sc->cdev_pcd)
		destroy_dev(sc->cdev_pcd);
	if (sc->cdev_fm)
		destroy_dev(sc->cdev_fm);

	sx_destroy(&sc->port_lock);
}
