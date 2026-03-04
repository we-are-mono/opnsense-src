/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Mono Technologies Inc.
 *
 * QMan CEETM (Channel Egress Enhancement and Traffic Management) driver
 * for FreeBSD.  Ported from the Linux NXP QBMan SDK driver.
 *
 * CEETM provides hierarchical egress QoS:
 *   Sub-portals → LNIs → Channels → Class Queues
 * with per-level token-bucket shapers and WBFQ scheduling.
 *
 * Original Linux source:
 *   drivers/staging/fsl_qbman/qman_high.c (API implementation)
 *   include/linux/fsl_qman.h (types, MC structures)
 *   drivers/staging/fsl_qbman/qman_driver.c (DT init)
 *   drivers/staging/fsl_qbman/qman_config.c (CCSR registers)
 */

#ifndef _QMAN_CEETM_H
#define _QMAN_CEETM_H

#include <sys/types.h>
#include <sys/queue.h>

/*
 * Forward declarations — NCSW types used by qman_ceetm.c but not
 * needed by external callers.
 */
struct qm_portal;

/* ================================================================
 * CEETM Management Command Verb Codes (QMan RM section 1.5.9.7)
 *
 * These are sent via the same portal MC interface as FQ/CGR commands
 * (qm_mc_start / qm_mc_commit / qm_mc_result).
 * ================================================================ */

#define	QM_CEETM_VERB_LFQMT_CONFIG			0x70
#define	QM_CEETM_VERB_LFQMT_QUERY			0x71
#define	QM_CEETM_VERB_CQ_CONFIG				0x72
#define	QM_CEETM_VERB_CQ_QUERY				0x73
#define	QM_CEETM_VERB_DCT_CONFIG			0x74
#define	QM_CEETM_VERB_DCT_QUERY			0x75
#define	QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG		0x76
#define	QM_CEETM_VERB_CLASS_SCHEDULER_QUERY		0x77
#define	QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG	0x78
#define	QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY		0x79
#define	QM_CEETM_VERB_CCGR_CONFIG			0x7A
#define	QM_CEETM_VERB_CCGR_QUERY			0x7B
#define	QM_CEETM_VERB_CQ_PEEK_POP_XFDRREAD		0x7C
#define	QM_CEETM_VERB_STATISTICS_QUERY_WRITE		0x7D

/* MAPPING_SHAPER_TCFC sub-command types (upper 4 bits of cid field) */
#define	CEETM_COMMAND_CHANNEL_MAPPING	(0 << 12)
#define	CEETM_COMMAND_SP_MAPPING	(1 << 12)
#define	CEETM_COMMAND_CHANNEL_SHAPER	(2 << 12)
#define	CEETM_COMMAND_LNI_SHAPER	(3 << 12)
#define	CEETM_COMMAND_TCFC		(4 << 12)

/* CCGR sub-command types (upper 2 bits of ccgrid field) */
#define	CEETM_CCGRID_MASK		0x01FF
#define	CEETM_CCGR_CM_CONFIGURE		(0 << 14)
#define	CEETM_CCGR_DN_CONFIGURE	(1 << 14)
#define	CEETM_CCGR_TEST_WRITE		(2 << 14)
#define	CEETM_CCGR_CM_QUERY		(0 << 14)
#define	CEETM_CCGR_DN_QUERY		(1 << 14)
#define	CEETM_CCGR_DN_QUERY_FLUSH	(2 << 14)
#define	CEETM_QUERY_CONGESTION_STATE	(3 << 14)

/* Statistics query/write command types */
#define	CEETM_QUERY_DEQUEUE_STATISTICS		0x00
#define	CEETM_QUERY_DEQUEUE_CLEAR_STATISTICS	0x01
#define	CEETM_WRITE_DEQUEUE_STATISTICS		0x02
#define	CEETM_QUERY_REJECT_STATISTICS		0x03
#define	CEETM_QUERY_REJECT_CLEAR_STATISTICS	0x04
#define	CEETM_WRITE_REJECT_STATISTICS		0x05

/* MC result codes */
#define	QM_MCR_RESULT_OK		0xf0
#define	QM_MCR_RESULT_ERR_BADCOMMAND	0xff

/* DCP_CFG CEETM enable bit for a sub-portal */
#define	QM_SP_ENABLE_CEETM(sp)		(0x80000000 >> (sp))

/* ================================================================
 * CEETM MC Command Structures (MCC)
 *
 * Each is exactly 63 bytes (64 - 1 verb byte).  These overlay the
 * generic qm_mc_command union returned by qm_mc_start().  All
 * multi-byte fields are big-endian on the wire; callers use
 * WRITE_UINT16/WRITE_UINT32 macros.
 *
 * Field layout matches Linux include/linux/fsl_qman.h verbatim.
 * ================================================================ */

struct qm_mcc_ceetm_lfqmt_config {
	uint8_t		__reserved1[3];
	uint8_t		lfqid_hi;	/* lfqid[23:16] */
	uint16_t	lfqid_lo;	/* lfqid[15:0] */
	uint8_t		__reserved2[2];
	uint16_t	cqid;
	uint8_t		__reserved3[2];
	uint16_t	dctidx;
	uint8_t		__reserved4[48];
} __packed;

struct qm_mcc_ceetm_lfqmt_query {
	uint8_t		__reserved1[3];
	uint8_t		lfqid_hi;
	uint16_t	lfqid_lo;
	uint8_t		__reserved2[56];
} __packed;

struct qm_mcc_ceetm_cq_config {
	uint16_t	cqid;
	uint8_t		dcpid;
	uint8_t		__reserved1;
	uint16_t	ccgid;
	uint8_t		__reserved2[56];
} __packed;

struct qm_mcc_ceetm_cq_query {
	uint16_t	cqid;
	uint8_t		dcpid;
	uint8_t		__reserved1[59];
} __packed;

struct qm_mcc_ceetm_dct_config {
	uint16_t	dctidx;
	uint8_t		dcpid;
	uint8_t		__reserved1[15];
	uint32_t	context_b;
	uint64_t	context_a;
	uint8_t		__reserved2[32];
} __packed;

struct qm_mcc_ceetm_dct_query {
	uint16_t	dctidx;
	uint8_t		dcpid;
	uint8_t		__reserved1[59];
} __packed;

/*
 * Class scheduler config — GPC byte layout (big-endian on wire):
 *   bit 7: reserved
 *   bit 6: combine_flag
 *   bits 5-3: prio_b
 *   bits 2-0: prio_a
 */
#define	CEETM_GPC_COMBINE_FLAG	0x40
#define	CEETM_GPC_PRIO_B_SHIFT	3
#define	CEETM_GPC_PRIO_B_MASK	0x38
#define	CEETM_GPC_PRIO_A_MASK	0x07

struct qm_mcc_ceetm_class_scheduler_config {
	uint16_t	cqcid;		/* channel CQ channel id */
	uint8_t		dcpid;
	uint8_t		__reserved1[6];
	uint8_t		gpc;		/* group priority control */
	uint16_t	crem;		/* CR eligibility mask */
	uint16_t	erem;		/* ER eligibility mask */
	uint8_t		w[8];		/* weight codes */
	uint8_t		__reserved2[40];
} __packed;

struct qm_mcc_ceetm_class_scheduler_query {
	uint16_t	cqcid;
	uint8_t		dcpid;
	uint8_t		__reserved1[59];
} __packed;

/*
 * MAPPING_SHAPER_TCFC config — multiplexed command.
 * The upper 4 bits of cid select the sub-command type.
 */
struct qm_mcc_ceetm_mapping_shaper_tcfc_config {
	uint16_t	cid;		/* sub-command type | resource index */
	uint8_t		dcpid;
	union {
		struct {
			uint8_t		map_flags;  /* bit7=shaped, bits2-0=lni_id */
			uint8_t		__reserved[58];
		} __packed channel_mapping;
		struct {
			uint8_t		map_flags;  /* bits2-0=lni_id */
			uint8_t		__reserved[58];
		} __packed sp_mapping;
		struct {
			uint8_t		cpl_oal;    /* bit7=coupled, bits4-0=oal */
			uint8_t		crtcr_hi;   /* crtcr[23:16] */
			uint16_t	crtcr_lo;   /* crtcr[15:0] */
			uint8_t		ertcr_hi;   /* ertcr[23:16] */
			uint16_t	ertcr_lo;   /* ertcr[15:0] */
			uint16_t	crtbl;
			uint16_t	ertbl;
			uint8_t		mps;
			uint8_t		__reserved[47];
		} __packed shaper_config;
		struct {
			uint8_t		__reserved1[11];
			uint64_t	lnitcfcc;
			uint8_t		__reserved2[40];
		} __packed tcfc_config;
	};
} __packed;

struct qm_mcc_ceetm_mapping_shaper_tcfc_query {
	uint16_t	cid;
	uint8_t		dcpid;
	uint8_t		__reserved[59];
} __packed;

/*
 * CCGR config — Congestion Manager or Dequeue Notification config.
 * We only use CM config (tail-drop threshold for class queues).
 */
struct qm_mcc_ceetm_ccgr_config {
	uint16_t	ccgrid;		/* sub-command | ccg index */
	uint8_t		dcpid;
	uint8_t		__reserved1;
	uint16_t	we_mask;
	/* CM config fields (when ccgrid sub-command = CM_CONFIGURE) */
	uint8_t		ctl;		/* control byte */
	uint8_t		cdv;
	uint16_t	cscn_tupd;
	uint8_t		oal;
	uint8_t		__reserved2;
	uint16_t	cs_thres;	/* qm_cgr_cs_thres format */
	uint16_t	cs_thres_x;
	uint16_t	td_thres;
	uint32_t	wr_parm_g;	/* qm_cgr_wr_parm format */
	uint32_t	wr_parm_y;
	uint32_t	wr_parm_r;
	uint8_t		__reserved3[32];
} __packed;

struct qm_mcc_ceetm_ccgr_query {
	uint16_t	ccgrid;
	uint8_t		dcpid;
	uint8_t		__reserved[59];
} __packed;

struct qm_mcc_ceetm_cq_peek_pop_xsfdrread {
	uint16_t	cqid;
	uint8_t		dcpid;
	uint8_t		ct;
	uint16_t	xsfdr;
	uint8_t		__reserved[56];
} __packed;

struct qm_mcc_ceetm_statistics_query_write {
	uint16_t	cid;
	uint8_t		dcpid;
	uint8_t		ct;
	uint8_t		__reserved1[13];
	uint8_t		frm_cnt[5];	/* 40-bit frame count */
	uint8_t		__reserved2[2];
	uint8_t		byte_cnt[6];	/* 48-bit byte count */
	uint8_t		__reserved3[32];
} __packed;

/* ================================================================
 * CEETM MC Result Structures (MCR)
 *
 * Each is exactly 64 bytes.  First two bytes are verb + result.
 * These overlay the generic qm_mc_result returned by qm_mc_result().
 * ================================================================ */

struct qm_mcr_ceetm_generic {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved[62];
} __packed;

/* LFQMT query result */
struct qm_mcr_ceetm_lfqmt_query {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[8];
	uint16_t	cqid;
	uint8_t		__reserved2[2];
	uint16_t	dctidx;
	uint8_t		__reserved3[2];
	uint16_t	ccgid;
	uint8_t		__reserved4[44];
} __packed;

/* CQ query result */
struct qm_mcr_ceetm_cq_query {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[4];
	uint16_t	ccgid;
	uint16_t	state;
	uint8_t		pfdr_hptr_hi;
	uint16_t	pfdr_hptr_lo;
	uint8_t		pfdr_tptr_hi;
	uint16_t	pfdr_tptr_lo;
	uint16_t	od1_xsfdr;
	uint16_t	od2_xsfdr;
	uint16_t	od3_xsfdr;
	uint16_t	od4_xsfdr;
	uint16_t	od5_xsfdr;
	uint16_t	od6_xsfdr;
	uint16_t	ra1_xsfdr;
	uint16_t	ra2_xsfdr;
	uint8_t		__reserved2;
	uint8_t		frm_cnt_hi;
	uint16_t	frm_cnt_lo;
	uint8_t		__reserved3[28];
} __packed;

/* DCT query result */
struct qm_mcr_ceetm_dct_query {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[18];
	uint32_t	context_b;
	uint64_t	context_a;
	uint8_t		__reserved2[32];
} __packed;

/* Class scheduler query result */
struct qm_mcr_ceetm_class_scheduler_query {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[9];
	uint8_t		gpc;
	uint16_t	crem;
	uint16_t	erem;
	uint8_t		w[8];
	uint8_t		__reserved2[5];
	uint8_t		wbfslist_hi;
	uint16_t	wbfslist_lo;
	uint32_t	d8;
	uint32_t	d9;
	uint32_t	d10;
	uint32_t	d11;
	uint32_t	d12;
	uint32_t	d13;
	uint32_t	d14;
	uint32_t	d15;
} __packed;

/* MAPPING_SHAPER_TCFC query result */
struct qm_mcr_ceetm_mapping_shaper_tcfc_query {
	uint8_t		verb;
	uint8_t		result;
	uint16_t	cid;
	uint8_t		__reserved1;
	union {
		struct {
			uint8_t		map_flags;
			uint8_t		__reserved[58];
		} __packed channel_mapping_query;
		struct {
			uint8_t		map_flags;
			uint8_t		__reserved[58];
		} __packed sp_mapping_query;
		struct {
			uint8_t		cpl_oal;
			uint8_t		crtcr_hi;
			uint16_t	crtcr_lo;
			uint8_t		ertcr_hi;
			uint16_t	ertcr_lo;
			uint16_t	crtbl;
			uint16_t	ertbl;
			uint8_t		mps;
			uint8_t		__reserved1[15];
			uint32_t	crat;
			uint32_t	erat;
			uint8_t		__reserved2[24];
		} __packed shaper_query;
		struct {
			uint8_t		__reserved1[11];
			uint64_t	lnitcfcc;
			uint8_t		__reserved2[40];
		} __packed tcfc_query;
	};
} __packed;

/* CCGR config result (test-write variant has extra fields) */
struct qm_mcr_ceetm_ccgr_config {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[62];
} __packed;

/* CCGR query result (CM query variant) */
struct qm_mcr_ceetm_ccgr_query {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[6];
	/* CM query fields */
	uint8_t		ctl;
	uint8_t		cdv;
	uint8_t		__reserved2[2];
	uint8_t		oal;
	uint8_t		__reserved3;
	uint16_t	cs_thres;
	uint16_t	cs_thres_x;
	uint16_t	td_thres;
	uint32_t	wr_parm_g;
	uint32_t	wr_parm_y;
	uint32_t	wr_parm_r;
	uint16_t	cscn_targ_dcp;
	uint8_t		dcp_lsn;
	uint8_t		i_cnt[5];	/* 40-bit */
	uint8_t		__reserved4[3];
	uint8_t		a_cnt[5];	/* 40-bit */
	uint32_t	cscn_targ_swp[4];
} __packed;

/* CQ peek/pop result */
struct qm_mcr_ceetm_cq_peek_pop_xsfdrread {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		stat;
	uint8_t		__reserved1[61];
} __packed;

/* Statistics query result */
struct qm_mcr_ceetm_statistics_query {
	uint8_t		verb;
	uint8_t		result;
	uint8_t		__reserved1[17];
	uint8_t		frm_cnt[5];	/* 40-bit frame count */
	uint8_t		__reserved2[2];
	uint8_t		byte_cnt[6];	/* 48-bit byte count */
	uint8_t		__reserved3[32];
} __packed;

/* ================================================================
 * CEETM High-Level Data Structures
 *
 * These represent the CEETM resource hierarchy.  Ported from
 * Linux include/linux/fsl_qman.h, adapted to FreeBSD conventions.
 * ================================================================ */

/* Maximum CEETM instances (one per FMan / DCP portal) */
#define	QMAN_CEETM_MAX		2

/* Token rate (for shapers) */
struct qm_ceetm_rate {
	uint32_t	whole;		/* integer part (0..2047) */
	uint32_t	fraction;	/* fractional part (0..8191) */
};

/* WBFS weight code */
struct qm_ceetm_weight_code {
	uint8_t		y;		/* 0..31 */
	uint8_t		x;		/* 0..7 */
	/* effective weight = 2^(x+6) / (64 - y) */
};

/* DCP portal enum — already defined in NCSW fsl_qman.h for kernel context */
#ifndef __FSL_QMAN_H
enum qm_dc_portal {
	qm_dc_portal_fman0 = 0,
	qm_dc_portal_fman1 = 1,
};
#endif

/* Sub-portal */
struct qm_ceetm_sp {
	TAILQ_ENTRY(qm_ceetm_sp) entries;
	unsigned int		idx;
	unsigned int		dcp_idx;
	int			is_claimed;
	struct qm_ceetm_lni	*lni;
};

/* Logical Network Interface */
struct qm_ceetm_lni {
	TAILQ_ENTRY(qm_ceetm_lni) entries;
	unsigned int		idx;
	unsigned int		dcp_idx;
	int			is_claimed;
	struct qm_ceetm_sp	*sp;
	TAILQ_HEAD(, qm_ceetm_channel) channels;
	int			shaper_enable;
	int			shaper_couple;
	int			oal;
	struct qm_ceetm_rate	cr_token_rate;
	struct qm_ceetm_rate	er_token_rate;
	uint16_t		cr_token_bucket_limit;
	uint16_t		er_token_bucket_limit;
};

/* CQ Channel */
struct qm_ceetm_channel {
	TAILQ_ENTRY(qm_ceetm_channel) entries;
	struct qm_ceetm_lni	*lni;	/* parent LNI */
	unsigned int		idx;
	unsigned int		lni_idx;
	unsigned int		dcp_idx;
	TAILQ_HEAD(, qm_ceetm_cq) class_queues;
	TAILQ_HEAD(, qm_ceetm_ccg) ccgs;
	uint8_t			shaper_enable;
	uint8_t			shaper_couple;
	struct qm_ceetm_rate	cr_token_rate;
	struct qm_ceetm_rate	er_token_rate;
	uint16_t		cr_token_bucket_limit;
	uint16_t		er_token_bucket_limit;
};

/* Class Queue */
struct qm_ceetm_cq {
	TAILQ_ENTRY(qm_ceetm_cq) entries;
	struct qm_ceetm_channel	*parent;
	struct qm_ceetm_ccg	*ccg;
	unsigned int		idx;	/* 0-7 individual, 8-15 grouped */
	int			is_claimed;
	TAILQ_HEAD(, qm_ceetm_lfq) bound_lfqids;
};

/* Logical Frame Queue */
struct qm_ceetm_lfq {
	TAILQ_ENTRY(qm_ceetm_lfq) entries;
	struct qm_ceetm_cq	*parent;
	unsigned int		idx;	/* LFQID */
	unsigned int		dctidx;	/* dequeue context table index */
	uint64_t		context_a;
	uint32_t		context_b;
};

/* Class Congestion Group */
struct qm_ceetm_ccg {
	TAILQ_ENTRY(qm_ceetm_ccg) entries;
	struct qm_ceetm_channel	*parent;
	unsigned int		idx;	/* 0-15 */
};

/* Top-level CEETM instance (one per DCP/FMan) */
struct qm_ceetm {
	unsigned int		idx;		/* DCP portal index */
	TAILQ_HEAD(, qm_ceetm_sp) sub_portals;
	TAILQ_HEAD(, qm_ceetm_lni) lnis;
	unsigned int		sp_range[2];	/* [start, count] */
	unsigned int		lni_range[2];
};

/* ================================================================
 * CEETM CCSR Register Offsets (relative to QMan CCSR base)
 * ================================================================ */

#define	QM_REG_DCP_CFG(n)		(0x0300 + ((n) * 0x10))
#define	QM_REG_CEETM_CFG_IDX		0x0900
#define	QM_REG_CEETM_CFG_PRES		0x0904
#define	QM_REG_CEETM_XSFDR_IN_USE	0x0908

/* ================================================================
 * CEETM CCGR WE (Write Enable) Mask Bits
 * ================================================================ */

#define	QM_CEETM_CCGR_WE_CSCN_EN	0x4000
#define	QM_CEETM_CCGR_WE_TD_EN		0x2000
#define	QM_CEETM_CCGR_WE_TD_MODE	0x1000
#define	QM_CEETM_CCGR_WE_TD_THRES	0x0800
#define	QM_CEETM_CCGR_WE_OAL		0x0400
#define	QM_CEETM_CCGR_WE_CDV		0x0200
#define	QM_CEETM_CCGR_WE_CSCN_TUPD	0x0100
#define	QM_CEETM_CCGR_WE_CS_THRES	0x0080
#define	QM_CEETM_CCGR_WE_CS_THRES_X	0x0040
#define	QM_CEETM_CCGR_WE_MODE		0x0020
#define	QM_CEETM_CCGR_WE_WR_PARM_G	0x0010
#define	QM_CEETM_CCGR_WE_WR_PARM_Y	0x0008
#define	QM_CEETM_CCGR_WE_WR_PARM_R	0x0004
#define	QM_CEETM_CCGR_WE_DN		0x0002

/* CCGR CTL byte bit definitions */
#define	QM_CEETM_CCGR_CTL_WR_EN_G	0x40
#define	QM_CEETM_CCGR_CTL_WR_EN_Y	0x20
#define	QM_CEETM_CCGR_CTL_WR_EN_R	0x10
#define	QM_CEETM_CCGR_CTL_TD_EN	0x08
#define	QM_CEETM_CCGR_CTL_TD_MODE	0x04
#define	QM_CEETM_CCGR_CTL_CSCN_EN	0x02
#define	QM_CEETM_CCGR_CTL_MODE		0x01

/* ================================================================
 * Public API — implemented in qman_ceetm.c
 * ================================================================ */

/* Initialization (called from qman_attach if DT node present) */
int	qman_ceetm_init(void);

/* Global state */
extern uint8_t		qman_num_ceetms;
extern struct qm_ceetm	qman_ceetms[QMAN_CEETM_MAX];

/* Pure math */
int	qman_ceetm_bps2tokenrate(uint64_t bps,
	    struct qm_ceetm_rate *token_rate, int rounding);
int	qman_ceetm_tokenrate2bps(const struct qm_ceetm_rate *token_rate,
	    uint64_t *bps, int rounding);
int	qman_ceetm_ratio2wbfs(uint32_t numerator, uint32_t denominator,
	    struct qm_ceetm_weight_code *weight_code, int rounding);
int	qman_ceetm_wbfs2ratio(struct qm_ceetm_weight_code *weight_code,
	    uint32_t *numerator, uint32_t *denominator);

/* Sub-portal management */
int	qman_ceetm_sp_claim(struct qm_ceetm_sp **sp,
	    enum qm_dc_portal dcp_idx, unsigned int sp_idx);
int	qman_ceetm_sp_release(struct qm_ceetm_sp *sp);
int	qman_ceetm_sp_set_lni(struct qm_ceetm_sp *sp,
	    struct qm_ceetm_lni *lni);
int	qman_ceetm_sp_get_lni(struct qm_ceetm_sp *sp, unsigned int *lni_idx);

/* LNI management */
int	qman_ceetm_lni_claim(struct qm_ceetm_lni **lni,
	    enum qm_dc_portal dcp_idx, unsigned int lni_idx);
int	qman_ceetm_lni_release(struct qm_ceetm_lni *lni);
int	qman_ceetm_lni_enable_shaper(struct qm_ceetm_lni *lni,
	    int coupled, int oal);
int	qman_ceetm_lni_disable_shaper(struct qm_ceetm_lni *lni);
int	qman_ceetm_lni_is_shaper_enabled(struct qm_ceetm_lni *lni);
int	qman_ceetm_lni_set_commit_rate(struct qm_ceetm_lni *lni,
	    const struct qm_ceetm_rate *cr, uint16_t cr_limit);
int	qman_ceetm_lni_set_excess_rate(struct qm_ceetm_lni *lni,
	    const struct qm_ceetm_rate *er, uint16_t er_limit);
int	qman_ceetm_lni_get_commit_rate(struct qm_ceetm_lni *lni,
	    struct qm_ceetm_rate *cr, uint16_t *cr_limit);
int	qman_ceetm_lni_get_excess_rate(struct qm_ceetm_lni *lni,
	    struct qm_ceetm_rate *er, uint16_t *er_limit);

/* Channel management */
int	qman_ceetm_channel_claim(struct qm_ceetm_channel **channel,
	    struct qm_ceetm_lni *lni);
int	qman_ceetm_channel_release(struct qm_ceetm_channel *channel);
int	qman_ceetm_channel_enable_shaper(struct qm_ceetm_channel *channel,
	    int coupled);
int	qman_ceetm_channel_disable_shaper(struct qm_ceetm_channel *channel);
int	qman_ceetm_channel_is_shaper_enabled(
	    struct qm_ceetm_channel *channel);
int	qman_ceetm_channel_set_commit_rate(struct qm_ceetm_channel *channel,
	    const struct qm_ceetm_rate *cr, uint16_t cr_limit);
int	qman_ceetm_channel_set_excess_rate(struct qm_ceetm_channel *channel,
	    const struct qm_ceetm_rate *er, uint16_t er_limit);
int	qman_ceetm_channel_get_commit_rate(struct qm_ceetm_channel *channel,
	    struct qm_ceetm_rate *cr, uint16_t *cr_limit);
int	qman_ceetm_channel_get_excess_rate(struct qm_ceetm_channel *channel,
	    struct qm_ceetm_rate *er, uint16_t *er_limit);
int	qman_ceetm_channel_set_group(struct qm_ceetm_channel *channel,
	    int group_b, unsigned int prio_a, unsigned int prio_b);
int	qman_ceetm_channel_get_group(struct qm_ceetm_channel *channel,
	    int *group_b, unsigned int *prio_a, unsigned int *prio_b);
int	qman_ceetm_channel_set_group_cr_eligibility(
	    struct qm_ceetm_channel *channel, int group_b, int cre);
int	qman_ceetm_channel_set_group_er_eligibility(
	    struct qm_ceetm_channel *channel, int group_b, int ere);
int	qman_ceetm_channel_set_cq_cr_eligibility(
	    struct qm_ceetm_channel *channel, unsigned int idx, int cre);
int	qman_ceetm_channel_set_cq_er_eligibility(
	    struct qm_ceetm_channel *channel, unsigned int idx, int ere);

/* Class queue management */
int	qman_ceetm_cq_claim(struct qm_ceetm_cq **cq,
	    struct qm_ceetm_channel *channel, unsigned int idx,
	    struct qm_ceetm_ccg *ccg);
int	qman_ceetm_cq_claim_A(struct qm_ceetm_cq **cq,
	    struct qm_ceetm_channel *channel, unsigned int idx,
	    struct qm_ceetm_ccg *ccg);
int	qman_ceetm_cq_release(struct qm_ceetm_cq *cq);
int	qman_ceetm_set_queue_weight(struct qm_ceetm_cq *cq,
	    struct qm_ceetm_weight_code *weight_code);
int	qman_ceetm_get_queue_weight(struct qm_ceetm_cq *cq,
	    struct qm_ceetm_weight_code *weight_code);

/* LFQ management */
int	qman_ceetm_lfq_claim(struct qm_ceetm_lfq **lfq,
	    struct qm_ceetm_cq *cq);
int	qman_ceetm_lfq_release(struct qm_ceetm_lfq *lfq);
int	qman_ceetm_lfq_set_context(struct qm_ceetm_lfq *lfq,
	    uint64_t context_a, uint32_t context_b);
int	qman_ceetm_lfq_get_context(struct qm_ceetm_lfq *lfq,
	    uint64_t *context_a, uint32_t *context_b);

/*
 * QMan FQ for CEETM egress — minimal struct matching Linux fsl_qman.h.
 * In kernel context, NCSW qm.h provides the full struct qman_fq.
 * In module context, dpaa_eth.h provides the minimal struct.
 */
#if !defined(__QM_H) && !defined(_QMAN_FQ_DEFINED)
#define	_QMAN_FQ_DEFINED
struct qman_fq {
	uint32_t	fqid;
	uint32_t	flags;
};
#endif
int	qman_ceetm_create_fq(struct qm_ceetm_lfq *lfq,
	    struct qman_fq *fq);

/* CCG management */
int	qman_ceetm_ccg_claim(struct qm_ceetm_ccg **ccg,
	    struct qm_ceetm_channel *channel, unsigned int idx,
	    void (*cb)(void *, int), void *cb_ctx);
int	qman_ceetm_ccg_release(struct qm_ceetm_ccg *ccg);
int	qman_ceetm_ccg_set(struct qm_ceetm_ccg *ccg,
	    uint16_t we_mask, struct qm_mcc_ceetm_ccgr_config *opts);
int	qman_ceetm_ccg_get_reject_statistics(struct qm_ceetm_ccg *ccg,
	    uint32_t flags, uint64_t *frame_count, uint64_t *byte_count);

/* Statistics/query */
int	qman_ceetm_query_cq(unsigned int cqid, unsigned int dcpid,
	    struct qm_mcr_ceetm_cq_query *cq_query);
int	qman_ceetm_cq_get_dequeue_statistics(struct qm_ceetm_cq *cq,
	    uint32_t flags, uint64_t *frame_count, uint64_t *byte_count);

/* CCSR register access */
int	qman_sp_enable_ceetm_mode(enum qm_dc_portal portal,
	    uint16_t sub_portal);
int	qman_sp_disable_ceetm_mode(enum qm_dc_portal portal,
	    uint16_t sub_portal);
int	qman_ceetm_configure_mapping_shaper_tcfc(
	    struct qm_mcc_ceetm_mapping_shaper_tcfc_config *opts);
int	qman_ceetm_get_xsfdr(enum qm_dc_portal portal, unsigned int *num);

#endif /* _QMAN_CEETM_H */
