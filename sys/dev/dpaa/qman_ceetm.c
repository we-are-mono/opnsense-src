/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Mono Technologies Inc.
 *
 * QMan CEETM (Channel Egress Enhancement and Traffic Management) driver
 * for FreeBSD.  Ported from the Linux NXP QBMan SDK driver.
 *
 * Original Linux source:
 *   drivers/staging/fsl_qbman/qman_high.c (API implementation)
 *   drivers/staging/fsl_qbman/qman_driver.c (DT init)
 *   drivers/staging/fsl_qbman/qman_config.c (CCSR registers)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/pcpu.h>
#include <sys/sched.h>
#include <sys/smp.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

/* NCSW internal headers — accessible because we compile with DPAA_COMPILE_CMD */
#include "qm.h"
#include "qman_low.h"

/* FreeBSD DPAA headers */
#include "qman.h"
#include "qman_ceetm.h"

MALLOC_DECLARE(M_NETCOMMSW);

/* ================================================================
 * Global State
 * ================================================================ */

uint8_t			qman_num_ceetms;
struct qm_ceetm		qman_ceetms[QMAN_CEETM_MAX];

/* QMan clock frequency (Hz) — set during init from DT or default */
static uint32_t		qman_clk;

/* CEETM update period in nanoseconds (default 1µs) */
#define	CEETM_UPDATE_PERIOD_NS	1000

/* LFQID and Channel allocators — simple bitmaps */
#define	CEETM_MAX_LFQIDS	4096
#define	CEETM_MAX_CHANNELS	32

struct ceetm_pool {
	uint32_t	base;
	uint32_t	count;
	uint32_t	bitmap[CEETM_MAX_LFQIDS / 32];	/* max size */
};

static struct ceetm_pool ceetm_lfqid_pool[QMAN_CEETM_MAX];
static struct ceetm_pool ceetm_channel_pool[QMAN_CEETM_MAX];
static struct mtx	ceetm_alloc_lock;

static void
ceetm_pool_seed(struct ceetm_pool *pool, uint32_t base, uint32_t count)
{
	pool->base = base;
	pool->count = count;
	memset(pool->bitmap, 0, sizeof(pool->bitmap));
}

static int
ceetm_pool_alloc(struct ceetm_pool *pool, uint32_t *id)
{
	unsigned int i;

	mtx_lock(&ceetm_alloc_lock);
	for (i = 0; i < pool->count; i++) {
		if (!(pool->bitmap[i / 32] & (1U << (i % 32)))) {
			pool->bitmap[i / 32] |= (1U << (i % 32));
			*id = pool->base + i;
			mtx_unlock(&ceetm_alloc_lock);
			return (0);
		}
	}
	mtx_unlock(&ceetm_alloc_lock);
	return (ENOSPC);
}

static void
ceetm_pool_free(struct ceetm_pool *pool, uint32_t id)
{
	unsigned int idx;

	if (id < pool->base || id >= pool->base + pool->count)
		return;
	idx = id - pool->base;
	mtx_lock(&ceetm_alloc_lock);
	pool->bitmap[idx / 32] &= ~(1U << (idx % 32));
	mtx_unlock(&ceetm_alloc_lock);
}

/* ================================================================
 * Portal MC (Management Command) Helpers
 *
 * Pattern: sched_pin → get portal → NCSW_PLOCK → mc_start →
 *          fill command → mc_commit → spin on mc_result →
 *          check result → PUNLOCK → sched_unpin
 * ================================================================ */

/*
 * Get the current CPU's NCSW QM portal, with preemption pinned.
 * Caller must have called sched_pin() first.
 */
static inline t_QmPortal *
ceetm_get_portal(void)
{
	return ((t_QmPortal *)qman_sc->sc_qph[PCPU_GET(cpuid)]);
}

/*
 * Get the t_Qm handle for CCSR register access.
 */
static inline t_Qm *
ceetm_get_qm(void)
{
	return ((t_Qm *)qman_sc->sc_qh);
}

/*
 * CCSR register read/write — for registers not in t_QmRegs struct.
 * Uses byte offset from QMan CCSR base.
 */
static inline uint32_t
ceetm_ccsr_read(uint32_t offset)
{
	volatile uint32_t *reg;

	reg = (volatile uint32_t *)((uint8_t *)ceetm_get_qm()->p_QmRegs +
	    offset);
	return (GET_UINT32(*reg));
}

static inline void
ceetm_ccsr_write(uint32_t offset, uint32_t val)
{
	volatile uint32_t *reg;

	reg = (volatile uint32_t *)((uint8_t *)ceetm_get_qm()->p_QmRegs +
	    offset);
	WRITE_UINT32(*reg, val);
}

/* Rounding helper for integer division */
static inline uint64_t
ceetm_div_round(uint64_t n, uint64_t d, int rounding)
{
	if (rounding < 0)
		return (n / d);		/* round down */
	else if (rounding > 0)
		return ((n + d - 1) / d);	/* round up */
	else
		return ((n + d / 2) / d);	/* round nearest */
}

/* ================================================================
 * CCSR Register Access Functions
 * ================================================================ */

int
qman_sp_enable_ceetm_mode(enum qm_dc_portal portal, uint16_t sub_portal)
{
	uint32_t dcp_cfg;
	t_Qm *qm;

	if (qman_num_ceetms == 0)
		return (ENODEV);

	qm = ceetm_get_qm();
	dcp_cfg = GET_UINT32(qm->p_QmRegs->dcpConfRegs[portal].cfg);
	dcp_cfg |= QM_SP_ENABLE_CEETM(sub_portal);
	WRITE_UINT32(qm->p_QmRegs->dcpConfRegs[portal].cfg, dcp_cfg);
	return (0);
}

int
qman_sp_disable_ceetm_mode(enum qm_dc_portal portal, uint16_t sub_portal)
{
	uint32_t dcp_cfg;
	t_Qm *qm;

	if (qman_num_ceetms == 0)
		return (ENODEV);

	qm = ceetm_get_qm();
	dcp_cfg = GET_UINT32(qm->p_QmRegs->dcpConfRegs[portal].cfg);
	dcp_cfg &= ~QM_SP_ENABLE_CEETM(sub_portal);
	WRITE_UINT32(qm->p_QmRegs->dcpConfRegs[portal].cfg, dcp_cfg);
	return (0);
}

int
qman_ceetm_get_xsfdr(enum qm_dc_portal portal, unsigned int *num)
{
	if (qman_num_ceetms == 0)
		return (ENODEV);
	*num = ceetm_ccsr_read(QM_REG_CEETM_XSFDR_IN_USE);
	return (0);
}

static int
qman_ceetm_set_prescaler(enum qm_dc_portal portal)
{
	uint64_t temp;
	uint16_t pres;

	if (qman_clk == 0)
		return (EINVAL);

	/*
	 * PRES = (2^22 * 10^9 / update_period_ns) / qman_clk
	 * Split to avoid 64-bit overflow:
	 *   temp = 0x400000 * 100
	 *   temp /= (update_period_ns / 10000000)
	 *   But simpler: temp = (0x400000ULL * 1000000000ULL) / (period * clk)
	 */
	temp = 0x400000ULL * 100;
	temp /= CEETM_UPDATE_PERIOD_NS;
	temp *= 10000000;
	temp /= qman_clk;
	pres = (uint16_t)temp;

	ceetm_ccsr_write(QM_REG_CEETM_CFG_IDX, portal);
	ceetm_ccsr_write(QM_REG_CEETM_CFG_PRES, pres);
	return (0);
}

static int
qman_ceetm_get_prescaler(uint16_t *pres)
{
	if (qman_num_ceetms == 0)
		return (ENODEV);
	*pres = (uint16_t)ceetm_ccsr_read(QM_REG_CEETM_CFG_PRES);
	return (0);
}

/* ================================================================
 * Low-Level MC Command Wrappers
 *
 * Each sends a single MC command and returns the result code.
 * The command struct is filled by the caller; these functions handle
 * portal lock/unlock, commit, and result polling.
 * ================================================================ */

/*
 * Generic MC command execution pattern.
 * cmd points to the filled command struct (excluding verb byte).
 * cmd_size is the size of the command struct.
 * verb is the CEETM verb code.
 * result is filled with the 64-byte result on success.
 * Returns 0 on success, error code otherwise.
 */
static int
ceetm_mc_command(uint8_t verb, const void *cmd, size_t cmd_size,
    void *result, size_t result_size)
{
	t_QmPortal *portal;
	struct qm_portal *low;
	struct qm_mc_command *mcc;
	struct qm_mc_result *mcr;
	struct qm_mcr_ceetm_generic *gen;
	int timeout;

	sched_pin();
	portal = ceetm_get_portal();
	if (portal == NULL) {
		sched_unpin();
		return (ENXIO);
	}

	low = portal->p_LowQmPortal;
	NCSW_PLOCK(portal);

	mcc = qm_mc_start(low);
	if (mcc == NULL) {
		PUNLOCK(portal);
		sched_unpin();
		return (EBUSY);
	}

	/* Copy command data after the verb byte */
	memcpy((uint8_t *)mcc + 1, cmd, cmd_size);
	qm_mc_commit(low, verb);

	/* Poll for result */
	timeout = 10000;
	do {
		mcr = qm_mc_result(low);
		if (mcr != NULL)
			break;
		cpu_spinwait();
	} while (--timeout > 0);

	PUNLOCK(portal);
	sched_unpin();

	if (timeout == 0) {
		printf("ceetm: MC command 0x%02x timeout\n", verb);
		return (ETIMEDOUT);
	}

	/* Check result code */
	gen = (struct qm_mcr_ceetm_generic *)mcr;
	if (gen->result != QM_MCR_RESULT_OK) {
		printf("ceetm: MC command 0x%02x failed, result=0x%02x\n",
		    verb, gen->result);
		return (EIO);
	}

	if (result != NULL && result_size > 0)
		memcpy(result, mcr, result_size);
	return (0);
}

/* Convenience wrappers for config (no result needed) and query (result needed) */
static int
ceetm_mc_config(uint8_t verb, const void *cmd, size_t cmd_size)
{
	return (ceetm_mc_command(verb, cmd, cmd_size, NULL, 0));
}

static int
ceetm_mc_query(uint8_t verb, const void *cmd, size_t cmd_size,
    void *result, size_t result_size)
{
	return (ceetm_mc_command(verb, cmd, cmd_size, result, result_size));
}

/* ================================================================
 * Pure Math Functions
 * ================================================================ */

int
qman_ceetm_bps2tokenrate(uint64_t bps, struct qm_ceetm_rate *token_rate,
    int rounding)
{
	uint16_t pres;
	uint64_t temp, qman_freq;
	int ret;

	ret = qman_ceetm_get_prescaler(&pres);
	if (ret)
		return (EINVAL);
	if (pres == 0)
		return (EINVAL);

	qman_freq = (uint64_t)qman_clk;
	if (qman_freq == 0)
		return (EINVAL);

	/*
	 * N = (bps * 2^32) / (PRES * QHz)
	 * Split into two 16-bit shifts to avoid overflow:
	 *   N = (((bps << 16) / PRES) << 16) / QHz
	 */
	temp = ceetm_div_round(bps << 16, pres, rounding);
	temp = ceetm_div_round(temp << 16, qman_freq, rounding);
	token_rate->whole = (uint32_t)(temp >> 13);
	token_rate->fraction = (uint32_t)(temp & 0x1FFF);
	return (0);
}

int
qman_ceetm_tokenrate2bps(const struct qm_ceetm_rate *token_rate,
    uint64_t *bps, int rounding)
{
	uint16_t pres;
	uint64_t temp, qman_freq;
	int ret;

	ret = qman_ceetm_get_prescaler(&pres);
	if (ret)
		return (EINVAL);

	qman_freq = (uint64_t)qman_clk;
	if (qman_freq == 0)
		return (EINVAL);

	/*
	 * bps = N * PRES * QHz / 2^32
	 * Split: temp = (PRES * QHz) >> 16, then bps = (temp * N) >> 16
	 */
	temp = ceetm_div_round(qman_freq * pres, (uint64_t)1 << 16, rounding);
	temp *= ((uint64_t)token_rate->whole << 13) + token_rate->fraction;
	*bps = ceetm_div_round(temp, (uint64_t)1 << 16, rounding);
	return (0);
}

/*
 * Find greatest common divisor.
 */
static uint32_t
ceetm_gcd(uint32_t a, uint32_t b)
{
	while (b != 0) {
		uint32_t t = b;
		b = a % b;
		a = t;
	}
	return (a);
}

int
qman_ceetm_ratio2wbfs(uint32_t numerator, uint32_t denominator,
    struct qm_ceetm_weight_code *weight_code, int rounding)
{
	uint32_t g;
	uint64_t n, d;
	int best_err;

	if (denominator == 0 || numerator == 0)
		return (EINVAL);

	/* Reduce fraction */
	g = ceetm_gcd(numerator, denominator);
	n = numerator / g;
	d = denominator / g;

	/*
	 * WBFS weight = 2^(x+6) / (64-y)
	 * Find (x,y) such that n/d ~ 2^(x+6)/(64-y)
	 * i.e., y ~ 64 - d * 2^(x+6) / n
	 */
	best_err = INT_MAX;
	weight_code->x = 0;
	weight_code->y = 0;

	for (uint8_t x = 0; x <= 7; x++) {
		int64_t ideal_64_minus_y;
		int64_t y_candidate;
		int err;

		ideal_64_minus_y = (int64_t)((d * ((uint64_t)1 << (x + 6))) /
		    n);
		y_candidate = 64 - ideal_64_minus_y;

		if (y_candidate < 0)
			y_candidate = 0;
		if (y_candidate > 31)
			y_candidate = 31;

		/* Check error */
		err = (int)((int64_t)n * (64 - y_candidate) -
		    (int64_t)d * ((uint64_t)1 << (x + 6)));
		if (err < 0)
			err = -err;

		if (err < best_err) {
			best_err = err;
			weight_code->x = x;
			weight_code->y = (uint8_t)y_candidate;
			if (err == 0)
				break;
		}
	}
	return (0);
}

int
qman_ceetm_wbfs2ratio(struct qm_ceetm_weight_code *weight_code,
    uint32_t *numerator, uint32_t *denominator)
{
	uint32_t g;

	/* weight = 2^(x+6) / (64-y) */
	*numerator = 1U << (weight_code->x + 6);
	*denominator = 64 - weight_code->y;
	if (*denominator == 0)
		return (EINVAL);

	g = ceetm_gcd(*numerator, *denominator);
	*numerator /= g;
	*denominator /= g;
	return (0);
}

/* ================================================================
 * Sub-Portal Management
 * ================================================================ */

int
qman_ceetm_sp_claim(struct qm_ceetm_sp **sp, enum qm_dc_portal dcp_idx,
    unsigned int sp_idx)
{
	struct qm_ceetm_sp *p;

	if (dcp_idx >= qman_num_ceetms)
		return (EINVAL);
	if (sp_idx < qman_ceetms[dcp_idx].sp_range[0] ||
	    sp_idx >= qman_ceetms[dcp_idx].sp_range[0] +
	    qman_ceetms[dcp_idx].sp_range[1])
		return (EINVAL);

	TAILQ_FOREACH(p, &qman_ceetms[dcp_idx].sub_portals, entries) {
		if (p->idx == sp_idx) {
			if (p->is_claimed)
				return (EBUSY);
			p->is_claimed = 1;
			*sp = p;
			return (0);
		}
	}
	return (ENOENT);
}

int
qman_ceetm_sp_release(struct qm_ceetm_sp *sp)
{
	if (sp == NULL)
		return (EINVAL);
	if (sp->lni != NULL)
		qman_sp_disable_ceetm_mode(sp->dcp_idx, sp->idx);
	sp->lni = NULL;
	sp->is_claimed = 0;
	return (0);
}

int
qman_ceetm_sp_set_lni(struct qm_ceetm_sp *sp, struct qm_ceetm_lni *lni)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;
	int ret;

	if (sp == NULL || lni == NULL)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_SP_MAPPING | sp->idx);
	cmd.dcpid = sp->dcp_idx;
	cmd.sp_mapping.map_flags = lni->idx & 0x07;

	ret = ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret)
		return (ret);

	sp->lni = lni;
	lni->sp = sp;
	qman_sp_enable_ceetm_mode(sp->dcp_idx, sp->idx);
	return (0);
}

int
qman_ceetm_sp_get_lni(struct qm_ceetm_sp *sp, unsigned int *lni_idx)
{
	if (sp == NULL || sp->lni == NULL)
		return (EINVAL);
	*lni_idx = sp->lni->idx;
	return (0);
}

/* ================================================================
 * LNI Management
 * ================================================================ */

int
qman_ceetm_lni_claim(struct qm_ceetm_lni **lni, enum qm_dc_portal dcp_idx,
    unsigned int lni_idx)
{
	struct qm_ceetm_lni *p;

	if (dcp_idx >= qman_num_ceetms)
		return (EINVAL);

	TAILQ_FOREACH(p, &qman_ceetms[dcp_idx].lnis, entries) {
		if (p->idx == lni_idx) {
			if (p->is_claimed)
				return (EBUSY);
			p->is_claimed = 1;
			*lni = p;
			return (0);
		}
	}
	return (ENOENT);
}

int
qman_ceetm_lni_release(struct qm_ceetm_lni *lni)
{
	if (lni == NULL)
		return (EINVAL);
	if (lni->shaper_enable)
		qman_ceetm_lni_disable_shaper(lni);
	lni->sp = NULL;
	lni->is_claimed = 0;
	return (0);
}

int
qman_ceetm_lni_enable_shaper(struct qm_ceetm_lni *lni, int coupled, int oal)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;
	int ret;

	if (lni == NULL)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	cmd.dcpid = lni->dcp_idx;
	cmd.shaper_config.cpl_oal = (coupled ? 0x80 : 0) | (oal & 0x1F);
	cmd.shaper_config.crtcr_hi = (lni->cr_token_rate.whole >> 8) & 0x07;
	cmd.shaper_config.crtcr_lo = htobe16(
	    ((lni->cr_token_rate.whole & 0xFF) << 13) |
	    (lni->cr_token_rate.fraction & 0x1FFF));
	cmd.shaper_config.ertcr_hi = (lni->er_token_rate.whole >> 8) & 0x07;
	cmd.shaper_config.ertcr_lo = htobe16(
	    ((lni->er_token_rate.whole & 0xFF) << 13) |
	    (lni->er_token_rate.fraction & 0x1FFF));
	cmd.shaper_config.crtbl = htobe16(lni->cr_token_bucket_limit);
	cmd.shaper_config.ertbl = htobe16(lni->er_token_bucket_limit);

	ret = ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret)
		return (ret);

	lni->shaper_enable = 1;
	lni->shaper_couple = coupled;
	lni->oal = oal;
	return (0);
}

int
qman_ceetm_lni_disable_shaper(struct qm_ceetm_lni *lni)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;
	int ret;

	if (lni == NULL)
		return (EINVAL);

	/* Write zero rates to disable shaper */
	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	cmd.dcpid = lni->dcp_idx;

	ret = ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret)
		return (ret);

	lni->shaper_enable = 0;
	return (0);
}

int
qman_ceetm_lni_is_shaper_enabled(struct qm_ceetm_lni *lni)
{
	if (lni == NULL)
		return (0);
	return (lni->shaper_enable);
}

int
qman_ceetm_lni_set_commit_rate(struct qm_ceetm_lni *lni,
    const struct qm_ceetm_rate *cr, uint16_t cr_limit)
{
	if (lni == NULL || cr == NULL)
		return (EINVAL);

	lni->cr_token_rate = *cr;
	lni->cr_token_bucket_limit = cr_limit;

	/* If shaper is already enabled, re-program */
	if (lni->shaper_enable)
		return (qman_ceetm_lni_enable_shaper(lni,
		    lni->shaper_couple, lni->oal));
	return (0);
}

int
qman_ceetm_lni_set_excess_rate(struct qm_ceetm_lni *lni,
    const struct qm_ceetm_rate *er, uint16_t er_limit)
{
	if (lni == NULL || er == NULL)
		return (EINVAL);

	lni->er_token_rate = *er;
	lni->er_token_bucket_limit = er_limit;

	if (lni->shaper_enable)
		return (qman_ceetm_lni_enable_shaper(lni,
		    lni->shaper_couple, lni->oal));
	return (0);
}

int
qman_ceetm_lni_get_commit_rate(struct qm_ceetm_lni *lni,
    struct qm_ceetm_rate *cr, uint16_t *cr_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query qcmd;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query qr;
	uint32_t tcr;
	int ret;

	if (lni == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cid = htobe16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	qcmd.dcpid = lni->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	tcr = ((uint32_t)qr.shaper_query.crtcr_hi << 16) |
	    be16toh(qr.shaper_query.crtcr_lo);
	if (cr != NULL) {
		cr->whole = tcr >> 13;
		cr->fraction = tcr & 0x1FFF;
	}
	if (cr_limit != NULL)
		*cr_limit = be16toh(qr.shaper_query.crtbl);
	return (0);
}

int
qman_ceetm_lni_get_excess_rate(struct qm_ceetm_lni *lni,
    struct qm_ceetm_rate *er, uint16_t *er_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query qcmd;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query qr;
	uint32_t tcr;
	int ret;

	if (lni == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cid = htobe16(CEETM_COMMAND_LNI_SHAPER | lni->idx);
	qcmd.dcpid = lni->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	tcr = ((uint32_t)qr.shaper_query.ertcr_hi << 16) |
	    be16toh(qr.shaper_query.ertcr_lo);
	if (er != NULL) {
		er->whole = tcr >> 13;
		er->fraction = tcr & 0x1FFF;
	}
	if (er_limit != NULL)
		*er_limit = be16toh(qr.shaper_query.ertbl);
	return (0);
}

/* ================================================================
 * Channel Management
 * ================================================================ */

int
qman_ceetm_channel_claim(struct qm_ceetm_channel **channel,
    struct qm_ceetm_lni *lni)
{
	struct qm_ceetm_channel *ch;
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;
	uint32_t ch_id;
	int ret;

	if (lni == NULL || channel == NULL)
		return (EINVAL);

	ret = ceetm_pool_alloc(&ceetm_channel_pool[lni->dcp_idx], &ch_id);
	if (ret)
		return (ret);

	ch = malloc(sizeof(*ch), M_NETCOMMSW, M_NOWAIT | M_ZERO);
	if (ch == NULL) {
		ceetm_pool_free(&ceetm_channel_pool[lni->dcp_idx], ch_id);
		return (ENOMEM);
	}

	ch->idx = ch_id;
	ch->lni = lni;
	ch->lni_idx = lni->idx;
	ch->dcp_idx = lni->dcp_idx;
	TAILQ_INIT(&ch->class_queues);
	TAILQ_INIT(&ch->ccgs);

	/* Map channel to LNI */
	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_CHANNEL_MAPPING | ch->idx);
	cmd.dcpid = ch->dcp_idx;
	cmd.channel_mapping.map_flags = lni->idx & 0x07;

	ret = ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret) {
		ceetm_pool_free(&ceetm_channel_pool[lni->dcp_idx], ch_id);
		free(ch, M_NETCOMMSW);
		return (ret);
	}

	TAILQ_INSERT_TAIL(&lni->channels, ch, entries);
	*channel = ch;
	return (0);
}

int
qman_ceetm_channel_release(struct qm_ceetm_channel *channel)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;

	if (channel == NULL)
		return (EINVAL);

	/* Disable shaper if enabled */
	if (channel->shaper_enable)
		qman_ceetm_channel_disable_shaper(channel);

	/* Clear channel mapping */
	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_CHANNEL_MAPPING | channel->idx);
	cmd.dcpid = channel->dcp_idx;
	ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));

	ceetm_pool_free(&ceetm_channel_pool[channel->dcp_idx], channel->idx);
	if (channel->lni != NULL)
		TAILQ_REMOVE(&channel->lni->channels, channel, entries);
	free(channel, M_NETCOMMSW);
	return (0);
}

int
qman_ceetm_channel_enable_shaper(struct qm_ceetm_channel *channel,
    int coupled)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_CHANNEL_SHAPER | channel->idx);
	cmd.dcpid = channel->dcp_idx;
	cmd.shaper_config.cpl_oal = coupled ? 0x80 : 0;
	cmd.shaper_config.crtcr_hi =
	    (channel->cr_token_rate.whole >> 8) & 0x07;
	cmd.shaper_config.crtcr_lo = htobe16(
	    ((channel->cr_token_rate.whole & 0xFF) << 13) |
	    (channel->cr_token_rate.fraction & 0x1FFF));
	cmd.shaper_config.ertcr_hi =
	    (channel->er_token_rate.whole >> 8) & 0x07;
	cmd.shaper_config.ertcr_lo = htobe16(
	    ((channel->er_token_rate.whole & 0xFF) << 13) |
	    (channel->er_token_rate.fraction & 0x1FFF));
	cmd.shaper_config.crtbl = htobe16(channel->cr_token_bucket_limit);
	cmd.shaper_config.ertbl = htobe16(channel->er_token_bucket_limit);

	ret = ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret)
		return (ret);

	channel->shaper_enable = 1;
	channel->shaper_couple = coupled;
	return (0);
}

int
qman_ceetm_channel_disable_shaper(struct qm_ceetm_channel *channel)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_config cmd;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16(CEETM_COMMAND_CHANNEL_SHAPER | channel->idx);
	cmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret)
		return (ret);

	channel->shaper_enable = 0;
	return (0);
}

int
qman_ceetm_channel_is_shaper_enabled(struct qm_ceetm_channel *channel)
{
	if (channel == NULL)
		return (0);
	return (channel->shaper_enable);
}

int
qman_ceetm_channel_set_commit_rate(struct qm_ceetm_channel *channel,
    const struct qm_ceetm_rate *cr, uint16_t cr_limit)
{
	if (channel == NULL || cr == NULL)
		return (EINVAL);

	channel->cr_token_rate = *cr;
	channel->cr_token_bucket_limit = cr_limit;

	if (channel->shaper_enable)
		return (qman_ceetm_channel_enable_shaper(channel,
		    channel->shaper_couple));
	return (0);
}

int
qman_ceetm_channel_set_excess_rate(struct qm_ceetm_channel *channel,
    const struct qm_ceetm_rate *er, uint16_t er_limit)
{
	if (channel == NULL || er == NULL)
		return (EINVAL);

	channel->er_token_rate = *er;
	channel->er_token_bucket_limit = er_limit;

	if (channel->shaper_enable)
		return (qman_ceetm_channel_enable_shaper(channel,
		    channel->shaper_couple));
	return (0);
}

int
qman_ceetm_channel_get_commit_rate(struct qm_ceetm_channel *channel,
    struct qm_ceetm_rate *cr, uint16_t *cr_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query qcmd;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query qr;
	uint32_t tcr;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cid = htobe16(CEETM_COMMAND_CHANNEL_SHAPER | channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	tcr = ((uint32_t)qr.shaper_query.crtcr_hi << 16) |
	    be16toh(qr.shaper_query.crtcr_lo);
	if (cr != NULL) {
		cr->whole = tcr >> 13;
		cr->fraction = tcr & 0x1FFF;
	}
	if (cr_limit != NULL)
		*cr_limit = be16toh(qr.shaper_query.crtbl);
	return (0);
}

int
qman_ceetm_channel_get_excess_rate(struct qm_ceetm_channel *channel,
    struct qm_ceetm_rate *er, uint16_t *er_limit)
{
	struct qm_mcc_ceetm_mapping_shaper_tcfc_query qcmd;
	struct qm_mcr_ceetm_mapping_shaper_tcfc_query qr;
	uint32_t tcr;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cid = htobe16(CEETM_COMMAND_CHANNEL_SHAPER | channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	tcr = ((uint32_t)qr.shaper_query.ertcr_hi << 16) |
	    be16toh(qr.shaper_query.ertcr_lo);
	if (er != NULL) {
		er->whole = tcr >> 13;
		er->fraction = tcr & 0x1FFF;
	}
	if (er_limit != NULL)
		*er_limit = be16toh(qr.shaper_query.ertbl);
	return (0);
}

/* ================================================================
 * Channel Class Scheduler (Group/Eligibility)
 * ================================================================ */

int
qman_ceetm_channel_set_group(struct qm_ceetm_channel *channel,
    int group_b, unsigned int prio_a, unsigned int prio_b)
{
	struct qm_mcc_ceetm_class_scheduler_config cmd;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqcid = htobe16(channel->idx);
	cmd.dcpid = channel->dcp_idx;
	cmd.gpc = (prio_a & CEETM_GPC_PRIO_A_MASK);
	if (group_b) {
		cmd.gpc |= CEETM_GPC_COMBINE_FLAG;
		cmd.gpc |= (prio_b << CEETM_GPC_PRIO_B_SHIFT) &
		    CEETM_GPC_PRIO_B_MASK;
	}

	ret = ceetm_mc_config(QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG,
	    &cmd, sizeof(cmd));
	return (ret);
}

int
qman_ceetm_channel_get_group(struct qm_ceetm_channel *channel,
    int *group_b, unsigned int *prio_a, unsigned int *prio_b)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	if (group_b != NULL)
		*group_b = (qr.gpc & CEETM_GPC_COMBINE_FLAG) ? 1 : 0;
	if (prio_a != NULL)
		*prio_a = qr.gpc & CEETM_GPC_PRIO_A_MASK;
	if (prio_b != NULL)
		*prio_b = (qr.gpc & CEETM_GPC_PRIO_B_MASK) >>
		    CEETM_GPC_PRIO_B_SHIFT;
	return (0);
}

int
qman_ceetm_channel_set_group_cr_eligibility(
    struct qm_ceetm_channel *channel, int group_b, int cre)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	struct qm_mcc_ceetm_class_scheduler_config cmd;
	uint16_t mask;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	/* Query current state */
	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	mask = be16toh(qr.crem);
	if (group_b) {
		if (cre)
			mask |= (1 << 9);
		else
			mask &= ~(1 << 9);
	} else {
		if (cre)
			mask |= (1 << 8);
		else
			mask &= ~(1 << 8);
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqcid = htobe16(channel->idx);
	cmd.dcpid = channel->dcp_idx;
	cmd.gpc = qr.gpc;
	cmd.crem = htobe16(mask);
	cmd.erem = qr.erem;
	memcpy(cmd.w, qr.w, 8);

	return (ceetm_mc_config(QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG,
	    &cmd, sizeof(cmd)));
}

int
qman_ceetm_channel_set_group_er_eligibility(
    struct qm_ceetm_channel *channel, int group_b, int ere)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	struct qm_mcc_ceetm_class_scheduler_config cmd;
	uint16_t mask;
	int ret;

	if (channel == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	mask = be16toh(qr.erem);
	if (group_b) {
		if (ere)
			mask |= (1 << 9);
		else
			mask &= ~(1 << 9);
	} else {
		if (ere)
			mask |= (1 << 8);
		else
			mask &= ~(1 << 8);
	}

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqcid = htobe16(channel->idx);
	cmd.dcpid = channel->dcp_idx;
	cmd.gpc = qr.gpc;
	cmd.crem = qr.crem;
	cmd.erem = htobe16(mask);
	memcpy(cmd.w, qr.w, 8);

	return (ceetm_mc_config(QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG,
	    &cmd, sizeof(cmd)));
}

int
qman_ceetm_channel_set_cq_cr_eligibility(
    struct qm_ceetm_channel *channel, unsigned int idx, int cre)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	struct qm_mcc_ceetm_class_scheduler_config cmd;
	uint16_t mask;
	int ret;

	if (channel == NULL || idx > 7)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	mask = be16toh(qr.crem);
	if (cre)
		mask |= (1 << idx);
	else
		mask &= ~(1 << idx);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqcid = htobe16(channel->idx);
	cmd.dcpid = channel->dcp_idx;
	cmd.gpc = qr.gpc;
	cmd.crem = htobe16(mask);
	cmd.erem = qr.erem;
	memcpy(cmd.w, qr.w, 8);

	return (ceetm_mc_config(QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG,
	    &cmd, sizeof(cmd)));
}

int
qman_ceetm_channel_set_cq_er_eligibility(
    struct qm_ceetm_channel *channel, unsigned int idx, int ere)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	struct qm_mcc_ceetm_class_scheduler_config cmd;
	uint16_t mask;
	int ret;

	if (channel == NULL || idx > 7)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(channel->idx);
	qcmd.dcpid = channel->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	mask = be16toh(qr.erem);
	if (ere)
		mask |= (1 << idx);
	else
		mask &= ~(1 << idx);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqcid = htobe16(channel->idx);
	cmd.dcpid = channel->dcp_idx;
	cmd.gpc = qr.gpc;
	cmd.crem = qr.crem;
	cmd.erem = htobe16(mask);
	memcpy(cmd.w, qr.w, 8);

	return (ceetm_mc_config(QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG,
	    &cmd, sizeof(cmd)));
}

/* ================================================================
 * Class Queue Management
 * ================================================================ */

int
qman_ceetm_cq_claim(struct qm_ceetm_cq **cq,
    struct qm_ceetm_channel *channel, unsigned int idx,
    struct qm_ceetm_ccg *ccg)
{
	struct qm_ceetm_cq *p;
	struct qm_mcc_ceetm_cq_config cmd;
	int ret;

	if (channel == NULL || cq == NULL || idx > 7)
		return (EINVAL);

	/* Check not already claimed */
	TAILQ_FOREACH(p, &channel->class_queues, entries) {
		if (p->idx == idx)
			return (EBUSY);
	}

	p = malloc(sizeof(*p), M_NETCOMMSW, M_NOWAIT | M_ZERO);
	if (p == NULL)
		return (ENOMEM);

	p->idx = idx;
	p->parent = channel;
	p->ccg = ccg;
	TAILQ_INIT(&p->bound_lfqids);

	/* Configure CQ */
	memset(&cmd, 0, sizeof(cmd));
	cmd.cqid = htobe16((channel->idx << 4) | idx);
	cmd.dcpid = channel->dcp_idx;
	if (ccg != NULL)
		cmd.ccgid = htobe16((ccg->parent->idx << 4) | ccg->idx);

	ret = ceetm_mc_config(QM_CEETM_VERB_CQ_CONFIG, &cmd, sizeof(cmd));
	if (ret) {
		free(p, M_NETCOMMSW);
		return (ret);
	}

	TAILQ_INSERT_TAIL(&channel->class_queues, p, entries);
	p->is_claimed = 1;
	*cq = p;
	return (0);
}

int
qman_ceetm_cq_claim_A(struct qm_ceetm_cq **cq,
    struct qm_ceetm_channel *channel, unsigned int idx,
    struct qm_ceetm_ccg *ccg)
{
	/* Group A uses CQ indices 8-15 */
	if (idx < 8 || idx > 15)
		return (EINVAL);
	return (qman_ceetm_cq_claim(cq, channel, idx, ccg));
}

int
qman_ceetm_cq_release(struct qm_ceetm_cq *cq)
{
	if (cq == NULL)
		return (EINVAL);

	TAILQ_REMOVE(&cq->parent->class_queues, cq, entries);
	free(cq, M_NETCOMMSW);
	return (0);
}

int
qman_ceetm_set_queue_weight(struct qm_ceetm_cq *cq,
    struct qm_ceetm_weight_code *weight_code)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	struct qm_mcc_ceetm_class_scheduler_config cmd;
	int ret;
	unsigned int w_idx;

	if (cq == NULL || weight_code == NULL)
		return (EINVAL);
	if (cq->idx > 15)
		return (EINVAL);

	/* Query current scheduler state */
	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(cq->parent->idx);
	qcmd.dcpid = cq->parent->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	/* Weight index: CQ 0-7 → w[0-7], CQ 8-15 → w[0-7] (group B offset) */
	w_idx = cq->idx & 0x07;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqcid = htobe16(cq->parent->idx);
	cmd.dcpid = cq->parent->dcp_idx;
	cmd.gpc = qr.gpc;
	cmd.crem = qr.crem;
	cmd.erem = qr.erem;
	memcpy(cmd.w, qr.w, 8);

	/* Encode weight: w = (y << 3) | x */
	cmd.w[w_idx] = (weight_code->y << 3) | weight_code->x;

	return (ceetm_mc_config(QM_CEETM_VERB_CLASS_SCHEDULER_CONFIG,
	    &cmd, sizeof(cmd)));
}

int
qman_ceetm_get_queue_weight(struct qm_ceetm_cq *cq,
    struct qm_ceetm_weight_code *weight_code)
{
	struct qm_mcc_ceetm_class_scheduler_query qcmd;
	struct qm_mcr_ceetm_class_scheduler_query qr;
	int ret;
	unsigned int w_idx;

	if (cq == NULL || weight_code == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.cqcid = htobe16(cq->parent->idx);
	qcmd.dcpid = cq->parent->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_CLASS_SCHEDULER_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	w_idx = cq->idx & 0x07;
	weight_code->y = (qr.w[w_idx] >> 3) & 0x1F;
	weight_code->x = qr.w[w_idx] & 0x07;
	return (0);
}

/* ================================================================
 * LFQ (Logical Frame Queue) Management
 * ================================================================ */

#define	CEETM_LFQMT_LFQID_MSB	0xF00000
#define	CEETM_LFQMT_LFQID_LSB	0x000FFF

int
qman_ceetm_lfq_claim(struct qm_ceetm_lfq **lfq, struct qm_ceetm_cq *cq)
{
	struct qm_ceetm_lfq *p;
	struct qm_mcc_ceetm_lfqmt_config cmd;
	uint32_t lfqid, lfqid_packed;
	int ret;

	if (cq == NULL || lfq == NULL)
		return (EINVAL);

	ret = ceetm_pool_alloc(&ceetm_lfqid_pool[cq->parent->dcp_idx],
	    &lfqid);
	if (ret)
		return (ret);

	p = malloc(sizeof(*p), M_NETCOMMSW, M_NOWAIT | M_ZERO);
	if (p == NULL) {
		ceetm_pool_free(&ceetm_lfqid_pool[cq->parent->dcp_idx],
		    lfqid);
		return (ENOMEM);
	}

	p->idx = lfqid;
	p->dctidx = (uint16_t)(lfqid & CEETM_LFQMT_LFQID_LSB);
	p->parent = cq;

	/* Configure LFQMT */
	memset(&cmd, 0, sizeof(cmd));
	lfqid_packed = CEETM_LFQMT_LFQID_MSB |
	    ((uint32_t)cq->parent->dcp_idx << 16) |
	    (lfqid & CEETM_LFQMT_LFQID_LSB);
	cmd.lfqid_hi = (lfqid_packed >> 16) & 0xFF;
	cmd.lfqid_lo = htobe16(lfqid_packed & 0xFFFF);
	cmd.cqid = htobe16((cq->parent->idx << 4) | cq->idx);
	cmd.dctidx = htobe16(p->dctidx);

	ret = ceetm_mc_config(QM_CEETM_VERB_LFQMT_CONFIG,
	    &cmd, sizeof(cmd));
	if (ret) {
		ceetm_pool_free(&ceetm_lfqid_pool[cq->parent->dcp_idx],
		    lfqid);
		free(p, M_NETCOMMSW);
		return (ret);
	}

	TAILQ_INSERT_TAIL(&cq->bound_lfqids, p, entries);
	*lfq = p;
	return (0);
}

int
qman_ceetm_lfq_release(struct qm_ceetm_lfq *lfq)
{
	enum qm_dc_portal dcp_idx;

	if (lfq == NULL)
		return (EINVAL);

	dcp_idx = lfq->parent->parent->dcp_idx;
	ceetm_pool_free(&ceetm_lfqid_pool[dcp_idx], lfq->idx);
	TAILQ_REMOVE(&lfq->parent->bound_lfqids, lfq, entries);
	free(lfq, M_NETCOMMSW);
	return (0);
}

int
qman_ceetm_lfq_set_context(struct qm_ceetm_lfq *lfq,
    uint64_t context_a, uint32_t context_b)
{
	struct qm_mcc_ceetm_dct_config cmd;

	if (lfq == NULL)
		return (EINVAL);

	lfq->context_a = context_a;
	lfq->context_b = context_b;

	memset(&cmd, 0, sizeof(cmd));
	cmd.dctidx = htobe16(lfq->dctidx);
	cmd.dcpid = lfq->parent->parent->dcp_idx;
	cmd.context_b = htobe32(context_b);
	cmd.context_a = htobe64(context_a);

	return (ceetm_mc_config(QM_CEETM_VERB_DCT_CONFIG,
	    &cmd, sizeof(cmd)));
}

int
qman_ceetm_lfq_get_context(struct qm_ceetm_lfq *lfq,
    uint64_t *context_a, uint32_t *context_b)
{
	struct qm_mcc_ceetm_dct_query qcmd;
	struct qm_mcr_ceetm_dct_query qr;
	int ret;

	if (lfq == NULL)
		return (EINVAL);

	memset(&qcmd, 0, sizeof(qcmd));
	qcmd.dctidx = htobe16(lfq->dctidx);
	qcmd.dcpid = lfq->parent->parent->dcp_idx;

	ret = ceetm_mc_query(QM_CEETM_VERB_DCT_QUERY,
	    &qcmd, sizeof(qcmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	if (context_a != NULL)
		*context_a = be64toh(qr.context_a);
	if (context_b != NULL)
		*context_b = be32toh(qr.context_b);
	return (0);
}

int
qman_ceetm_create_fq(struct qm_ceetm_lfq *lfq, struct qman_fq *fq)
{
	if (lfq == NULL || fq == NULL)
		return (EINVAL);

	memset(fq, 0, sizeof(*fq));
	fq->fqid = lfq->idx;
	fq->flags = 0;	/* QMAN_FQ_FLAG_NO_MODIFY equivalent */
	return (0);
}

/* ================================================================
 * CCG (Class Congestion Group) Management
 * ================================================================ */

#define	MAX_CCG_IDX	0x000F

int
qman_ceetm_ccg_claim(struct qm_ceetm_ccg **ccg,
    struct qm_ceetm_channel *channel, unsigned int idx,
    void (*cb)(void *, int), void *cb_ctx)
{
	struct qm_ceetm_ccg *p;

	if (channel == NULL || ccg == NULL || idx > MAX_CCG_IDX)
		return (EINVAL);

	/* Check not already claimed */
	TAILQ_FOREACH(p, &channel->ccgs, entries) {
		if (p->idx == idx)
			return (EBUSY);
	}

	p = malloc(sizeof(*p), M_NETCOMMSW, M_NOWAIT | M_ZERO);
	if (p == NULL)
		return (ENOMEM);

	p->idx = idx;
	p->parent = channel;
	TAILQ_INSERT_TAIL(&channel->ccgs, p, entries);
	*ccg = p;
	return (0);
}

int
qman_ceetm_ccg_release(struct qm_ceetm_ccg *ccg)
{
	if (ccg == NULL)
		return (EINVAL);

	TAILQ_REMOVE(&ccg->parent->ccgs, ccg, entries);
	free(ccg, M_NETCOMMSW);
	return (0);
}

int
qman_ceetm_ccg_set(struct qm_ceetm_ccg *ccg, uint16_t we_mask,
    struct qm_mcc_ceetm_ccgr_config *opts)
{
	struct qm_mcc_ceetm_ccgr_config cmd;

	if (ccg == NULL || opts == NULL)
		return (EINVAL);

	memcpy(&cmd, opts, sizeof(cmd));
	cmd.ccgrid = htobe16(CEETM_CCGR_CM_CONFIGURE |
	    (ccg->parent->idx << 4) | ccg->idx);
	cmd.dcpid = ccg->parent->dcp_idx;
	cmd.we_mask = htobe16(we_mask);

	return (ceetm_mc_config(QM_CEETM_VERB_CCGR_CONFIG,
	    &cmd, sizeof(cmd)));
}

int
qman_ceetm_ccg_get_reject_statistics(struct qm_ceetm_ccg *ccg,
    uint32_t flags, uint64_t *frame_count, uint64_t *byte_count)
{
	struct qm_mcc_ceetm_statistics_query_write cmd;
	struct qm_mcr_ceetm_statistics_query qr;
	uint16_t command_type;
	int ret;

	if (ccg == NULL)
		return (EINVAL);

	if (flags & 0x01)
		command_type = CEETM_QUERY_REJECT_CLEAR_STATISTICS;
	else
		command_type = CEETM_QUERY_REJECT_STATISTICS;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16((ccg->parent->idx << 4) | ccg->idx);
	cmd.dcpid = ccg->parent->dcp_idx;
	cmd.ct = command_type;

	ret = ceetm_mc_query(QM_CEETM_VERB_STATISTICS_QUERY_WRITE,
	    &cmd, sizeof(cmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	/* Decode 40-bit frame count and 48-bit byte count */
	if (frame_count != NULL) {
		*frame_count = ((uint64_t)qr.frm_cnt[0] << 32) |
		    ((uint64_t)qr.frm_cnt[1] << 24) |
		    ((uint64_t)qr.frm_cnt[2] << 16) |
		    ((uint64_t)qr.frm_cnt[3] << 8) |
		    (uint64_t)qr.frm_cnt[4];
	}
	if (byte_count != NULL) {
		*byte_count = ((uint64_t)qr.byte_cnt[0] << 40) |
		    ((uint64_t)qr.byte_cnt[1] << 32) |
		    ((uint64_t)qr.byte_cnt[2] << 24) |
		    ((uint64_t)qr.byte_cnt[3] << 16) |
		    ((uint64_t)qr.byte_cnt[4] << 8) |
		    (uint64_t)qr.byte_cnt[5];
	}
	return (0);
}

/* ================================================================
 * Statistics/Query
 * ================================================================ */

int
qman_ceetm_query_cq(unsigned int cqid, unsigned int dcpid,
    struct qm_mcr_ceetm_cq_query *cq_query)
{
	struct qm_mcc_ceetm_cq_query cmd;

	if (cq_query == NULL)
		return (EINVAL);

	memset(&cmd, 0, sizeof(cmd));
	cmd.cqid = htobe16(cqid);
	cmd.dcpid = dcpid;

	return (ceetm_mc_query(QM_CEETM_VERB_CQ_QUERY,
	    &cmd, sizeof(cmd), cq_query, sizeof(*cq_query)));
}

int
qman_ceetm_cq_get_dequeue_statistics(struct qm_ceetm_cq *cq,
    uint32_t flags, uint64_t *frame_count, uint64_t *byte_count)
{
	struct qm_mcc_ceetm_statistics_query_write cmd;
	struct qm_mcr_ceetm_statistics_query qr;
	uint16_t command_type;
	int ret;

	if (cq == NULL)
		return (EINVAL);

	if (flags & 0x01)
		command_type = CEETM_QUERY_DEQUEUE_CLEAR_STATISTICS;
	else
		command_type = CEETM_QUERY_DEQUEUE_STATISTICS;

	memset(&cmd, 0, sizeof(cmd));
	cmd.cid = htobe16((cq->parent->idx << 4) | cq->idx);
	cmd.dcpid = cq->parent->dcp_idx;
	cmd.ct = command_type;

	ret = ceetm_mc_query(QM_CEETM_VERB_STATISTICS_QUERY_WRITE,
	    &cmd, sizeof(cmd), &qr, sizeof(qr));
	if (ret)
		return (ret);

	if (frame_count != NULL) {
		*frame_count = ((uint64_t)qr.frm_cnt[0] << 32) |
		    ((uint64_t)qr.frm_cnt[1] << 24) |
		    ((uint64_t)qr.frm_cnt[2] << 16) |
		    ((uint64_t)qr.frm_cnt[3] << 8) |
		    (uint64_t)qr.frm_cnt[4];
	}
	if (byte_count != NULL) {
		*byte_count = ((uint64_t)qr.byte_cnt[0] << 40) |
		    ((uint64_t)qr.byte_cnt[1] << 32) |
		    ((uint64_t)qr.byte_cnt[2] << 24) |
		    ((uint64_t)qr.byte_cnt[3] << 16) |
		    ((uint64_t)qr.byte_cnt[4] << 8) |
		    (uint64_t)qr.byte_cnt[5];
	}
	return (0);
}

/* ================================================================
 * MAPPING_SHAPER_TCFC direct command
 * ================================================================ */

int
qman_ceetm_configure_mapping_shaper_tcfc(
    struct qm_mcc_ceetm_mapping_shaper_tcfc_config *opts)
{
	if (opts == NULL)
		return (EINVAL);
	return (ceetm_mc_config(QM_CEETM_VERB_MAPPING_SHAPER_TCFC_CONFIG,
	    opts, sizeof(*opts)));
}

/* ================================================================
 * Initialization — called from qman_attach() if DT node present
 * ================================================================ */

int
qman_ceetm_init(void)
{
	phandle_t qman_node, ceetm_node, child;
	pcell_t range[2];
	enum qm_dc_portal dcp_portal;
	struct qm_ceetm_sp *sp;
	struct qm_ceetm_lni *lni;
	int ret, i, found;

	mtx_init(&ceetm_alloc_lock, "ceetm_alloc", NULL, MTX_DEF);
	qman_num_ceetms = 0;

	/* Find the qman node */
	qman_node = ofw_bus_get_node(qman_sc->sc_dev);
	if (qman_node <= 0)
		return (0);	/* No DT node — CEETM not available */

	/*
	 * Get QMan clock frequency.
	 * Try "clock-frequency" on qman node first, then fall back to
	 * the soc node's "bus-frequency", then default to 400MHz.
	 */
	qman_clk = 0;
	if (OF_getencprop(qman_node, "clock-frequency",
	    (void *)&qman_clk, sizeof(qman_clk)) <= 0) {
		phandle_t soc_node;

		soc_node = OF_finddevice("/soc");
		if (soc_node > 0)
			OF_getencprop(soc_node, "bus-frequency",
			    (void *)&qman_clk, sizeof(qman_clk));
	}
	if (qman_clk == 0) {
		printf("ceetm: no clock-frequency, defaulting to 400MHz\n");
		qman_clk = 400000000;
	}
	/*
	 * Search for fsl,qman-ceetm child nodes under the qman-portals
	 * parent, or directly under qman.
	 * The DT layout varies — check all children of the qman portals
	 * node and the qman node itself.
	 */
	found = 0;

	/* Search children of qman node */
	for (child = OF_child(qman_node); child > 0; child = OF_peer(child)) {
		if (!ofw_bus_node_is_compatible(child, "fsl,qman-ceetm"))
			continue;
		if (!ofw_bus_node_status_okay(child))
			continue;

		/* Parse LFQID range — extract DCP portal index */
		if (OF_getencprop(child, "fsl,ceetm-lfqid-range",
		    (void *)range, sizeof(range)) != sizeof(range)) {
			printf("ceetm: missing fsl,ceetm-lfqid-range\n");
			continue;
		}

		dcp_portal = (range[0] & 0x0F0000) >> 16;
		if (dcp_portal >= QMAN_CEETM_MAX) {
			printf("ceetm: DCP portal %d out of range\n",
			    dcp_portal);
			continue;
		}

		/* Seed LFQID allocator */
		ceetm_pool_seed(&ceetm_lfqid_pool[dcp_portal],
		    range[0], range[1]);

		/* Initialize CEETM instance */
		qman_ceetms[dcp_portal].idx = dcp_portal;
		TAILQ_INIT(&qman_ceetms[dcp_portal].sub_portals);
		TAILQ_INIT(&qman_ceetms[dcp_portal].lnis);

		/* Parse SP range */
		if (OF_getencprop(child, "fsl,ceetm-sp-range",
		    (void *)range, sizeof(range)) != sizeof(range)) {
			printf("ceetm: missing fsl,ceetm-sp-range\n");
			continue;
		}
		qman_ceetms[dcp_portal].sp_range[0] = range[0];
		qman_ceetms[dcp_portal].sp_range[1] = range[1];

		for (i = 0; i < (int)range[1]; i++) {
			sp = malloc(sizeof(*sp), M_NETCOMMSW,
			    M_WAITOK | M_ZERO);
			sp->idx = range[0] + i;
			sp->dcp_idx = dcp_portal;
			TAILQ_INSERT_TAIL(
			    &qman_ceetms[dcp_portal].sub_portals,
			    sp, entries);
		}

		/* Parse LNI range */
		if (OF_getencprop(child, "fsl,ceetm-lni-range",
		    (void *)range, sizeof(range)) != sizeof(range)) {
			printf("ceetm: missing fsl,ceetm-lni-range\n");
			continue;
		}
		qman_ceetms[dcp_portal].lni_range[0] = range[0];
		qman_ceetms[dcp_portal].lni_range[1] = range[1];

		for (i = 0; i < (int)range[1]; i++) {
			lni = malloc(sizeof(*lni), M_NETCOMMSW,
			    M_WAITOK | M_ZERO);
			lni->idx = range[0] + i;
			lni->dcp_idx = dcp_portal;
			TAILQ_INIT(&lni->channels);
			TAILQ_INSERT_TAIL(
			    &qman_ceetms[dcp_portal].lnis,
			    lni, entries);
		}

		/* Parse channel range */
		if (OF_getencprop(child, "fsl,ceetm-channel-range",
		    (void *)range, sizeof(range)) != sizeof(range)) {
			printf("ceetm: missing fsl,ceetm-channel-range\n");
			continue;
		}
		ceetm_pool_seed(&ceetm_channel_pool[dcp_portal],
		    range[0], range[1]);

		/* Set prescaler */
		ret = qman_ceetm_set_prescaler(dcp_portal);
		if (ret) {
			printf("ceetm: failed to set prescaler for DCP%d\n",
			    dcp_portal);
			continue;
		}

		if (dcp_portal >= qman_num_ceetms)
			qman_num_ceetms = dcp_portal + 1;
		found++;

		printf("ceetm: DCP%d initialized: %d SPs, %d LNIs, "
		    "clk=%uMHz\n",
		    dcp_portal,
		    qman_ceetms[dcp_portal].sp_range[1],
		    qman_ceetms[dcp_portal].lni_range[1],
		    qman_clk / 1000000);
	}

	if (found == 0) {
		/* Also search under qman-portals parent node */
		ceetm_node = OF_finddevice("/soc/fsl,dpaa/fsl,qman-portals");
		if (ceetm_node > 0) {
			for (child = OF_child(ceetm_node); child > 0;
			    child = OF_peer(child)) {
				if (ofw_bus_node_is_compatible(child,
				    "fsl,qman-ceetm"))
					printf("ceetm: found CEETM under "
					    "qman-portals, not yet "
					    "supported — add to qman "
					    "node instead\n");
			}
		}
	}

	if (qman_num_ceetms > 0)
		printf("ceetm: %d CEETM instance%s available\n",
		    qman_num_ceetms,
		    qman_num_ceetms > 1 ? "s" : "");

	return (0);
}
