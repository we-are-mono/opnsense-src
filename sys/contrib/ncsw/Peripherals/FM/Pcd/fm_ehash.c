/*
 * fm_ehash.c — FMan enhanced external hash table (kernel-side)
 *
 * Ported from:
 *   linux/drivers/net/ethernet/freescale/sdk_fman/Peripherals/FM/Pcd/fm_ehash.c
 *
 * Implements the ExternalHashTable* API used by CDX for hardware flow
 * offload.  Manages DDR-based hash tables with CRC64-indexed buckets
 * and per-bucket spinlocks.
 *
 * With USE_ENHANCED_EHASH, FM_PCD_HashTableSet (in fm_cc.c) redirects
 * to ExternalHashTableSet here.  The returned handle (en_exthash_info*)
 * is opaque to all callers (fmlib, FMC, dpa_app, CDX).
 *
 * The MURAM Action Descriptor is NOT allocated here.  copy_td_to_ccbase()
 * writes the AD content from info->node into the CC tree's MURAM slot
 * during FM_PCD_CcRootBuild, and saves the MURAM address to info->h_Ad.
 * ExternalHashTableModifyMissNextEngine then uses info->h_Ad to patch
 * the miss action in-place.
 *
 * Simplifications for initial port:
 *   - NO_CUMULATIVE_ENTRY: simple linked-list buckets, no compaction
 *   - No IP reassembly table support (table_type REASSM is accepted
 *     but reassembly-specific params are ignored)
 *
 * Copyright 2018 NXP (original Linux SDK code)
 * Copyright (c) 2026 Mono Technologies Inc.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "opt_dpaa.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/endian.h>

#include <vm/vm.h>
#include <machine/cpufunc.h>

/* NCSW public headers (resolved via -I paths in DPAA_COMPILE_CMD) */
#include "ncsw_ext.h"
#include "error_ext.h"
#include "endian_ext.h"
#include "xx_ext.h"
#include "fm_pcd_ext.h"
#include "fm_muram_ext.h"

/* NCSW internal headers — needed for t_FmPcd, FmPcdGetMuramHandle, etc. */
#include "fm_pcd.h"
#include "fm_cc.h"
#include "fm_common.h"

/* CRC64 from NCSW PCD */
#include "crc64.h"

/* CDX fm_ehash.h — entry/bucket structs and opcode constants */
#include "fm_ehash.h"

/* Linux endian compat — guarded against LinuxKPI which defines them too */
#ifndef cpu_to_be16
#define cpu_to_be16(x)	htobe16(x)
#endif
#ifndef cpu_to_be32
#define cpu_to_be32(x)	htobe32(x)
#endif
#ifndef be64_to_cpu
#define be64_to_cpu(x)	be64toh(x)
#endif

/* ----------------------------------------------------------------
 * Direct MURAM accessors — batch-safe volatile stores/loads
 *
 * MURAM on LS1046A ARM64 requires that multiple stores to the same
 * 16-byte region are batched WITHOUT a dsb sy between each store.
 * A dsb sy after each store causes all but the last to be lost
 * (confirmed empirically: raw batch stores + single dsb work,
 * per-store dsb does not).
 *
 * muram_wr32 does a raw volatile store with bswap, NO barrier.
 * muram_rd32 does a raw volatile load with bswap + read barrier.
 * muram_barrier() issues a single dsb sy — call ONCE after all
 * stores in a batch are complete.
 * ---------------------------------------------------------------- */
#define	muram_barrier()	__asm __volatile("dsb sy" ::: "memory")

static __inline void
muram_wr32(volatile uint32_t *addr, uint32_t native_val)
{

	*addr = __builtin_bswap32(native_val);
	/* NO barrier here — caller must call muram_barrier() after batch */
}

/*
 * muram_wr64 — write two native uint32_t values as a single 64-bit store.
 *
 * A 64-bit store is a single AXI bus transaction, which the MURAM
 * controller handles atomically.  This avoids the partial-write
 * corruption seen with two separate 32-bit stores when FMan microcode
 * concurrently reads the same 16-byte AD region.
 *
 * On LE ARM64, a 64-bit store writes:
 *   bits[31:0]  → bytes at addr+0  (native_lo, byte-swapped)
 *   bits[63:32] → bytes at addr+4  (native_hi, byte-swapped)
 */
static __inline void
muram_wr64(volatile uint64_t *addr, uint32_t native_lo, uint32_t native_hi)
{

	*addr = ((uint64_t)__builtin_bswap32(native_hi) << 32) |
	    (uint64_t)__builtin_bswap32(native_lo);
	/* NO barrier here — caller must call muram_barrier() after batch */
}

static __inline uint32_t
muram_rd32(volatile uint32_t *addr)
{
	uint32_t v;

	v = *addr;
	__asm __volatile("dsb sy" ::: "memory");
	return (__builtin_bswap32(v));
}

/* ----------------------------------------------------------------
 * Internal definitions (supplements fm_ehash.h)
 * ---------------------------------------------------------------- */

/* Miss action types for en_exthash_node */
#define EN_EHASH_MISS_ACTION_DROP	3
#define EN_EHASH_MISS_ACTION_NIA	1
#define EN_EHASH_MISS_ACTION_ENQUE	2
#define EN_EHASH_MISS_ACTION_DONE	0

/* Internal buffer pool size and table alignment */
#define EN_INTERNAL_BUFF_POOL_SIZE	(256 * 128)
#define EN_EXTHASH_TBL_ALIGNMENT	256

/*
 * Hardware table_type constants for en_exthash_node word_0 bits [23:20]
 * (LE ARM64 C bitfield position).
 * The CDX microcode uses table_type to identify the hash table's
 * protocol layer.  Without a valid table_type, the microcode does
 * not recognize the AD as an enhanced hash table entry.
 */
#define	EHASH_HW_TABLE_TYPE_L2		(1 << 0)	/* 0x1 */
#define	EHASH_HW_TABLE_TYPE_L3		(1 << 1)	/* 0x2 */
#define	EHASH_HW_TABLE_TYPE_L4		(1 << 2)	/* 0x4 */
#define	EHASH_HW_TABLE_TYPE_REASSM	(1 << 3)	/* 0x8 */

/*
 * Convert user-facing table_type enum (from dpa_app/fmlib) to
 * hardware table_type constant.  Enum values match fm_eh_types.h.
 */
static uint32_t
ehash_hw_table_type(uint32_t user_type)
{

	switch (user_type & 0xf) {
	case 0:		/* IPV4_UDP_TABLE */
	case 1:		/* IPV4_TCP_TABLE */
	case 2:		/* IPV6_UDP_TABLE */
	case 3:		/* IPV6_TCP_TABLE */
	case 4:		/* ESP_IPV4_TABLE */
	case 5:		/* ESP_IPV6_TABLE */
	case 10:	/* IPV4_3TUPLE_UDP_TABLE */
	case 11:	/* IPV4_3TUPLE_TCP_TABLE */
	case 12:	/* IPV6_3TUPLE_UDP_TABLE */
	case 13:	/* IPV6_3TUPLE_TCP_TABLE */
		return (EHASH_HW_TABLE_TYPE_L4);
	case 6:		/* IPV4_MULTICAST_TABLE */
	case 7:		/* IPV6_MULTICAST_TABLE */
		return (EHASH_HW_TABLE_TYPE_L3);
	case 14:	/* IPV4_REASSM_TABLE */
	case 15:	/* IPV6_REASSM_TABLE */
		return (EHASH_HW_TABLE_TYPE_REASSM);
	default:	/* PPPOE_RELAY_TABLE, ETHERNET_TABLE, etc. */
		return (EHASH_HW_TABLE_TYPE_L2);
	}
}

/* Flags for en_exthash_info.flags */
#define TIMESTAMP_EN	(1 << 0)
#define STATS_EN	(1 << 1)

/*
 * en_exthash_node — Action Descriptor for enhanced external hash table.
 * Written to MURAM (via copy_td_to_ccbase); read by FMan CDX microcode.
 *
 * The Linux SDK (fm_ehash.h line 563) defines this with C bitfield unions
 * "assuming a LE GPP core".  On LE ARM64, the first declared struct field
 * occupies the LSB.  muram_wr32/muram_wr64 byte-swap the native value
 * so FMan (always BE) reads the correct numeric value.
 *
 * CONFIRMED: LE ARM64 C bitfield positions are correct.  Attempting
 * PPC/BE positions (build #194) causes FMan exception 5 (crash).
 *
 * Non-EXCLUDE_FMAN_IPR_OFFLOAD layout (active on LS1046A):
 *
 * word_0 — LE ARM64 C bitfield positions (first field at LSB):
 *   bits  7:0  = table_base_hi (8)     — upper bits of 40-bit table addr
 *   bits 15:8  = ipv4_ad_offset (8)    — IPR reassembly AD offset (0)
 *   bits 18:16 = hash_bytes_offset (3) — CRC64 shift parameter
 *   bit  19    = reserved (1)
 *   bits 23:20 = table_type (4)        — L2/L3/L4/REASSM identifier
 *   bits 29:24 = key_size (6)
 *   bits 31:30 = miss_action_type (2)
 *
 * word_1 — LE ARM64 C bitfield positions:
 *   bits  3:0  = hash_mask_bits (4)
 *   bits 15:4  = global_mem_offset (12)
 *   bits 31:16 = int_buf_pool_addr (16)
 *
 * word_2: nia or fqid depending on miss_action_type
 */
struct en_exthash_node {
	uint32_t word_0;
	uint32_t table_base_lo;
	uint32_t word_1;
	uint32_t word_2;		/* nia or fqid */
} __attribute__((packed));

/* word_0 field construction — LE ARM64 C bitfield positions.
 *
 * On LE ARM64, C bitfields place the first declared field at bit 0 (LSB).
 * muram_wr32/muram_wr64 byte-swap the native value for MURAM storage.
 *
 * Non-EXCLUDE_FMAN_IPR_OFFLOAD variant (LS1046A). */
#define	EHASH_W0_TABLE_BASE_HI(v)	((uint32_t)(v) & 0xFF)		   /* bits  7:0  */
#define	EHASH_W0_IPV4_AD_OFF(v)		(((uint32_t)(v) & 0xFF) << 8)	   /* bits 15:8  */
#define	EHASH_W0_HASH_BYTES_OFF(v)	(((uint32_t)(v) & 0x7) << 16)	   /* bits 18:16 */
#define	EHASH_W0_TABLE_TYPE(v)		(((uint32_t)(v) & 0xF) << 20)	   /* bits 23:20 */
#define	EHASH_W0_KEY_SIZE(v)		(((uint32_t)(v) & 0x3F) << 24)	   /* bits 29:24 */
#define	EHASH_W0_MISS_ACTION(v)		(((uint32_t)(v) & 0x3) << 30)	   /* bits 31:30 */
#define	EHASH_W0_MISS_ACTION_MASK	(0x3U << 30)			   /* bits 31:30 */

/* word_1 field construction — LE ARM64 C bitfield positions.
 * Non-EXCLUDE variant: hash_mask_bits at LSB, then
 * global_mem_offset, then int_buf_pool_addr at MSB. */
#define	EHASH_W1_HASH_MASK_BITS(v)	((uint32_t)(v) & 0xF)		   /* bits  3:0  */
#define	EHASH_W1_GLOBAL_MEM_OFF(v)	(((uint32_t)(v) & 0xFFF) << 4)	   /* bits 15:4  */
#define	EHASH_W1_INT_BUF_POOL(v)	(((uint32_t)(v) & 0xFFFF) << 16)   /* bits 31:16 */

/*
 * en_exthash_info — Per-table software state.
 * Returned as opaque t_Handle to callers.
 */
struct en_exthash_info {
	uint32_t	flags;
	void		*table_base;	/* DDR bucket array */
	void		**pSpinlock;	/* per-bucket spinlock array */
	void		*h_Ad;		/* MURAM AD pointer (set by copy_td_to_ccbase) */
	struct en_exthash_node node;	/* local copy of table descriptor */
	uint32_t	tablesize;
	uint32_t	hashmask;
	uint32_t	keysize;
	uint32_t	hashshift;
	uint32_t	dataMemId;
	uint32_t	dataLiodnOffset;
	uint32_t	num_keys;
	uint32_t	max_collisions;
	void		*pcd;		/* PCD handle */
	uint32_t	table_type;	/* user-facing type (L2/L3/L4/REASSM) */
	struct ip_reassembly_params *ip_reassem_info;	/* MURAM VA, REASSM only */
};

MALLOC_DEFINE(M_EHASH, "ehash", "CDX external hash table");

/* ----------------------------------------------------------------
 * Internal buffer pool management — allocated once in MURAM
 * ---------------------------------------------------------------- */

static void		*ehash_int_muram_ptr;	/* virtual address */
static uint32_t		 ehash_int_muram_area;	/* MURAM offset >> 8 */
static void		*ehash_first_table_base;/* DDR base of first table */

/* Global MURAM data area — dscp_vlanpcp_map + SEC_failure_stats.
 * Points to pool + EN_INTERNAL_BUFF_POOL_SIZE.  Initialized in
 * ehash_init_muram_pool(). */
static en_exthash_global_mem	*ehash_global_mem;

/* Reassembly table tracking — set during ExternalHashTableSet()
 * for IPV4_REASSM_TABLE (14) and IPV6_REASSM_TABLE (15). */
#define	EHASH_IPV4_REASSM_TABLE	14
#define	EHASH_IPV6_REASSM_TABLE	15

static struct en_exthash_info	*ehash_ipv4_reassly_tbl;
static struct en_exthash_info	*ehash_ipv6_reassly_tbl;

/* ----------------------------------------------------------------
 * External timestamp slots — separate MURAM allocation
 *
 * Timestamps MUST be in their own MURAM region, NOT in the global
 * data area.  The global data area (at int_buf_pool + POOL_SIZE)
 * contains en_exthash_global_mem (dscp_vlanpcp_map + SEC_failure_stats)
 * which the CDX microcode reads/writes at runtime.
 *
 * Linux SDK allocates timestamps separately in FM_PCD_Init()
 * (fm_pcd.c: FM_MURAM_AllocMem for MAX_EXT_TS_TIMERS * EXT_TS_SIZE).
 * ---------------------------------------------------------------- */

#define	MAX_EXT_TS_TIMERS	4

static uint32_t		*ehash_ts_virt;		/* virtual ptr to slot[0] */
static uint64_t		 ehash_ts_phys;		/* physical addr of slot[0] */
static uint64_t		 ehash_muram_phys_base;	/* MURAM physical base */

/*
 * Allocate the internal buffer pool + global data area in MURAM,
 * plus a SEPARATE timestamp region.
 * Called once on first ExternalHashTableSet.
 */
static int
ehash_init_muram_pool(t_Handle h_FmPcd)
{
	t_FmPcd *p_FmPcd = (t_FmPcd *)h_FmPcd;
	t_Handle h_FmMuram;
	uint32_t total_size;
	uint32_t muram_offset;
	uint32_t ts_size;

	if (ehash_int_muram_ptr != NULL)
		return (0);

	h_FmMuram = FmPcdGetMuramHandle(h_FmPcd);
	if (h_FmMuram == NULL)
		return (-1);

	/*
	 * Layout of the pool+global allocation:
	 *   [0 .. EN_INTERNAL_BUFF_POOL_SIZE-1]  internal buffer pool
	 *   [EN_INTERNAL_BUFF_POOL_SIZE .. +256]  global data area
	 *     (en_exthash_global_mem: dscp_vlanpcp_map + SEC_failure_stats)
	 *
	 * Timestamps are allocated SEPARATELY below.
	 */
	total_size = EN_INTERNAL_BUFF_POOL_SIZE + 256;

	ehash_int_muram_ptr = FM_MURAM_AllocMem(h_FmMuram,
	    total_size, EN_EXTHASH_TBL_ALIGNMENT);
	if (ehash_int_muram_ptr == NULL) {
		printf("fm_ehash: MURAM alloc failed (%u bytes)\n",
		    total_size);
		return (-1);
	}
	IOMemSet32(ehash_int_muram_ptr, 0, total_size);

	/* Compute compressed MURAM offset (physical offset / 256) */
	muram_offset = (uint32_t)(XX_VirtToPhys(ehash_int_muram_ptr) -
	    p_FmPcd->physicalMuramBase);
	if (muram_offset & 0xff) {
		printf("fm_ehash: MURAM pool not 256-aligned (0x%x)\n",
		    muram_offset);
		FM_MURAM_FreeMem(h_FmMuram, ehash_int_muram_ptr);
		ehash_int_muram_ptr = NULL;
		return (-1);
	}

	ehash_int_muram_area = muram_offset >> 8;

	ehash_muram_phys_base = p_FmPcd->physicalMuramBase;

	printf("fm_ehash: MURAM pool at %p, offset 0x%x (compressed 0x%x)\n",
	    ehash_int_muram_ptr, muram_offset, ehash_int_muram_area);
	ehash_global_mem = (en_exthash_global_mem *)((uint8_t *)ehash_int_muram_ptr +
	    EN_INTERNAL_BUFF_POOL_SIZE);

	printf("fm_ehash: global data at pool+0x%x (en_exthash_global_mem)\n",
	    EN_INTERNAL_BUFF_POOL_SIZE);

	/*
	 * Allocate timestamp slots as a SEPARATE MURAM region.
	 * Linux SDK does this in FM_PCD_Init() via FM_MURAM_AllocMem.
	 * These must NOT overlap the global data area.
	 */
	ts_size = MAX_EXT_TS_TIMERS * sizeof(uint32_t);
	ehash_ts_virt = FM_MURAM_AllocMem(h_FmMuram,
	    ts_size, sizeof(uint32_t));
	if (ehash_ts_virt == NULL) {
		printf("fm_ehash: MURAM timestamp alloc failed (%u bytes)\n",
		    ts_size);
		FM_MURAM_FreeMem(h_FmMuram, ehash_int_muram_ptr);
		ehash_int_muram_ptr = NULL;
		return (-1);
	}
	IOMemSet32(ehash_ts_virt, 0, ts_size);
	ehash_ts_phys = XX_VirtToPhys(ehash_ts_virt);

	printf("fm_ehash: timestamp slots at %p, phys 0x%jx "
	    "(MURAM offset 0x%jx)\n",
	    ehash_ts_virt, (uintmax_t)ehash_ts_phys,
	    (uintmax_t)(ehash_ts_phys - ehash_muram_phys_base));

	return (0);
}

/* ----------------------------------------------------------------
 * CRC64-based hash bucket indexing
 *
 * Ported from sdk_fman fm_cc.c:get_indexed_hash_bucket()
 * ---------------------------------------------------------------- */

static void
get_indexed_hash_bucket(uint8_t key_size, uint8_t *key_ptr,
    uint8_t crc_shift, uint16_t mask, uint16_t *bucket_index)
{
	uint64_t crc64;

	crc64 = crc64_init();
	crc64 = crc64_compute(key_ptr, key_size, crc64);
	crc64 >>= ((6 - crc_shift) << 3);
	*bucket_index = (uint16_t)crc64 & mask;
}

/* ----------------------------------------------------------------
 * Bucket chain search (NO_CUMULATIVE_ENTRY path)
 * ---------------------------------------------------------------- */

static struct en_exthash_tbl_entry *
find_entry_in_bucket(struct en_exthash_tbl_entry *entry,
    uint8_t *key, uint32_t size)
{

	while (entry != NULL) {
		if (memcmp(key, &entry->hashentry.key[0], size) == 0)
			return (entry);
		entry = entry->next;
	}
	return (NULL);
}

/* ----------------------------------------------------------------
 * Cleanup helper
 * ---------------------------------------------------------------- */

static void
delete_ehash_info(struct en_exthash_info *info)
{
	uint32_t ii;

	if (info == NULL)
		return;

	if (info->pSpinlock != NULL) {
		for (ii = 0; ii <= info->hashmask; ii++) {
			if (*(info->pSpinlock + ii) != NULL)
				XX_FreeSpinlock(*(info->pSpinlock + ii));
			else
				break;
		}
		free(info->pSpinlock, M_EHASH);
	}

	if (info->table_base != NULL)
		contigfree(info->table_base, info->tablesize, M_EHASH);

	/*
	 * Do NOT free h_Ad — it points into the CC tree's MURAM slot
	 * (set by copy_td_to_ccbase).  The CC tree owns that MURAM.
	 */

	free(info, M_EHASH);
}


/* ================================================================
 * Public API
 * ================================================================ */

/*
 * ExternalHashTableSet — Create an enhanced external hash table.
 *
 * Called from FM_PCD_HashTableSet (via USE_ENHANCED_EHASH redirect).
 * Allocates DDR bucket array and per-bucket spinlocks.  Builds the
 * AD node data in info->node but does NOT write it to MURAM.
 * copy_td_to_ccbase() (called during FM_PCD_CcRootBuild) writes
 * the AD into the CC tree's MURAM entry and sets info->h_Ad.
 *
 * Returns opaque handle on success, NULL on failure.
 */
t_Handle
ExternalHashTableSet(t_Handle h_FmPcd, t_FmPcdHashTableParams *p_Param)
{
	struct en_exthash_info *info;
	uint32_t ii;
	struct en_exthash_node *node;
	uint64_t tblphysaddr;
	uint32_t num_bits;

	if (ehash_init_muram_pool(h_FmPcd) != 0)
		return (NULL);

	info = malloc(sizeof(*info), M_EHASH, M_WAITOK | M_ZERO);
	if (info == NULL) {
		REPORT_ERROR(MAJOR, E_NO_MEMORY,
		    ("en_exthash_info allocation"));
		return (NULL);
	}

	info->hashmask = p_Param->hashResMask;
	info->pcd = h_FmPcd;

	/*
	 * Validate hash mask: must be (2^N - 1) where N <= 15.
	 */
	if (info->hashmask > 0x7fff) {
		REPORT_ERROR(MAJOR, E_INVALID_VALUE,
		    ("unsupported hash mask value"));
		goto err_ret;
	}

	num_bits = 0;
	for (ii = 0; ii < 15; ii++) {
		if ((info->hashmask + 1) & (1U << ii))
			break;
	}
	num_bits = ii;

	if ((1U << num_bits) != (info->hashmask + 1)) {
		REPORT_ERROR(MAJOR, E_INVALID_VALUE,
		    ("hash mask not power-of-2 minus 1"));
		goto err_ret;
	}

	/* Allocate per-bucket spinlocks (CPU-only, no DMA) */
	info->pSpinlock = malloc(
	    sizeof(t_Handle) * (info->hashmask + 1),
	    M_EHASH, M_WAITOK | M_ZERO);
	if (info->pSpinlock == NULL) {
		REPORT_ERROR(MAJOR, E_NO_MEMORY, ("spinlock array"));
		goto err_ret;
	}

	for (ii = 0; ii <= info->hashmask; ii++) {
		*(info->pSpinlock + ii) = XX_InitSpinlock();
		if (*(info->pSpinlock + ii) == NULL) {
			REPORT_ERROR(MAJOR, E_NO_MEMORY,
			    ("bucket spinlock"));
			goto err_ret;
		}
	}

	/*
	 * Allocate DDR hash table (bucket array).
	 * Must be physically contiguous — FMan microcode accesses
	 * buckets via table_base_phys + index * sizeof(bucket).
	 * Use contigmalloc directly instead of the shared NCSW
	 * MallocSmart pool, which is sized for dTSEC buffers only.
	 */
	info->tablesize = (uint32_t)(sizeof(struct en_exthash_bucket) *
	    (info->hashmask + 1));
	info->table_base = contigmalloc(info->tablesize, M_EHASH,
	    M_WAITOK | M_ZERO, 0, (vm_paddr_t)0xFFFFFFFF,
	    EN_EXTHASH_TBL_ALIGNMENT, 0);
	if (info->table_base == NULL) {
		REPORT_ERROR(MAJOR, E_NO_MEMORY,
		    ("hash table buckets"));
		goto err_ret;
	}

	/* Save first table base for diagnostic DDR bucket verification */
	if (ehash_first_table_base == NULL)
		ehash_first_table_base = info->table_base;

	/* Fill table parameters */
	info->keysize = p_Param->matchKeySize;
	info->hashshift = p_Param->hashShift;
	info->flags |= TIMESTAMP_EN;
	if (p_Param->statisticsMode)
		info->flags |= STATS_EN;

	/* Build AD node in local copy — NOT written to MURAM yet.
	 * copy_td_to_ccbase() writes this into the CC tree MURAM slot. */
	node = &info->node;
	memset(node, 0, sizeof(*node));

	tblphysaddr = XX_VirtToPhys(info->table_base);

	{
		uint32_t miss_action;
		uint32_t hw_table_type;

		/* Convert user-facing table_type enum to hardware constant.
		 * p_Param->table_type is populated by dpa_app via fmlib. */
		hw_table_type = ehash_hw_table_type(p_Param->table_type);

		switch (p_Param->ccNextEngineParamsForMiss.nextEngine) {
		case e_FM_PCD_KG: {
			t_FmPcdCcNextKgParams *kgparams =
			    &p_Param->ccNextEngineParamsForMiss.params.kgParams;

			node->word_2 = NIA_ENG_KG | NIA_KG_DIRECT;
			if (kgparams->overrideFqid)
				node->word_2 |= kgparams->newFqid |
				    NIA_KG_CC_EN;
			node->word_2 |=
			    FmPcdKgGetSchemeId(kgparams->h_DirectScheme);
			miss_action = EN_EHASH_MISS_ACTION_NIA;
			break;
		}
		case e_FM_PCD_DONE: {
			t_FmPcdCcNextEnqueueParams *enqparams =
			    &p_Param->ccNextEngineParamsForMiss.params
			    .enqueueParams;

			if (enqparams->overrideFqid) {
				node->word_2 = enqparams->newFqid;
				miss_action = EN_EHASH_MISS_ACTION_ENQUE;
			} else {
				miss_action = EN_EHASH_MISS_ACTION_DONE;
			}
			break;
		}
		default:
			miss_action = EN_EHASH_MISS_ACTION_DROP;
			break;
		}

		/* Build word_0 — non-EXCLUDE layout matching CDX microcode.
		 * ipv4_ad_offset = 0 (no IPR reassembly support). */
		node->word_0 =
		    EHASH_W0_TABLE_BASE_HI((tblphysaddr >> 32) & 0xff) |
		    EHASH_W0_IPV4_AD_OFF(0) |
		    EHASH_W0_HASH_BYTES_OFF(info->hashshift) |
		    EHASH_W0_TABLE_TYPE(hw_table_type) |
		    EHASH_W0_KEY_SIZE(info->keysize) |
		    EHASH_W0_MISS_ACTION(miss_action);
		node->table_base_lo = tblphysaddr & 0xffffffff;
		node->word_1 =
		    EHASH_W1_HASH_MASK_BITS(num_bits) |
		    EHASH_W1_GLOBAL_MEM_OFF(EN_INTERNAL_BUFF_POOL_SIZE >> 8) |
		    EHASH_W1_INT_BUF_POOL(ehash_int_muram_area);
	}

	/*
	 * No MURAM AD allocation here.  info->h_Ad stays NULL until
	 * copy_td_to_ccbase() writes the AD into the CC tree's MURAM
	 * slot during FM_PCD_CcRootBuild.
	 */
	info->h_Ad = NULL;
	info->table_type = p_Param->table_type;

	/*
	 * Track reassembly tables for ExternalHashSetReasslyPool().
	 * For REASSM tables, word_2 holds the MURAM offset (>>8) of
	 * ip_reassembly_params.  Compute the VA so we can write pool
	 * configuration fields later.
	 */
	if (p_Param->table_type == EHASH_IPV4_REASSM_TABLE ||
	    p_Param->table_type == EHASH_IPV6_REASSM_TABLE) {
		t_FmPcd *p_FmPcd = (t_FmPcd *)h_FmPcd;
		uint64_t reassm_phys;

		if (node->word_2 != 0) {
			reassm_phys = p_FmPcd->physicalMuramBase +
			    ((uint64_t)node->word_2 << 8);
			info->ip_reassem_info =
			    (struct ip_reassembly_params *)XX_PhysToVirt(
			    reassm_phys);
			printf("fm_ehash: REASSM table type %u, "
			    "ip_reassem_info at %p (MURAM off 0x%x)\n",
			    p_Param->table_type, info->ip_reassem_info,
			    node->word_2 << 8);
		}

		if (p_Param->table_type == EHASH_IPV4_REASSM_TABLE)
			ehash_ipv4_reassly_tbl = info;
		else
			ehash_ipv6_reassly_tbl = info;
	}

	return ((t_Handle)info);

err_ret:
	delete_ehash_info(info);
	return (NULL);
}

/*
 * copy_td_to_ccbase — Write enhanced hash AD into CC tree MURAM entry.
 *
 * Called from FM_PCD_CcRootBuild (USE_ENHANCED_EHASH path) for each
 * CC tree entry that points to a hash table.  Copies the 16-byte AD
 * from info->node to the CC tree's MURAM slot using big-endian
 * accessors, and saves the MURAM address to info->h_Ad.
 *
 * After this call, FMan reads the CC tree entry and finds CDX enhanced
 * hash format.  ExternalHashTableModifyMissNextEngine can later patch
 * the miss action via info->h_Ad.
 *
 * No IPR reassembly support (void return).
 */
void
copy_td_to_ccbase(void *handle, t_Handle p_CcTreeTmp)
{
	struct en_exthash_info *info = (struct en_exthash_info *)handle;
	volatile uint32_t *p = (volatile uint32_t *)p_CcTreeTmp;

	struct en_exthash_node *ptr = &info->node;

	/*
	 * Write enhanced hash AD into CC tree MURAM slot.
	 *
	 * Enhanced hash node layout (16 bytes, all BE in MURAM):
	 *   p[0] = word_0:        flags, miss_action, int_buf_pool, etc.
	 *   p[1] = table_base_lo: DDR hash table physical address (low 32)
	 *   p[2] = word_1:        global_mem_offset, hash_mask, key_size
	 *   p[3] = word_2:        miss FQID (set to 0 here, patched by ModifyMiss)
	 *
	 * The CDX microcode reads this format for CC lookup.
	 * Standard Result ADs do NOT work — CDX microcode ignores
	 * the standard FQID field and uses KG baseFqid instead
	 * (confirmed build #207: all frames discarded with QMan
	 * Invalid Enqueue State for FQID 65882).
	 */
	{
		uint32_t v0, v1, v2, v3;
		int retry;

		for (retry = 0; retry < 4; retry++) {
			muram_wr32(&p[2], ptr->word_1);        /* offset 8  */
			muram_wr32(&p[1], ptr->table_base_lo); /* offset 4  */
			muram_wr32(&p[3], ptr->word_2);        /* offset 12 */
			muram_wr32(&p[0], ptr->word_0);        /* offset 0  */
			muram_barrier();

			v0 = muram_rd32(&p[0]);
			v1 = muram_rd32(&p[1]);
			v2 = muram_rd32(&p[2]);
			v3 = muram_rd32(&p[3]);
			if (v0 == ptr->word_0 &&
			    v1 == ptr->table_base_lo &&
			    v2 == ptr->word_1 &&
			    v3 == ptr->word_2)
				break;

			printf("fm_ehash: copy_td VERIFY FAIL attempt %d "
			    "AD=%p [%08x %08x %08x %08x] != "
			    "[%08x %08x %08x %08x]\n",
			    retry + 1, p, v0, v1, v2, v3,
			    ptr->word_0, ptr->table_base_lo,
			    ptr->word_1, ptr->word_2);
		}
	}

	/* Save MURAM address for AD — used by ModifyMissNextEngine later */
	info->h_Ad = (void *)p;
}

/*
 * ExternalHashTableAllocEntry — Allocate a new hash table entry.
 */
void *
ExternalHashTableAllocEntry(void *h_HashTbl)
{
	struct en_exthash_info *info;
	void *entry;

	info = (struct en_exthash_info *)h_HashTbl;
	if (info == NULL)
		return (NULL);

	entry = XX_MallocSmart(sizeof(struct en_exthash_tbl_entry),
	    0, EN_EHASH_ENTRY_ALIGN);
	if (entry != NULL)
		memset(entry, 0, sizeof(struct en_exthash_tbl_entry));
	else
		REPORT_ERROR(MAJOR, E_NO_MEMORY, ("hash table entry"));

	return (entry);
}

/*
 * ExternalHashTableEntryFree — Free a hash table entry.
 */
void
ExternalHashTableEntryFree(void *entry)
{

	if (entry != NULL)
		XX_FreeSmart(entry);
}

/*
 * ExternalHashTableAddKey — Add an entry to the hash table.
 *
 * Returns the bucket index on success, -1 on failure.
 */
int
ExternalHashTableAddKey(void *h_HashTbl, uint8_t keySize, void *tbl_entry)
{
	struct en_exthash_info *info;
	struct en_exthash_tbl_entry *first_entry;
	struct en_exthash_tbl_entry *new_entry;
	uint16_t index;
	struct en_exthash_bucket *bucket;
	t_Handle *h_Spinlock;
	uint32_t intFlags;
	int retval;
	uint64_t phyaddr;

	SANITY_CHECK_RETURN_ERROR(h_HashTbl, E_INVALID_HANDLE);
	SANITY_CHECK_RETURN_ERROR(tbl_entry, E_NULL_POINTER);

	new_entry = (struct en_exthash_tbl_entry *)tbl_entry;
	info = (struct en_exthash_info *)h_HashTbl;

	if (info->table_base == NULL || info->pSpinlock == NULL) {
		REPORT_ERROR(MAJOR, E_INVALID_HANDLE,
		    ("hash table not initialized (table_base=%p pSpinlock=%p)",
		    info->table_base, info->pSpinlock));
		return (-1);
	}

	get_indexed_hash_bucket(keySize, &new_entry->hashentry.key[0],
	    info->hashshift, (uint16_t)info->hashmask, &index);

	bucket = (struct en_exthash_bucket *)info->table_base + index;

	h_Spinlock = *(info->pSpinlock + index);
	intFlags = XX_LockIntrSpinlock(h_Spinlock);

	retval = index;
	phyaddr = bucket->h;

	if (phyaddr) {
		first_entry = XX_PhysToVirt(SwapUint64(phyaddr));
		if (find_entry_in_bucket(first_entry,
		    &new_entry->hashentry.key[0], keySize)) {
			REPORT_ERROR(MAJOR, E_ALREADY_EXISTS,
			    ("hash table entry"));
			retval = -1;
			goto func_ret;
		}
		first_entry->prev = new_entry;
	} else {
		first_entry = NULL;
	}

	/* Link new entry at head of bucket chain */
	new_entry->next = first_entry;

	/* Fill hardware next-entry pointer (byte-swapped physical addr) */
	phyaddr = SwapUint64(phyaddr);
	new_entry->hashentry.next_entry_hi =
	    cpu_to_be16((phyaddr >> 32) & 0xffff);
	new_entry->hashentry.next_entry_lo =
	    cpu_to_be32(phyaddr & 0xffffffff);

	/* Update bucket head to point to new entry */
	phyaddr = XX_VirtToPhys(new_entry);
	bucket->h = SwapUint64(phyaddr);

func_ret:
	XX_UnlockIntrSpinlock(h_Spinlock, intFlags);
	return (retval);
}

/*
 * ExternalHashTableDeleteKey — Remove an entry from the hash table.
 *
 * Returns 0 on success, -1 on failure.
 */
int
ExternalHashTableDeleteKey(void *h_HashTbl, uint16_t index, void *tbl_entry)
{
	struct en_exthash_info *info;
	struct en_exthash_bucket *bucket;
	struct en_exthash_tbl_entry *entry;
	struct en_exthash_tbl_entry *temp_entry;
	t_Handle *h_Spinlock;
	uint32_t intFlags;
	uint64_t phyaddr;
	uint64_t update_entry;

	if (h_HashTbl == NULL || tbl_entry == NULL)
		return (-1);

	info = (struct en_exthash_info *)h_HashTbl;

	if (info->table_base == NULL || info->pSpinlock == NULL) {
		REPORT_ERROR(MAJOR, E_INVALID_HANDLE,
		    ("hash table not initialized in DeleteKey"));
		return (-1);
	}

	if (index > info->hashmask) {
		REPORT_ERROR(MAJOR, E_INVALID_VALUE,
		    ("DeleteKey index %u exceeds hashmask %u",
		    (unsigned)index, (unsigned)info->hashmask));
		return (-1);
	}

	bucket = (struct en_exthash_bucket *)info->table_base + index;
	entry = (struct en_exthash_tbl_entry *)tbl_entry;

	h_Spinlock = *(info->pSpinlock + index);
	intFlags = XX_LockIntrSpinlock(h_Spinlock);

	/* Mark entry as invalid for microcode */
	update_entry = SwapUint64(entry->hashentry.next_entry);
	SET_INVALID_ENTRY_64BIT(update_entry);
	entry->hashentry.next_entry = SwapUint64(update_entry);

	if (entry->prev != NULL) {
		/* Not at head — unlink from middle/end */
		temp_entry = entry->prev;
		temp_entry->next = entry->next;
		if (entry->next != NULL)
			entry->next->prev = temp_entry;
		/* Update hardware next-entry pointer */
		update_entry = entry->hashentry.next_entry & ~0xffffULL;
		temp_entry->hashentry.next_entry =
		    update_entry | temp_entry->hashentry.flags;
	} else {
		/* At head — update bucket pointer */
		if (entry->next != NULL) {
			phyaddr = XX_VirtToPhys(entry->next);
			bucket->h = SwapUint64(phyaddr);
			entry->next->prev = NULL;
		} else {
			/* Last entry in bucket — clear to zero (empty) */
			bucket->h = 0;
		}
	}

	XX_UnlockIntrSpinlock(h_Spinlock, intFlags);

	if (FmPcdHcSync(info->pcd)) {
		printf("fm_ehash: FmPcdHcSync failed in DeleteKey\n");
		return (-1);
	}

	return (0);
}

/*
 * ExternalHashTableEntryGetStatsAndTS — Read entry stats and timestamp.
 */
int
ExternalHashTableEntryGetStatsAndTS(void *tbl_entry,
    struct en_tbl_entry_stats *stats)
{
	struct en_exthash_tbl_entry *hash_node;
	struct en_ehash_entry *entry;
	uint16_t flags;

	hash_node = (struct en_exthash_tbl_entry *)tbl_entry;
	if (hash_node == NULL)
		return (-1);

	entry = &hash_node->hashentry;
	flags = cpu_to_be16(entry->flags);
	stats->flags = 0;

	if (GET_TIMESTAMP_ENABLE(flags)) {
		stats->flags |= TIMESTAMP_VALID;
		stats->timestamp = cpu_to_be32(entry->timestamp);
	}

	if (GET_STATS_ENABLE(flags)) {
		stats->flags |= STATS_VALID;
		stats->pkts = be64_to_cpu(entry->packet_count);
		stats->bytes = be64_to_cpu(entry->packet_bytes);
	}

	return (0);
}

/*
 * ExternalHashTableModifyMissNextEngine — Change the miss action.
 *
 * Modifies the MURAM AD node directly (via info->h_Ad, set by
 * copy_td_to_ccbase).  ALL 4 words of the AD must be written in
 * a single batch — partial writes corrupt the unwritten words
 * (LS1046A MURAM / Normal-NC ARM64 store behavior).
 *
 * Uses info->node as the base (unchanged table_base_lo, word_1)
 * and modifies word_0 (miss action) and word_2 (FQID/NIA).
 * Also updates info->node so it stays in sync for future calls.
 */
t_Error
ExternalHashTableModifyMissNextEngine(t_Handle h_HashTbl,
    t_FmPcdCcNextEngineParams *p_FmPcdCcNextEngineParams)
{
	struct en_exthash_info *info;
	volatile uint32_t *p;
	struct en_exthash_node *ptr;

	info = (struct en_exthash_info *)h_HashTbl;
	if (info->h_Ad == NULL) {
		printf("fm_ehash: ModifyMissNextEngine: h_Ad is NULL "
		    "(copy_td_to_ccbase not called?)\n");
		return (E_INVALID_STATE);
	}

	ptr = &info->node;

	/*
	 * Enhanced hash AD miss action patching.
	 *
	 * Modify word_0 (miss_action bits [31:30]) and word_2 (FQID/NIA)
	 * while preserving table_base_lo and word_1 (hash table params).
	 *
	 * Build #208: reverted from standard Result AD format which the
	 * CDX microcode cannot interpret (build #207 showed all frames
	 * discarded with QMan Invalid Enqueue State for KG baseFqid).
	 */
	{
		uint32_t fqid = 0;

		switch (p_FmPcdCcNextEngineParams->nextEngine) {
		case e_FM_PCD_DONE: {
			t_FmPcdCcNextEnqueueParams *enqparams =
			    &p_FmPcdCcNextEngineParams->params.enqueueParams;
			if (enqparams->overrideFqid)
				fqid = enqparams->newFqid & 0x00FFFFFF;

			/* Set miss_action to ENQUE, preserve other word_0 fields */
			ptr->word_0 = (ptr->word_0 & ~EHASH_W0_MISS_ACTION_MASK) |
			    EHASH_W0_MISS_ACTION(EN_EHASH_MISS_ACTION_ENQUE);
			ptr->word_2 = fqid;
			break;
		}
		default:
			printf("fm_ehash: ModifyMiss: unhandled engine %d\n",
			    p_FmPcdCcNextEngineParams->nextEngine);
			break;
		}

	}

	/*
	 * Write the 16-byte AD to MURAM.
	 * Verify + retry in case of CCI-400/AXI flush timing.
	 */
	p = (volatile uint32_t *)info->h_Ad;
	{
		uint32_t v0, v1, v2, v3;
		int retry;

		for (retry = 0; retry < 4; retry++) {
			muram_wr32(&p[2], ptr->word_1);        /* offset 8  */
			muram_wr32(&p[1], ptr->table_base_lo); /* offset 4  */
			muram_wr32(&p[3], ptr->word_2);        /* offset 12 */
			muram_wr32(&p[0], ptr->word_0);        /* offset 0  */
			muram_barrier();

			v0 = muram_rd32(&p[0]);
			v1 = muram_rd32(&p[1]);
			v2 = muram_rd32(&p[2]);
			v3 = muram_rd32(&p[3]);
			if (v0 == ptr->word_0 &&
			    v1 == ptr->table_base_lo &&
			    v2 == ptr->word_1 &&
			    v3 == ptr->word_2)
				break;

			printf("fm_ehash: ModifyMiss AD=%p attempt %d "
			    "VERIFY FAIL [%08x %08x %08x %08x] != "
			    "[%08x %08x %08x %08x]\n",
			    info->h_Ad, retry + 1,
			    v0, v1, v2, v3,
			    ptr->word_0, ptr->table_base_lo,
			    ptr->word_1, ptr->word_2);
		}
	}

	/*
	 * Propagate miss action to CC tree copy entries.
	 *
	 * The CC tree fill logic (fm_cc.c CcRootBuild) copies entry 0
	 * to unused slots (indices numOfEntries..15) via raw memcpy.
	 * These copies have the pre-ModifyMiss state because
	 * copy_td_to_ccbase only tracks a single h_Ad per hash table.
	 *
	 * Scan all 16 CC tree entries: any entry (other than h_Ad
	 * itself) that is a copy and needs its miss action updated.
	 *
	 * CC tree is 256-byte aligned (FM_PCD_CC_TREE_ADDR_ALIGN).
	 */
	{
		uintptr_t tree_base =
		    (uintptr_t)info->h_Ad & ~(uintptr_t)0xFF;
		int j;

		for (j = 0; j < FM_PCD_MAX_NUM_OF_CC_GROUPS; j++) {
			volatile uint32_t *ep = (volatile uint32_t *)
			    (tree_base + j * FM_PCD_CC_AD_ENTRY_SIZE);
			if (ep == (volatile uint32_t *)info->h_Ad)
				continue;  /* skip the entry we just wrote */
			if (muram_rd32(&ep[1]) == ptr->table_base_lo) {
				/* Copy with same table — update all 4 words */
				muram_wr32(&ep[2], ptr->word_1);
				muram_wr32(&ep[1], ptr->table_base_lo);
				muram_wr32(&ep[3], ptr->word_2);
				muram_wr32(&ep[0], ptr->word_0);
				muram_barrier();
			}
		}
	}

	return (E_OK);
}

/*
 * ExternalHashTableFmPcdHcSync — Flush PCD Host Command channel.
 */
int
ExternalHashTableFmPcdHcSync(void *h_HashTbl)
{
	struct en_exthash_info *info;

	info = (struct en_exthash_info *)h_HashTbl;
	if (FmPcdHcSync(info->pcd)) {
		printf("fm_ehash: FmPcdHcSync failed\n");
		return (-1);
	}
	return (0);
}

/*
 * ExternalHashTableDelete — Destroy a table and free all resources.
 */
void
ExternalHashTableDelete(t_Handle h_HashTbl)
{

	if (h_HashTbl != NULL)
		delete_ehash_info((struct en_exthash_info *)h_HashTbl);
}

/* ================================================================
 * Global MURAM features — read/write the global data area at
 * pool + EN_INTERNAL_BUFF_POOL_SIZE (en_exthash_global_mem).
 *
 * All fields are big-endian in MURAM.  Use GET_UINT32/WRITE_UINT32
 * for 32-bit fields, GET_UINT8/WRITE_UINT8 for byte fields.
 * ================================================================ */

int32_t
ExternalHashGetSECfailureStats(en_SEC_failure_stats *stats)
{
	volatile en_SEC_failure_stats *src;

	if (stats == NULL)
		return (-1);
	if (ehash_global_mem == NULL) {
		memset(stats, 0, sizeof(*stats));
		return (-1);
	}

	src = &ehash_global_mem->SEC_failure_stats;

	stats->icv_failures			= GET_UINT32(src->icv_failures);
	stats->hw_errs				= GET_UINT32(src->hw_errs);
	stats->CCM_AAD_size_errs		= GET_UINT32(src->CCM_AAD_size_errs);
	stats->anti_replay_late_errs		= GET_UINT32(src->anti_replay_late_errs);
	stats->anti_replay_replay_errs		= GET_UINT32(src->anti_replay_replay_errs);
	stats->seq_num_overflows		= GET_UINT32(src->seq_num_overflows);
	stats->DMA_errs				= GET_UINT32(src->DMA_errs);
	stats->DECO_watchdog_timer_timedout_errs = GET_UINT32(src->DECO_watchdog_timer_timedout_errs);
	stats->input_frame_read_errs		= GET_UINT32(src->input_frame_read_errs);
	stats->protocol_format_errs		= GET_UINT32(src->protocol_format_errs);
	stats->ipsec_ttl_zero_errs		= GET_UINT32(src->ipsec_ttl_zero_errs);
	stats->ipsec_pad_chk_failures		= GET_UINT32(src->ipsec_pad_chk_failures);
	stats->output_frame_length_rollover_errs = GET_UINT32(src->output_frame_length_rollover_errs);
	stats->tbl_buff_too_small_errs		= GET_UINT32(src->tbl_buff_too_small_errs);
	stats->tbl_buff_pool_depletion_errs	= GET_UINT32(src->tbl_buff_pool_depletion_errs);
	stats->output_frame_too_large_errs	= GET_UINT32(src->output_frame_too_large_errs);
	stats->cmpnd_frame_write_errs		= GET_UINT32(src->cmpnd_frame_write_errs);
	stats->buff_too_small_errs		= GET_UINT32(src->buff_too_small_errs);
	stats->buff_pool_depletion_errs		= GET_UINT32(src->buff_pool_depletion_errs);
	stats->output_frame_write_errs		= GET_UINT32(src->output_frame_write_errs);
	stats->cmpnd_frame_read_errs		= GET_UINT32(src->cmpnd_frame_read_errs);
	stats->prehdr_read_errs			= GET_UINT32(src->prehdr_read_errs);
	stats->other_errs			= GET_UINT32(src->other_errs);

	return (0);
}

int32_t
ExternalHashResetSECfailureStats(void)
{
	volatile en_SEC_failure_stats *dst;

	if (ehash_global_mem == NULL)
		return (-1);

	dst = &ehash_global_mem->SEC_failure_stats;

	WRITE_UINT32(dst->icv_failures, 0);
	WRITE_UINT32(dst->hw_errs, 0);
	WRITE_UINT32(dst->CCM_AAD_size_errs, 0);
	WRITE_UINT32(dst->anti_replay_late_errs, 0);
	WRITE_UINT32(dst->anti_replay_replay_errs, 0);
	WRITE_UINT32(dst->seq_num_overflows, 0);
	WRITE_UINT32(dst->DMA_errs, 0);
	WRITE_UINT32(dst->DECO_watchdog_timer_timedout_errs, 0);
	WRITE_UINT32(dst->input_frame_read_errs, 0);
	WRITE_UINT32(dst->protocol_format_errs, 0);
	WRITE_UINT32(dst->ipsec_ttl_zero_errs, 0);
	WRITE_UINT32(dst->ipsec_pad_chk_failures, 0);
	WRITE_UINT32(dst->output_frame_length_rollover_errs, 0);
	WRITE_UINT32(dst->tbl_buff_too_small_errs, 0);
	WRITE_UINT32(dst->tbl_buff_pool_depletion_errs, 0);
	WRITE_UINT32(dst->output_frame_too_large_errs, 0);
	WRITE_UINT32(dst->cmpnd_frame_write_errs, 0);
	WRITE_UINT32(dst->buff_too_small_errs, 0);
	WRITE_UINT32(dst->buff_pool_depletion_errs, 0);
	WRITE_UINT32(dst->output_frame_write_errs, 0);
	WRITE_UINT32(dst->cmpnd_frame_read_errs, 0);
	WRITE_UINT32(dst->prehdr_read_errs, 0);
	WRITE_UINT32(dst->other_errs, 0);

	return (0);
}

int32_t
ExternalHashSetDscpVlanpcpMapCfg(en_dscp_vlanpcp_map_cfg *map)
{
	volatile uint8_t *dst;
	int ii;

	if (map == NULL)
		return (-1);
	if (ehash_global_mem == NULL)
		return (-1);

	dst = ehash_global_mem->dscp_vlanpcp_map.dscp_vlanpcp;
	for (ii = 0; ii <= MAX_VLAN_PCP; ii++)
		WRITE_UINT8(dst[ii], map->dscp_vlanpcp[ii]);

	return (0);
}

int32_t
ExternalHashGetDscpVlanpcpMapCfg(en_dscp_vlanpcp_map_cfg *map)
{
	volatile uint8_t *src;
	int ii;

	if (map == NULL)
		return (-1);
	if (ehash_global_mem == NULL) {
		memset(map, 0, sizeof(*map));
		return (-1);
	}

	src = ehash_global_mem->dscp_vlanpcp_map.dscp_vlanpcp;
	for (ii = 0; ii <= MAX_VLAN_PCP; ii++)
		map->dscp_vlanpcp[ii] = GET_UINT8(src[ii]);

	return (0);
}

/*
 * ExternalHashReasslyTableExists — Check if a reassembly table exists.
 *
 * Returns 1 if the specified reassembly table (IPv4 or IPv6) was
 * created during PCD setup, 0 otherwise.  Used by the CDX module
 * to skip resource allocation when reassembly is not configured.
 */
int
ExternalHashReasslyTableExists(uint32_t type)
{
	struct en_exthash_info *tbl_info;

	switch (type) {
	case EHASH_IPV4_REASSM_TABLE:
		tbl_info = ehash_ipv4_reassly_tbl;
		break;
	case EHASH_IPV6_REASSM_TABLE:
		tbl_info = ehash_ipv6_reassly_tbl;
		break;
	default:
		return (0);
	}

	return (tbl_info != NULL && tbl_info->ip_reassem_info != NULL);
}

/*
 * ExternalHashSetReasslyPool — Configure reassembly buffer pools.
 *
 * Called from CDX module (cdx_reassm_freebsd.c) to program BMan pool
 * IDs, buffer sizes, timeout, and TX-confirm FQIDs into the MURAM
 * ip_reassembly_params structure for IPv4/IPv6 reassembly tables.
 *
 * The ip_reassem_info pointer is set during ExternalHashTableSet()
 * when a REASSM table is created.  If no REASSM table exists (e.g.
 * dpa_app PCD XML doesn't include reassembly), this returns -1.
 */
int
ExternalHashSetReasslyPool(uint32_t type, uint32_t ctx_bpid,
    uint32_t ctx_bpsize, uint32_t frag_bpid, uint32_t frag_size,
    uint32_t txc_fqid, uint32_t ipr_timer_freq)
{
	struct en_exthash_info *tbl_info;
	volatile struct ip_reassembly_params *params;
	uint32_t timeout;

	switch (type) {
	case EHASH_IPV4_REASSM_TABLE:
		tbl_info = ehash_ipv4_reassly_tbl;
		break;
	case EHASH_IPV6_REASSM_TABLE:
		tbl_info = ehash_ipv6_reassly_tbl;
		break;
	default:
		printf("fm_ehash: SetReasslyPool: invalid type %u\n", type);
		return (-1);
	}

	if (tbl_info == NULL || tbl_info->ip_reassem_info == NULL)
		return (-1);

	params = (volatile struct ip_reassembly_params *)tbl_info->ip_reassem_info;

	printf("fm_ehash: SetReasslyPool type %u: ctx_bpid=%u frag_bpid=%u "
	    "timer_freq=%u txc_fqid=%u\n",
	    type, ctx_bpid, frag_bpid, ipr_timer_freq, txc_fqid);

	WRITE_UINT32(params->reassem_bpid, ctx_bpid);
	WRITE_UINT32(params->reassem_bsize, ctx_bpsize);
	WRITE_UINT32(params->frag_bpid, frag_bpid);
	WRITE_UINT32(params->frag_bsize, frag_size);
	WRITE_UINT32(params->txc_fqid, txc_fqid);

	/* Convert timeout from absolute to timer-frequency-relative.
	 * The existing timeout_val was set by the PCD configuration
	 * in absolute units; divide by timer frequency. */
	if (ipr_timer_freq != 0) {
		timeout = GET_UINT32(params->timeout_val);
		WRITE_UINT32(params->timeout_val, timeout / ipr_timer_freq);
	}

	return (0);
}

void
ipr_update_timestamp(void)
{

	/* Timestamp updates handled by cdx_ehash_update_timestamp() */
}

/*
 * cdx_ehash_update_timestamp — Write current time to MURAM timestamp slot.
 *
 * Called from the CDX timer thread (via dpa_update_timestamp) on every
 * tick.  FMan microcode reads these slots and copies the raw value into
 * each hash entry's timestamp field on packet match (MURAM-to-MURAM).
 *
 * NO byte-swap: Linux SDK's FM_PCD_UpdateExtTimeStamp does a raw store.
 * CDX software reads the timestamp back with the same raw access, so
 * both sides see the same byte pattern.
 */
void
cdx_ehash_update_timestamp(uint32_t id, uint32_t value)
{

	if (ehash_ts_virt == NULL || id >= MAX_EXT_TS_TIMERS)
		return;
	*(volatile uint32_t *)&ehash_ts_virt[id] = value;
	__asm __volatile("dsb sy" ::: "memory");
}

/*
 * cdx_ehash_get_timestamp_addr — Return MURAM physical offset for a
 * timestamp slot.
 */
uint32_t
cdx_ehash_get_timestamp_addr(uint32_t id)
{

	if (ehash_ts_virt == NULL || id >= MAX_EXT_TS_TIMERS)
		return (0);
	return ((uint32_t)(ehash_ts_phys + id * sizeof(uint32_t) -
	    ehash_muram_phys_base));
}

/*
 * ehash_dump_muram_diag — Dump MURAM state for stall diagnosis.
 *
 * Prints global data area, timestamp slots, and first internal
 * buffer pool slot.  Safe to call from callout context.
 */
static void
ehash_dump_muram_diag(void)
{
	uint8_t *global_data;
	volatile uint32_t *pool32;

	if (ehash_int_muram_ptr == NULL) {
		printf("fm_ehash: diag: pool not initialized\n");
		return;
	}

	/* Global data area: at pool + EN_INTERNAL_BUFF_POOL_SIZE */
	global_data = (uint8_t *)ehash_int_muram_ptr +
	    EN_INTERNAL_BUFF_POOL_SIZE;
	printf("fm_ehash: global_data (dscp_vlanpcp_map): "
	    "%02x %02x %02x %02x %02x %02x %02x %02x\n",
	    global_data[0], global_data[1], global_data[2], global_data[3],
	    global_data[4], global_data[5], global_data[6], global_data[7]);
	/* SEC failure stats — 23 × uint32_t starting at global_data+8.
	 * CDX microcode writes these in BE (via WRITE_UINT32), so read
	 * with GET_UINT32 for correct byte order. */
	{
		static const char *stat_names[] = {
			"icv_fail", "hw_err", "ccm_aad", "ar_late",
			"ar_replay", "seq_ovfl", "dma_err", "deco_wdt",
			"in_rd_err", "proto_fmt", "ttl_zero", "pad_chk",
			"out_len_roll", "tbl_buf_small", "tbl_pool_depl",
			"out_too_large", "cmpnd_wr", "buf_small",
			"pool_depl", "out_wr", "cmpnd_rd",
			"prehdr_rd", "other"
		};
		volatile uint32_t *sp =
		    (volatile uint32_t *)(global_data + 8);
		int si, any = 0;
		for (si = 0; si < 23; si++) {
			uint32_t v = GET_UINT32(sp[si]);
			if (v != 0) {
				printf("fm_ehash: SEC stat[%d] %s = %u\n",
				    si, stat_names[si], v);
				any = 1;
			}
		}
		if (!any)
			printf("fm_ehash: SEC stats: all zero\n");
	}

	/* Timestamp slots (separate allocation) */
	if (ehash_ts_virt != NULL)
		printf("fm_ehash: ts[0..3]: %08x %08x %08x %08x "
		    "(raw, no swap)\n",
		    *(volatile uint32_t *)&ehash_ts_virt[0],
		    *(volatile uint32_t *)&ehash_ts_virt[1],
		    *(volatile uint32_t *)&ehash_ts_virt[2],
		    *(volatile uint32_t *)&ehash_ts_virt[3]);
	else
		printf("fm_ehash: ts: not allocated\n");

	/* Internal buffer pool — first 4 slots (each 256 bytes).
	 * Read with GET_UINT32 since microcode writes in BE. */
	pool32 = (volatile uint32_t *)ehash_int_muram_ptr;
	{
		int slot;
		for (slot = 0; slot < 4; slot++) {
			int base = slot * 64; /* 256B / 4B per word */
			printf("fm_ehash: pool[%d]: %08x %08x %08x %08x\n",
			    slot,
			    GET_UINT32(pool32[base + 0]),
			    GET_UINT32(pool32[base + 1]),
			    GET_UINT32(pool32[base + 2]),
			    GET_UINT32(pool32[base + 3]));
		}
	}

	/* DDR hash table bucket[0] verification */
	if (ehash_first_table_base != NULL) {
		volatile uint64_t *bkt =
		    (volatile uint64_t *)ehash_first_table_base;
		printf("fm_ehash: DDR bucket[0] (cached): "
		    "%016lx %016lx\n", bkt[0], bkt[1]);
		/* Writeback + invalidate to force re-read from DDR */
		cpu_dcache_wbinv_range((vm_offset_t)ehash_first_table_base, 64);
		printf("fm_ehash: DDR bucket[0] (after wbinv): "
		    "%016lx %016lx\n", bkt[0], bkt[1]);
	}
}

/* ================================================================
 * Debug
 * ================================================================ */

static void
EhashTableWalk(void *h_HashTbl)
{
	struct en_exthash_info *info;
	struct en_exthash_tbl_entry *entry;
	struct en_exthash_bucket *bucket;
	uint32_t ii;
	uint32_t num_entries = 0;
	uint32_t max_collisions = 0;

	info = (struct en_exthash_info *)h_HashTbl;
	if (info == NULL)
		return;

	bucket = (struct en_exthash_bucket *)info->table_base;
	for (ii = 0; ii <= info->hashmask; ii++) {
		if (bucket->h != 0) {
			uint32_t bucket_entries = 0;

			entry = XX_PhysToVirt(SwapUint64(bucket->h));
			while (entry != NULL) {
				bucket_entries++;
				entry = entry->next;
			}
			num_entries += bucket_entries;
			if (bucket_entries > max_collisions)
				max_collisions = bucket_entries;
		}
		bucket++;
	}

	printf("fm_ehash: table %p — %u entries, max collisions %u, "
	    "%u buckets\n", info, num_entries, max_collisions,
	    info->hashmask + 1);
}
