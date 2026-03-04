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
 * DPAA1 / NetCommSW build flags for ARM64 (LS1046A).
 *
 * Equivalent to dflags.h (which targets PowerPC e500mc/P5020).
 * Force-included before every ncsw source file via config.dpaa_arm64.
 */
#ifndef DFLAGS_ARM64_H_
#define DFLAGS_ARM64_H_

#include "opt_platform.h"
#include "events_mapping.h"

/*
 * Core selection: ARM64 (Cortex-A72).
 * The PPC path uses NCSW_PPC_CORE which pulls in ppc_ext.h and e500v2_ext.h.
 * Our ARM path uses NCSW_ARM_CORE which pulls in arm_ext.h with FreeBSD
 * ARM64 implementations.
 */
#define	NCSW_ARM_CORE
#define	NCSW_FREEBSD

/* Cortex-A72 cache line size (LS1046A) */
#define	CORE_CACHELINE_SIZE	64

/* Linux-ism helpers needed by mEMAC flib code */
#include <asm/byteorder.h>
#include <linux/delay.h>

/*
 * LIST_ compatibility macros — FreeBSD's list_ext.h prefixes everything
 * with NCSW_ to avoid conflicts with <sys/queue.h>.  The mEMAC code
 * uses the unprefixed original names from the NXP SDK.
 *
 * Note: LIST_FIRST/NEXT/PREV/LAST conflict with <sys/queue.h> and
 * are intentionally omitted here.
 */
#define	LIST_Add		NCSW_LIST_Add
#define	LIST_AddToTail		NCSW_LIST_AddToTail
#define	LIST_DelAndInit		NCSW_LIST_DelAndInit
#define	LIST_IsEmpty		NCSW_LIST_IsEmpty
#define	LIST_OBJECT		NCSW_LIST_OBJECT
#define	LIST_FOR_EACH		NCSW_LIST_FOR_EACH
#define	LIST_FOR_EACH_SAFE	NCSW_LIST_FOR_EACH_SAFE
#define	LIST_FOR_EACH_OBJECT	NCSW_LIST_FOR_EACH_OBJECT
#define	LIST_FOR_EACH_OBJECT_SAFE	NCSW_LIST_FOR_EACH_OBJECT_SAFE

/* Debugging */
#define	DEBUG_ERRORS		1
#define	DPAA_DEBUG		1
#if defined(DPAA_DEBUG)
#define	DEBUG_GLOBAL_LEVEL	REPORT_LEVEL_INFO
#else
#define	DEBUG_GLOBAL_LEVEL	REPORT_LEVEL_WARNING
#endif

/* Events */
#define	REPORT_EVENTS		1
#define	EVENT_GLOBAL_LEVEL	REPORT_LEVEL_MINOR

#endif /* DFLAGS_ARM64_H_ */
