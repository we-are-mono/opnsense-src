/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Mono Technologies Inc.
 *
 * I/O macros for ARM64 — equivalent to powerpc/include/pio.h.
 * Provides in8/out8/../in64/out64 used by the NetCommSW (ncsw) DPAA1 driver.
 * All accesses are memory-mapped volatile loads/stores with a DSB barrier
 * to match the ordering semantics of the PowerPC "sync" in the original.
 */

#ifndef _MACHINE_PIO_H_
#define	_MACHINE_PIO_H_

#include <sys/types.h>

/*
 * Full system data synchronisation barrier.
 * Ensures completion of all preceding memory accesses before any
 * subsequent access can begin.  Equivalent to PowerPC "sync".
 */
#define	arm64_iomb()	__asm __volatile("dsb sy" : : : "memory")

static __inline void
__outb(volatile u_int8_t *a, u_int8_t v)
{
	*a = v;
	arm64_iomb();
}

static __inline void
__outw(volatile u_int16_t *a, u_int16_t v)
{
	*a = v;
	arm64_iomb();
}

static __inline void
__outl(volatile u_int32_t *a, u_int32_t v)
{
	*a = v;
	arm64_iomb();
}

static __inline void
__outll(volatile u_int64_t *a, u_int64_t v)
{
	*a = v;
	arm64_iomb();
}

static __inline u_int8_t
__inb(volatile u_int8_t *a)
{
	u_int8_t v;

	v = *a;
	arm64_iomb();
	return (v);
}

static __inline u_int16_t
__inw(volatile u_int16_t *a)
{
	u_int16_t v;

	v = *a;
	arm64_iomb();
	return (v);
}

static __inline u_int32_t
__inl(volatile u_int32_t *a)
{
	u_int32_t v;

	v = *a;
	arm64_iomb();
	return (v);
}

static __inline u_int64_t
__inll(volatile u_int64_t *a)
{
	u_int64_t v;

	v = *a;
	arm64_iomb();
	return (v);
}

#define	outb(a,v)	(__outb((volatile u_int8_t *)(a), v))
#define	out8(a,v)	outb(a,v)
#define	outw(a,v)	(__outw((volatile u_int16_t *)(a), v))
#define	out16(a,v)	outw(a,v)
#define	outl(a,v)	(__outl((volatile u_int32_t *)(a), v))
#define	out32(a,v)	outl(a,v)
#define	outll(a,v)	(__outll((volatile u_int64_t *)(a), v))
#define	out64(a,v)	outll(a,v)
#define	inb(a)		(__inb((volatile u_int8_t *)(a)))
#define	in8(a)		inb(a)
#define	inw(a)		(__inw((volatile u_int16_t *)(a)))
#define	in16(a)		inw(a)
#define	inl(a)		(__inl((volatile u_int32_t *)(a)))
#define	in32(a)		inl(a)
#define	inll(a)		(__inll((volatile u_int64_t *)(a)))
#define	in64(a)		inll(a)

/* Reverse-byte variants — ARM64 is LE, so these byte-swap for BE regs */
static __inline void
__outwrb(volatile u_int16_t *a, u_int16_t v)
{
	*a = __builtin_bswap16(v);
	arm64_iomb();
}

static __inline void
__outlrb(volatile u_int32_t *a, u_int32_t v)
{
	*a = __builtin_bswap32(v);
	arm64_iomb();
}

static __inline u_int16_t
__inwrb(volatile u_int16_t *a)
{
	u_int16_t v;

	v = *a;
	arm64_iomb();
	return (__builtin_bswap16(v));
}

static __inline u_int32_t
__inlrb(volatile u_int32_t *a)
{
	u_int32_t v;

	v = *a;
	arm64_iomb();
	return (__builtin_bswap32(v));
}

#define	out8rb(a,v)	outb(a,v)
#define	outwrb(a,v)	(__outwrb((volatile u_int16_t *)(a), v))
#define	out16rb(a,v)	outwrb(a,v)
#define	outlrb(a,v)	(__outlrb((volatile u_int32_t *)(a), v))
#define	out32rb(a,v)	outlrb(a,v)
#define	in8rb(a)	inb(a)
#define	inwrb(a)	(__inwrb((volatile u_int16_t *)(a)))
#define	in16rb(a)	inwrb(a)
#define	inlrb(a)	(__inlrb((volatile u_int32_t *)(a)))
#define	in32rb(a)	inlrb(a)

/* Bulk accessors */
static __inline void
__outsb(volatile u_int8_t *a, const u_int8_t *s, size_t c)
{
	while (c--)
		*a = *s++;
	arm64_iomb();
}

static __inline void
__outsw(volatile u_int16_t *a, const u_int16_t *s, size_t c)
{
	while (c--)
		*a = *s++;
	arm64_iomb();
}

static __inline void
__outsl(volatile u_int32_t *a, const u_int32_t *s, size_t c)
{
	while (c--)
		*a = *s++;
	arm64_iomb();
}

static __inline void
__outsll(volatile u_int64_t *a, const u_int64_t *s, size_t c)
{
	while (c--)
		*a = *s++;
	arm64_iomb();
}

static __inline void
__insb(volatile u_int8_t *a, u_int8_t *d, size_t c)
{
	while (c--)
		*d++ = *a;
	arm64_iomb();
}

static __inline void
__insw(volatile u_int16_t *a, u_int16_t *d, size_t c)
{
	while (c--)
		*d++ = *a;
	arm64_iomb();
}

static __inline void
__insl(volatile u_int32_t *a, u_int32_t *d, size_t c)
{
	while (c--)
		*d++ = *a;
	arm64_iomb();
}

static __inline void
__insll(volatile u_int64_t *a, u_int64_t *d, size_t c)
{
	while (c--)
		*d++ = *a;
	arm64_iomb();
}

#define	outsb(a,s,c)	(__outsb((volatile u_int8_t *)(a), s, c))
#define	outs8(a,s,c)	outsb(a,s,c)
#define	outsw(a,s,c)	(__outsw((volatile u_int16_t *)(a), s, c))
#define	outs16(a,s,c)	outsw(a,s,c)
#define	outsl(a,s,c)	(__outsl((volatile u_int32_t *)(a), s, c))
#define	outs32(a,s,c)	outsl(a,s,c)
#define	outsll(a,s,c)	(__outsll((volatile u_int64_t *)(a), s, c))
#define	outs64(a,s,c)	outsll(a,s,c)
#define	insb(a,d,c)	(__insb((volatile u_int8_t *)(a), d, c))
#define	ins8(a,d,c)	insb(a,d,c)
#define	insw(a,d,c)	(__insw((volatile u_int16_t *)(a), d, c))
#define	ins16(a,d,c)	insw(a,d,c)
#define	insl(a,d,c)	(__insl((volatile u_int32_t *)(a), d, c))
#define	ins32(a,d,c)	insl(a,d,c)
#define	insll(a,d,c)	(__insll((volatile u_int64_t *)(a), d, c))
#define	ins64(a,d,c)	insll(a,d,c)

#endif /* _MACHINE_PIO_H_ */
