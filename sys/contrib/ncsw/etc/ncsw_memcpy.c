/*
 * Wrapper to compile ncsw memcpy.c as ncsw_memcpy.o, avoiding
 * a name collision with arm64/arm64/memcpy.S (which also produces
 * memcpy.o and defines memcpy/memmove).
 */
#include "memcpy.c"
