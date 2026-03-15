.PATH:	${SRCTOP}/sys/dev/sff

KMOD=	sff

SRCS+=	sff_if.c sff_if.h
SRCS+=	bus_if.h device_if.h

.if !empty(OPT_FDT)
SRCS+=	sfp_fdt.c sfp_fdt.h \
	ofw_bus_if.h \
	gpio_if.h iicbus_if.h
.endif

.include <bsd.kmod.mk>
