obj-m         := usbfifo.o
usbfifo-objs  := mod_main.o

ARCH          := x86
CROSS_COMPILE :=

M  := $(shell pwd)
K  := $(shell uname -r)
KS := /lib/modules/$(K)/build

all:
	make -C $(KS) M=$(M) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules
