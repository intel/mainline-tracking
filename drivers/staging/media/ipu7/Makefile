# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2017 - 2025 Intel Corporation.

is_kernel_lt_6_10 = $(shell if [ $$(printf "6.10\n$(KERNELVERSION)" | sort -V | head -n1) != "6.10" ]; then echo 1; fi)
ifeq ($(is_kernel_lt_6_10), 1)
ifneq ($(EXTERNAL_BUILD), 1)
src := $(srctree)/$(src)
endif
endif

intel-ipu7-objs				+= ipu7.o \
					   ipu7-bus.o \
					   ipu7-dma.o \
					   ipu7-mmu.o \
					   ipu7-buttress.o \
					   ipu7-cpd.o \
					   ipu7-syscom.o \
					   ipu7-boot.o

obj-$(CONFIG_VIDEO_INTEL_IPU7)		+= intel-ipu7.o

intel-ipu7-isys-objs			+= ipu7-isys.o \
					   ipu7-isys-csi2.o \
					   ipu7-isys-csi-phy.o \
					   ipu7-fw-isys.o \
					   ipu7-isys-video.o \
					   ipu7-isys-queue.o \
					   ipu7-isys-subdev.o

obj-$(CONFIG_VIDEO_INTEL_IPU7)		+= intel-ipu7-isys.o

obj-$(CONFIG_VIDEO_INTEL_IPU7)		+= psys/

ccflags-y += -I$(src)/
