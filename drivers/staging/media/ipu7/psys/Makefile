# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2017 - 2024 Intel Corporation.

is_kernel_lt_6_10 = $(shell if [ $$(printf "6.10\n$(KERNELVERSION)" | sort -V | head -n1) != "6.10" ]; then echo 1; fi)
ifeq ($(is_kernel_lt_6_10), 1)
ifneq ($(EXTERNAL_BUILD), 1)
src := $(srctree)/$(src)
endif
endif

intel-ipu7-psys-objs		+= ipu-psys.o \
				ipu7-psys.o \
				ipu7-fw-psys.o

obj-$(CONFIG_VIDEO_INTEL_IPU7)	+= intel-ipu7-psys.o

ccflags-y += -I$(src)/
ccflags-y += -I$(src)/../
