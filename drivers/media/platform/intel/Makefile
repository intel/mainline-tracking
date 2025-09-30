# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2010 - 2022 Intel Corporation.

is_kernel_lt_6_10 = $(shell if [ $$(printf "6.10\n$(KERNELVERSION)" | sort -V | head -n1) != "6.10" ]; then echo 1; fi)
ifeq ($(is_kernel_lt_6_10), 1)
ifneq ($(EXTERNAL_BUILD), 1)
src := $(srctree)/$(src)
endif
endif

ccflags-y += -I$(src)/../../pci/intel/ipu7/

ifneq ($(filter y m, $(CONFIG_INTEL_IPU7_ACPI)),)
obj-$(CONFIG_INTEL_IPU7_ACPI) += ipu-acpi.o \
                                 ipu-acpi-pdata.o \
                                 ipu-acpi-common.o
else
obj-y += ipu7-fpga-pdata.o
endif
