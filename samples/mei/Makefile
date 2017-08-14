# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2012-2019, Intel Corporation. All rights reserved.

userprogs := mei-amt-version
always-y := $(userprogs)

userccflags += -I usr/include
obj-$(CONFIG_SAMPLE_INTEL_MEI_DAL_TEST) := dal_test.o


HOSTCFLAGS_mei-amt-version.o += -I$(objtree)/usr/include

always-y := $(hostprogs)

all: mei-amt-version
