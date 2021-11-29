# SPDX-License-Identifier: GPL-2.0

CAN_BUILD_UINTR := $(shell ./../x86/check_cc.sh $(CC) ../x86/trivial_64bit_program.c -muintr)

ifeq ($(CAN_BUILD_UINTR),1)
CFLAGS += -g -muintr -mgeneral-regs-only
LDLIBS += -lpthread

TEST_GEN_PROGS = ipc_base \
	syscall_handler \
	syscall_vector \
	syscall_sender \
	syscall_flags \
	senduipi_instr

TEST_GEN_PROGS_EXTENDED = context_switch_ipc

endif

include ../lib.mk
