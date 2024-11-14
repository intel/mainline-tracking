// SPDX-License-Identifier: GPL-2.0

#include "../perf_regs.h"
#include "../../../arch/x86/include/uapi/asm/perf_regs.h"

const char *__perf_reg_name_x86(int id)
{
	switch (id) {
	case PERF_REG_X86_AX:
		return "AX";
	case PERF_REG_X86_BX:
		return "BX";
	case PERF_REG_X86_CX:
		return "CX";
	case PERF_REG_X86_DX:
		return "DX";
	case PERF_REG_X86_SI:
		return "SI";
	case PERF_REG_X86_DI:
		return "DI";
	case PERF_REG_X86_BP:
		return "BP";
	case PERF_REG_X86_SP:
		return "SP";
	case PERF_REG_X86_IP:
		return "IP";
	case PERF_REG_X86_FLAGS:
		return "FLAGS";
	case PERF_REG_X86_CS:
		return "CS";
	case PERF_REG_X86_SS:
		return "SS";
	case PERF_REG_X86_DS:
		return "DS";
	case PERF_REG_X86_ES:
		return "ES";
	case PERF_REG_X86_FS:
		return "FS";
	case PERF_REG_X86_GS:
		return "GS";
	case PERF_REG_X86_R8:
		return "R8";
	case PERF_REG_X86_R9:
		return "R9";
	case PERF_REG_X86_R10:
		return "R10";
	case PERF_REG_X86_R11:
		return "R11";
	case PERF_REG_X86_R12:
		return "R12";
	case PERF_REG_X86_R13:
		return "R13";
	case PERF_REG_X86_R14:
		return "R14";
	case PERF_REG_X86_R15:
		return "R15";
	case PERF_REG_X86_SSP:
		return "SSP";

#define XMM(x) \
	case PERF_REG_X86_XMM ## x:	\
	case PERF_REG_X86_XMM ## x + 1:	\
		return "XMM" #x;
	XMM(0)
	XMM(1)
	XMM(2)
	XMM(3)
	XMM(4)
	XMM(5)
	XMM(6)
	XMM(7)
	XMM(8)
	XMM(9)
	XMM(10)
	XMM(11)
	XMM(12)
	XMM(13)
	XMM(14)
	XMM(15)
#undef XMM

#define YMM(x)					\
	case PERF_REG_X86_YMM ## x:		\
	case PERF_REG_X86_YMM ## x + 1:		\
	case PERF_REG_X86_YMM ## x + 2:		\
	case PERF_REG_X86_YMM ## x + 3:		\
		return "YMM" #x;
	YMM(0)
	YMM(1)
	YMM(2)
	YMM(3)
	YMM(4)
	YMM(5)
	YMM(6)
	YMM(7)
	YMM(8)
	YMM(9)
	YMM(10)
	YMM(11)
	YMM(12)
	YMM(13)
	YMM(14)
	YMM(15)
#undef YMM

#define ZMM(x)				\
	case PERF_REG_X86_ZMM ## x:		\
	case PERF_REG_X86_ZMM ## x + 1:	\
	case PERF_REG_X86_ZMM ## x + 2:	\
	case PERF_REG_X86_ZMM ## x + 3:	\
	case PERF_REG_X86_ZMM ## x + 4:	\
	case PERF_REG_X86_ZMM ## x + 5:	\
	case PERF_REG_X86_ZMM ## x + 6:	\
	case PERF_REG_X86_ZMM ## x + 7:	\
		return "ZMM" #x;
	ZMM(0)
	ZMM(1)
	ZMM(2)
	ZMM(3)
	ZMM(4)
	ZMM(5)
	ZMM(6)
	ZMM(7)
	ZMM(8)
	ZMM(9)
	ZMM(10)
	ZMM(11)
	ZMM(12)
	ZMM(13)
	ZMM(14)
	ZMM(15)
	ZMM(16)
	ZMM(17)
	ZMM(18)
	ZMM(19)
	ZMM(20)
	ZMM(21)
	ZMM(22)
	ZMM(23)
	ZMM(24)
	ZMM(25)
	ZMM(26)
	ZMM(27)
	ZMM(28)
	ZMM(29)
	ZMM(30)
	ZMM(31)
#undef ZMM

#define OPMASK(x)				\
	case PERF_REG_X86_OPMASK ## x:		\
		return "opmask" #x;

	OPMASK(0)
	OPMASK(1)
	OPMASK(2)
	OPMASK(3)
	OPMASK(4)
	OPMASK(5)
	OPMASK(6)
	OPMASK(7)
#undef OPMASK
	default:
		return NULL;
	}

	return NULL;
}

uint64_t __perf_reg_ip_x86(void)
{
	return PERF_REG_X86_IP;
}

uint64_t __perf_reg_sp_x86(void)
{
	return PERF_REG_X86_SP;
}
