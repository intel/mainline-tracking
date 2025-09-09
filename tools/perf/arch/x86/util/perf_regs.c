// SPDX-License-Identifier: GPL-2.0
#include <errno.h>
#include <string.h>
#include <regex.h>
#include <linux/kernel.h>
#include <linux/zalloc.h>

#include "perf_regs.h"
#include "../../../perf-sys.h"
#include "../../../util/perf_regs.h"
#include "../../../util/debug.h"
#include "../../../util/event.h"
#include "../../../util/pmu.h"
#include "../../../util/pmus.h"

static const struct sample_reg sample_reg_masks_ext[] = {
	SMPL_REG(AX, PERF_REG_X86_AX),
	SMPL_REG(BX, PERF_REG_X86_BX),
	SMPL_REG(CX, PERF_REG_X86_CX),
	SMPL_REG(DX, PERF_REG_X86_DX),
	SMPL_REG(SI, PERF_REG_X86_SI),
	SMPL_REG(DI, PERF_REG_X86_DI),
	SMPL_REG(BP, PERF_REG_X86_BP),
	SMPL_REG(SP, PERF_REG_X86_SP),
	SMPL_REG(IP, PERF_REG_X86_IP),
	SMPL_REG(FLAGS, PERF_REG_X86_FLAGS),
	SMPL_REG(CS, PERF_REG_X86_CS),
	SMPL_REG(SS, PERF_REG_X86_SS),
#ifdef HAVE_ARCH_X86_64_SUPPORT
	SMPL_REG(R8, PERF_REG_X86_R8),
	SMPL_REG(R9, PERF_REG_X86_R9),
	SMPL_REG(R10, PERF_REG_X86_R10),
	SMPL_REG(R11, PERF_REG_X86_R11),
	SMPL_REG(R12, PERF_REG_X86_R12),
	SMPL_REG(R13, PERF_REG_X86_R13),
	SMPL_REG(R14, PERF_REG_X86_R14),
	SMPL_REG(R15, PERF_REG_X86_R15),
	SMPL_REG(R16, PERF_REG_X86_R16),
	SMPL_REG(R17, PERF_REG_X86_R17),
	SMPL_REG(R18, PERF_REG_X86_R18),
	SMPL_REG(R19, PERF_REG_X86_R19),
	SMPL_REG(R20, PERF_REG_X86_R20),
	SMPL_REG(R21, PERF_REG_X86_R21),
	SMPL_REG(R22, PERF_REG_X86_R22),
	SMPL_REG(R23, PERF_REG_X86_R23),
	SMPL_REG(R24, PERF_REG_X86_R24),
	SMPL_REG(R25, PERF_REG_X86_R25),
	SMPL_REG(R26, PERF_REG_X86_R26),
	SMPL_REG(R27, PERF_REG_X86_R27),
	SMPL_REG(R28, PERF_REG_X86_R28),
	SMPL_REG(R29, PERF_REG_X86_R29),
	SMPL_REG(R30, PERF_REG_X86_R30),
	SMPL_REG(R31, PERF_REG_X86_R31),
	SMPL_REG(SSP, PERF_REG_X86_SSP),
#endif
	SMPL_REG_END
};

static const struct sample_reg sample_reg_masks[] = {
	SMPL_REG(AX, PERF_REG_X86_AX),
	SMPL_REG(BX, PERF_REG_X86_BX),
	SMPL_REG(CX, PERF_REG_X86_CX),
	SMPL_REG(DX, PERF_REG_X86_DX),
	SMPL_REG(SI, PERF_REG_X86_SI),
	SMPL_REG(DI, PERF_REG_X86_DI),
	SMPL_REG(BP, PERF_REG_X86_BP),
	SMPL_REG(SP, PERF_REG_X86_SP),
	SMPL_REG(IP, PERF_REG_X86_IP),
	SMPL_REG(FLAGS, PERF_REG_X86_FLAGS),
	SMPL_REG(CS, PERF_REG_X86_CS),
	SMPL_REG(SS, PERF_REG_X86_SS),
#ifdef HAVE_ARCH_X86_64_SUPPORT
	SMPL_REG(R8, PERF_REG_X86_R8),
	SMPL_REG(R9, PERF_REG_X86_R9),
	SMPL_REG(R10, PERF_REG_X86_R10),
	SMPL_REG(R11, PERF_REG_X86_R11),
	SMPL_REG(R12, PERF_REG_X86_R12),
	SMPL_REG(R13, PERF_REG_X86_R13),
	SMPL_REG(R14, PERF_REG_X86_R14),
	SMPL_REG(R15, PERF_REG_X86_R15),
#endif
	SMPL_REG2(XMM0, PERF_REG_X86_XMM0),
	SMPL_REG2(XMM1, PERF_REG_X86_XMM1),
	SMPL_REG2(XMM2, PERF_REG_X86_XMM2),
	SMPL_REG2(XMM3, PERF_REG_X86_XMM3),
	SMPL_REG2(XMM4, PERF_REG_X86_XMM4),
	SMPL_REG2(XMM5, PERF_REG_X86_XMM5),
	SMPL_REG2(XMM6, PERF_REG_X86_XMM6),
	SMPL_REG2(XMM7, PERF_REG_X86_XMM7),
	SMPL_REG2(XMM8, PERF_REG_X86_XMM8),
	SMPL_REG2(XMM9, PERF_REG_X86_XMM9),
	SMPL_REG2(XMM10, PERF_REG_X86_XMM10),
	SMPL_REG2(XMM11, PERF_REG_X86_XMM11),
	SMPL_REG2(XMM12, PERF_REG_X86_XMM12),
	SMPL_REG2(XMM13, PERF_REG_X86_XMM13),
	SMPL_REG2(XMM14, PERF_REG_X86_XMM14),
	SMPL_REG2(XMM15, PERF_REG_X86_XMM15),
	SMPL_REG_END
};

struct sdt_name_reg {
	const char *sdt_name;
	const char *uprobe_name;
};
#define SDT_NAME_REG(n, m) {.sdt_name = "%" #n, .uprobe_name = "%" #m}
#define SDT_NAME_REG_END {.sdt_name = NULL, .uprobe_name = NULL}

static const struct sdt_name_reg sdt_reg_tbl[] = {
	SDT_NAME_REG(eax, ax),
	SDT_NAME_REG(rax, ax),
	SDT_NAME_REG(al,  ax),
	SDT_NAME_REG(ah,  ax),
	SDT_NAME_REG(ebx, bx),
	SDT_NAME_REG(rbx, bx),
	SDT_NAME_REG(bl,  bx),
	SDT_NAME_REG(bh,  bx),
	SDT_NAME_REG(ecx, cx),
	SDT_NAME_REG(rcx, cx),
	SDT_NAME_REG(cl,  cx),
	SDT_NAME_REG(ch,  cx),
	SDT_NAME_REG(edx, dx),
	SDT_NAME_REG(rdx, dx),
	SDT_NAME_REG(dl,  dx),
	SDT_NAME_REG(dh,  dx),
	SDT_NAME_REG(esi, si),
	SDT_NAME_REG(rsi, si),
	SDT_NAME_REG(sil, si),
	SDT_NAME_REG(edi, di),
	SDT_NAME_REG(rdi, di),
	SDT_NAME_REG(dil, di),
	SDT_NAME_REG(ebp, bp),
	SDT_NAME_REG(rbp, bp),
	SDT_NAME_REG(bpl, bp),
	SDT_NAME_REG(rsp, sp),
	SDT_NAME_REG(esp, sp),
	SDT_NAME_REG(spl, sp),

	/* rNN registers */
	SDT_NAME_REG(r8b,  r8),
	SDT_NAME_REG(r8w,  r8),
	SDT_NAME_REG(r8d,  r8),
	SDT_NAME_REG(r9b,  r9),
	SDT_NAME_REG(r9w,  r9),
	SDT_NAME_REG(r9d,  r9),
	SDT_NAME_REG(r10b, r10),
	SDT_NAME_REG(r10w, r10),
	SDT_NAME_REG(r10d, r10),
	SDT_NAME_REG(r11b, r11),
	SDT_NAME_REG(r11w, r11),
	SDT_NAME_REG(r11d, r11),
	SDT_NAME_REG(r12b, r12),
	SDT_NAME_REG(r12w, r12),
	SDT_NAME_REG(r12d, r12),
	SDT_NAME_REG(r13b, r13),
	SDT_NAME_REG(r13w, r13),
	SDT_NAME_REG(r13d, r13),
	SDT_NAME_REG(r14b, r14),
	SDT_NAME_REG(r14w, r14),
	SDT_NAME_REG(r14d, r14),
	SDT_NAME_REG(r15b, r15),
	SDT_NAME_REG(r15w, r15),
	SDT_NAME_REG(r15d, r15),
	SDT_NAME_REG_END,
};

/*
 * Perf only supports OP which is in  +/-NUM(REG)  form.
 * Here plus-minus sign, NUM and parenthesis are optional,
 * only REG is mandatory.
 *
 * SDT events also supports indirect addressing mode with a
 * symbol as offset, scaled mode and constants in OP. But
 * perf does not support them yet. Below are few examples.
 *
 * OP with scaled mode:
 *     (%rax,%rsi,8)
 *     10(%ras,%rsi,8)
 *
 * OP with indirect addressing mode:
 *     check_action(%rip)
 *     mp_+52(%rip)
 *     44+mp_(%rip)
 *
 * OP with constant values:
 *     $0
 *     $123
 *     $-1
 */
#define SDT_OP_REGEX  "^([+\\-]?)([0-9]*)(\\(?)(%[a-z][a-z0-9]+)(\\)?)$"

static regex_t sdt_op_regex;

static int sdt_init_op_regex(void)
{
	static int initialized;
	int ret = 0;

	if (initialized)
		return 0;

	ret = regcomp(&sdt_op_regex, SDT_OP_REGEX, REG_EXTENDED);
	if (ret < 0) {
		pr_debug4("Regex compilation error.\n");
		return ret;
	}

	initialized = 1;
	return 0;
}

/*
 * Max x86 register name length is 5(ex: %r15d). So, 6th char
 * should always contain NULL. This helps to find register name
 * length using strlen, instead of maintaining one more variable.
 */
#define SDT_REG_NAME_SIZE  6

/*
 * The uprobe parser does not support all gas register names;
 * so, we have to replace them (ex. for x86_64: %rax -> %ax).
 * Note: If register does not require renaming, just copy
 * paste as it is, but don't leave it empty.
 */
static void sdt_rename_register(char *sdt_reg, int sdt_len, char *uprobe_reg)
{
	int i = 0;

	for (i = 0; sdt_reg_tbl[i].sdt_name != NULL; i++) {
		if (!strncmp(sdt_reg_tbl[i].sdt_name, sdt_reg, sdt_len)) {
			strcpy(uprobe_reg, sdt_reg_tbl[i].uprobe_name);
			return;
		}
	}

	strncpy(uprobe_reg, sdt_reg, sdt_len);
}

int arch_sdt_arg_parse_op(char *old_op, char **new_op)
{
	char new_reg[SDT_REG_NAME_SIZE] = {0};
	int new_len = 0, ret;
	/*
	 * rm[0]:  +/-NUM(REG)
	 * rm[1]:  +/-
	 * rm[2]:  NUM
	 * rm[3]:  (
	 * rm[4]:  REG
	 * rm[5]:  )
	 */
	regmatch_t rm[6];
	/*
	 * Max prefix length is 2 as it may contains sign(+/-)
	 * and displacement 0 (Both sign and displacement 0 are
	 * optional so it may be empty). Use one more character
	 * to hold last NULL so that strlen can be used to find
	 * prefix length, instead of maintaining one more variable.
	 */
	char prefix[3] = {0};

	ret = sdt_init_op_regex();
	if (ret < 0)
		return ret;

	/*
	 * If unsupported OR does not match with regex OR
	 * register name too long, skip it.
	 */
	if (strchr(old_op, ',') || strchr(old_op, '$') ||
	    regexec(&sdt_op_regex, old_op, 6, rm, 0)   ||
	    rm[4].rm_eo - rm[4].rm_so > SDT_REG_NAME_SIZE) {
		pr_debug4("Skipping unsupported SDT argument: %s\n", old_op);
		return SDT_ARG_SKIP;
	}

	/*
	 * Prepare prefix.
	 * If SDT OP has parenthesis but does not provide
	 * displacement, add 0 for displacement.
	 *     SDT         Uprobe     Prefix
	 *     -----------------------------
	 *     +24(%rdi)   +24(%di)   +
	 *     24(%rdi)    +24(%di)   +
	 *     %rdi        %di
	 *     (%rdi)      +0(%di)    +0
	 *     -80(%rbx)   -80(%bx)   -
	 */
	if (rm[3].rm_so != rm[3].rm_eo) {
		if (rm[1].rm_so != rm[1].rm_eo)
			prefix[0] = *(old_op + rm[1].rm_so);
		else if (rm[2].rm_so != rm[2].rm_eo)
			prefix[0] = '+';
		else
			scnprintf(prefix, sizeof(prefix), "+0");
	}

	/* Rename register */
	sdt_rename_register(old_op + rm[4].rm_so, rm[4].rm_eo - rm[4].rm_so,
			    new_reg);

	/* Prepare final OP which should be valid for uprobe_events */
	new_len = strlen(prefix)              +
		  (rm[2].rm_eo - rm[2].rm_so) +
		  (rm[3].rm_eo - rm[3].rm_so) +
		  strlen(new_reg)             +
		  (rm[5].rm_eo - rm[5].rm_so) +
		  1;					/* NULL */

	*new_op = zalloc(new_len);
	if (!*new_op)
		return -ENOMEM;

	scnprintf(*new_op, new_len, "%.*s%.*s%.*s%.*s%.*s",
		  strlen(prefix), prefix,
		  (int)(rm[2].rm_eo - rm[2].rm_so), old_op + rm[2].rm_so,
		  (int)(rm[3].rm_eo - rm[3].rm_so), old_op + rm[3].rm_so,
		  strlen(new_reg), new_reg,
		  (int)(rm[5].rm_eo - rm[5].rm_so), old_op + rm[5].rm_so);

	return SDT_ARG_VALID;
}

static bool support_simd_reg(u64 sample_type, u16 qwords, u64 mask, bool pred)
{
	struct perf_event_attr attr = {
		.type				= PERF_TYPE_HARDWARE,
		.config				= PERF_COUNT_HW_CPU_CYCLES,
		.sample_type			= sample_type,
		.disabled 			= 1,
		.exclude_kernel			= 1,
		.sample_simd_regs_enabled	= 1,
	};
	int fd;

	attr.sample_period = 1;

	if (!pred) {
		attr.sample_simd_vec_reg_qwords = qwords;
		if (sample_type == PERF_SAMPLE_REGS_INTR)
			attr.sample_simd_vec_reg_intr = mask;
		else
			attr.sample_simd_vec_reg_user = mask;
	} else {
		attr.sample_simd_pred_reg_qwords = PERF_X86_OPMASK_QWORDS;
		if (sample_type == PERF_SAMPLE_REGS_INTR)
			attr.sample_simd_pred_reg_intr = PERF_X86_SIMD_PRED_MASK;
		else
			attr.sample_simd_pred_reg_user = PERF_X86_SIMD_PRED_MASK;
	}

	if (perf_pmus__num_core_pmus() > 1) {
		struct perf_pmu *pmu = NULL;
		__u64 type = PERF_TYPE_RAW;

		/*
		 * The same register set is supported among different hybrid PMUs.
		 * Only check the first available one.
		 */
		while ((pmu = perf_pmus__scan_core(pmu)) != NULL) {
			type = pmu->type;
			break;
		}
		attr.config |= type << PERF_PMU_TYPE_SHIFT;
	}

	event_attr_init(&attr);

	fd = sys_perf_event_open(&attr, 0, -1, -1, 0);
	if (fd != -1) {
		close(fd);
		return true;
	}

	return false;
}

static bool __arch_simd_reg_mask(u64 sample_type, int reg, uint64_t *mask, u16 *qwords)
{
	bool supported = false;
	u64 bits;

	*mask = 0;
	*qwords = 0;

	switch (reg) {
	case PERF_REG_X86_XMM:
		bits = BIT_ULL(PERF_X86_SIMD_XMM_REGS) - 1;
		supported = support_simd_reg(sample_type, PERF_X86_XMM_QWORDS, bits, false);
		if (supported) {
			*mask = bits;
			*qwords = PERF_X86_XMM_QWORDS;
		}
		break;
	case PERF_REG_X86_YMM:
		bits = BIT_ULL(PERF_X86_SIMD_YMM_REGS) - 1;
		supported = support_simd_reg(sample_type, PERF_X86_YMM_QWORDS, bits, false);
		if (supported) {
			*mask = bits;
			*qwords = PERF_X86_YMM_QWORDS;
		}
		break;
	case PERF_REG_X86_ZMM:
		bits = BIT_ULL(PERF_X86_SIMD_ZMM_REGS) - 1;
		supported = support_simd_reg(sample_type, PERF_X86_ZMM_QWORDS, bits, false);
		if (supported) {
			*mask = bits;
			*qwords = PERF_X86_ZMM_QWORDS;
			break;
		}

		bits = BIT_ULL(PERF_X86_SIMD_ZMMH_REGS) - 1;
		supported = support_simd_reg(sample_type, PERF_X86_ZMM_QWORDS, bits, false);
		if (supported) {
			*mask = bits;
			*qwords = PERF_X86_ZMMH_QWORDS;
		}
		break;
	default:
		break;
	}

	return supported;
}

static bool __arch_pred_reg_mask(u64 sample_type, int reg, uint64_t *mask, u16 *qwords)
{
	bool supported = false;
	u64 bits;

	*mask = 0;
	*qwords = 0;

	switch (reg) {
	case PERF_REG_X86_OPMASK:
		bits = BIT_ULL(PERF_X86_SIMD_OPMASK_REGS) - 1;
		supported = support_simd_reg(sample_type, PERF_X86_OPMASK_QWORDS, bits, true);
		if (supported) {
			*mask = bits;
			*qwords = PERF_X86_OPMASK_QWORDS;
		}
		break;
	default:
		break;
	}

	return supported;
}

static bool has_cap_simd_regs(void)
{
	uint64_t mask = BIT_ULL(PERF_X86_SIMD_XMM_REGS) - 1;
	u16 qwords = PERF_X86_XMM_QWORDS;
	static bool has_cap_simd_regs;
	static bool cached = false;

	if (cached)
		return has_cap_simd_regs;

	has_cap_simd_regs = __arch_simd_reg_mask(PERF_SAMPLE_REGS_INTR,
						 PERF_REG_X86_XMM, &mask, &qwords);
	has_cap_simd_regs |= __arch_simd_reg_mask(PERF_SAMPLE_REGS_USER,
						 PERF_REG_X86_XMM, &mask, &qwords);
	cached = true;

	return has_cap_simd_regs;
}


static const struct sample_reg sample_simd_reg_masks[] = {
	SMPL_REG(XMM, PERF_REG_X86_XMM),
	SMPL_REG(YMM, PERF_REG_X86_YMM),
	SMPL_REG(ZMM, PERF_REG_X86_ZMM),
	SMPL_REG_END
};

static const struct sample_reg sample_pred_reg_masks[] = {
	SMPL_REG(OPMASK, PERF_REG_X86_OPMASK),
	SMPL_REG_END
};

const struct sample_reg *arch__sample_simd_reg_masks(void)
{
	return sample_simd_reg_masks;
}

const struct sample_reg *arch__sample_pred_reg_masks(void)
{
	return sample_pred_reg_masks;
}

static bool x86_intr_simd_updated;
static u64 x86_intr_simd_mask[PERF_REG_X86_MAX_SIMD_REGS];
static u16 x86_intr_simd_qwords[PERF_REG_X86_MAX_SIMD_REGS];
static bool x86_user_simd_updated;
static u64 x86_user_simd_mask[PERF_REG_X86_MAX_SIMD_REGS];
static u16 x86_user_simd_qwords[PERF_REG_X86_MAX_SIMD_REGS];

static bool x86_intr_pred_updated;
static u64 x86_intr_pred_mask[PERF_REG_X86_MAX_PRED_REGS];
static u16 x86_intr_pred_qwords[PERF_REG_X86_MAX_PRED_REGS];
static bool x86_user_pred_updated;
static u64 x86_user_pred_mask[PERF_REG_X86_MAX_PRED_REGS];
static u16 x86_user_pred_qwords[PERF_REG_X86_MAX_PRED_REGS];

static uint64_t __arch__simd_reg_mask(u64 sample_type)
{
	const struct sample_reg *r = NULL;
	bool supported;
	u64 mask = 0;
	int reg;

	if (!has_cap_simd_regs())
		return 0;

	for (r = arch__sample_simd_reg_masks(); r->name; r++) {
		supported = false;

		if (!r->mask)
			continue;
		reg = fls64(r->mask) - 1;

		if (reg >= PERF_REG_X86_MAX_SIMD_REGS)
			break;
		if (sample_type == PERF_SAMPLE_REGS_INTR)
			supported = __arch_simd_reg_mask(sample_type, reg,
							 &x86_intr_simd_mask[reg],
							 &x86_intr_simd_qwords[reg]);
		else if (sample_type == PERF_SAMPLE_REGS_USER)
			supported = __arch_simd_reg_mask(sample_type, reg,
							 &x86_user_simd_mask[reg],
							 &x86_user_simd_qwords[reg]);
		if (supported)
			mask |= BIT_ULL(reg);
	}

	if (sample_type == PERF_SAMPLE_REGS_INTR)
		x86_intr_simd_updated = true;
	else
		x86_user_simd_updated = true;

	return mask;
}

static uint64_t __arch__pred_reg_mask(u64 sample_type)
{
	const struct sample_reg *r = NULL;
	bool supported;
	u64 mask = 0;
	int reg;

	if (!has_cap_simd_regs())
		return 0;

	for (r = arch__sample_pred_reg_masks(); r->name; r++) {
		supported = false;

		if (!r->mask)
			continue;
		reg = fls64(r->mask) - 1;

		if (reg >= PERF_REG_X86_MAX_PRED_REGS)
			break;
		if (sample_type == PERF_SAMPLE_REGS_INTR)
			supported = __arch_pred_reg_mask(sample_type, reg,
							 &x86_intr_pred_mask[reg],
							 &x86_intr_pred_qwords[reg]);
		else if (sample_type == PERF_SAMPLE_REGS_USER)
			supported = __arch_pred_reg_mask(sample_type, reg,
							 &x86_user_pred_mask[reg],
							 &x86_user_pred_qwords[reg]);
		if (supported)
			mask |= BIT_ULL(reg);
	}

	if (sample_type == PERF_SAMPLE_REGS_INTR)
		x86_intr_pred_updated = true;
	else
		x86_user_pred_updated = true;

	return mask;
}

uint64_t arch__intr_simd_reg_mask(void)
{
	return __arch__simd_reg_mask(PERF_SAMPLE_REGS_INTR);
}

uint64_t arch__user_simd_reg_mask(void)
{
	return __arch__simd_reg_mask(PERF_SAMPLE_REGS_USER);
}

uint64_t arch__intr_pred_reg_mask(void)
{
	return __arch__pred_reg_mask(PERF_SAMPLE_REGS_INTR);
}

uint64_t arch__user_pred_reg_mask(void)
{
	return __arch__pred_reg_mask(PERF_SAMPLE_REGS_USER);
}

static uint64_t arch__simd_reg_bitmap_qwords(int reg, u16 *qwords, bool intr)
{
	uint64_t mask = 0;

	*qwords = 0;
	if (reg < PERF_REG_X86_MAX_SIMD_REGS) {
		if (intr) {
			*qwords = x86_intr_simd_qwords[reg];
			mask = x86_intr_simd_mask[reg];
		} else {
			*qwords = x86_user_simd_qwords[reg];
			mask = x86_user_simd_mask[reg];
		}
	}

	return mask;
}

static uint64_t arch__pred_reg_bitmap_qwords(int reg, u16 *qwords, bool intr)
{
	uint64_t mask = 0;

	*qwords = 0;
	if (reg < PERF_REG_X86_MAX_PRED_REGS) {
		if (intr) {
			*qwords = x86_intr_pred_qwords[reg];
			mask = x86_intr_pred_mask[reg];
		} else {
			*qwords = x86_user_pred_qwords[reg];
			mask = x86_user_pred_mask[reg];
		}
	}

	return mask;
}

uint64_t arch__intr_simd_reg_bitmap_qwords(int reg, u16 *qwords)
{
	if (!x86_intr_simd_updated)
		arch__intr_simd_reg_mask();
	return arch__simd_reg_bitmap_qwords(reg, qwords, true);
}

uint64_t arch__user_simd_reg_bitmap_qwords(int reg, u16 *qwords)
{
	if (!x86_user_simd_updated)
		arch__user_simd_reg_mask();
	return arch__simd_reg_bitmap_qwords(reg, qwords, false);
}

uint64_t arch__intr_pred_reg_bitmap_qwords(int reg, u16 *qwords)
{
	if (!x86_intr_pred_updated)
		arch__intr_pred_reg_mask();
	return arch__pred_reg_bitmap_qwords(reg, qwords, true);
}

uint64_t arch__user_pred_reg_bitmap_qwords(int reg, u16 *qwords)
{
	if (!x86_user_pred_updated)
		arch__user_pred_reg_mask();
	return arch__pred_reg_bitmap_qwords(reg, qwords, false);
}

const struct sample_reg *arch__sample_reg_masks(void)
{
	if (has_cap_simd_regs())
		return sample_reg_masks_ext;
	return sample_reg_masks;
}

static uint64_t __arch__reg_mask(u64 sample_type, u64 mask, bool has_simd_regs)
{
	struct perf_event_attr attr = {
		.type				= PERF_TYPE_HARDWARE,
		.config				= PERF_COUNT_HW_CPU_CYCLES,
		.sample_type			= sample_type,
		.precise_ip			= 1,
		.disabled 			= 1,
		.exclude_kernel			= 1,
		.sample_simd_regs_enabled	= has_simd_regs,
	};
	int fd;
	/*
	 * In an unnamed union, init it here to build on older gcc versions
	 */
	attr.sample_period = 1;
	if (sample_type == PERF_SAMPLE_REGS_INTR)
		attr.sample_regs_intr = mask;
	else
		attr.sample_regs_user = mask;

	if (perf_pmus__num_core_pmus() > 1) {
		struct perf_pmu *pmu = NULL;
		__u64 type = PERF_TYPE_RAW;

		/*
		 * The same register set is supported among different hybrid PMUs.
		 * Only check the first available one.
		 */
		while ((pmu = perf_pmus__scan_core(pmu)) != NULL) {
			type = pmu->type;
			break;
		}
		attr.config |= type << PERF_PMU_TYPE_SHIFT;
	}

	event_attr_init(&attr);

	fd = sys_perf_event_open(&attr, 0, -1, -1, 0);
	if (fd != -1) {
		close(fd);
		return mask;
	}

	return 0;
}

uint64_t arch__intr_reg_mask(void)
{
	uint64_t mask = PERF_REGS_MASK;

	if (has_cap_simd_regs()) {
		mask |= __arch__reg_mask(PERF_SAMPLE_REGS_INTR,
					 GENMASK_ULL(PERF_REG_X86_R31, PERF_REG_X86_R16),
					 true);
		mask |= __arch__reg_mask(PERF_SAMPLE_REGS_INTR,
					 BIT_ULL(PERF_REG_X86_SSP),
					 true);
	} else
		mask |= __arch__reg_mask(PERF_SAMPLE_REGS_INTR, PERF_REG_EXTENDED_MASK, false);

	return mask;
}

uint64_t arch__user_reg_mask(void)
{
	uint64_t mask = PERF_REGS_MASK;

	if (has_cap_simd_regs()) {
		mask |= __arch__reg_mask(PERF_SAMPLE_REGS_USER,
					 GENMASK_ULL(PERF_REG_X86_R31, PERF_REG_X86_R16),
					 true);
		mask |= __arch__reg_mask(PERF_SAMPLE_REGS_USER,
					 BIT_ULL(PERF_REG_X86_SSP),
					 true);
	}

	return mask;
}