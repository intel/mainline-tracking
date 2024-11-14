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
	SMPL_REG(SSP, PERF_REG_X86_SSP),

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

	SMPL_REG2_EXT(YMMH0, PERF_REG_X86_YMMH0),
	SMPL_REG2_EXT(YMMH1, PERF_REG_X86_YMMH1),
	SMPL_REG2_EXT(YMMH2, PERF_REG_X86_YMMH2),
	SMPL_REG2_EXT(YMMH3, PERF_REG_X86_YMMH3),
	SMPL_REG2_EXT(YMMH4, PERF_REG_X86_YMMH4),
	SMPL_REG2_EXT(YMMH5, PERF_REG_X86_YMMH5),
	SMPL_REG2_EXT(YMMH6, PERF_REG_X86_YMMH6),
	SMPL_REG2_EXT(YMMH7, PERF_REG_X86_YMMH7),
	SMPL_REG2_EXT(YMMH8, PERF_REG_X86_YMMH8),
	SMPL_REG2_EXT(YMMH9, PERF_REG_X86_YMMH9),
	SMPL_REG2_EXT(YMMH10, PERF_REG_X86_YMMH10),
	SMPL_REG2_EXT(YMMH11, PERF_REG_X86_YMMH11),
	SMPL_REG2_EXT(YMMH12, PERF_REG_X86_YMMH12),
	SMPL_REG2_EXT(YMMH13, PERF_REG_X86_YMMH13),
	SMPL_REG2_EXT(YMMH14, PERF_REG_X86_YMMH14),
	SMPL_REG2_EXT(YMMH15, PERF_REG_X86_YMMH15),

	SMPL_REG4_EXT(ZMMH0, PERF_REG_X86_ZMMH0),
	SMPL_REG4_EXT(ZMMH1, PERF_REG_X86_ZMMH1),
	SMPL_REG4_EXT(ZMMH2, PERF_REG_X86_ZMMH2),
	SMPL_REG4_EXT(ZMMH3, PERF_REG_X86_ZMMH3),
	SMPL_REG4_EXT(ZMMH4, PERF_REG_X86_ZMMH4),
	SMPL_REG4_EXT(ZMMH5, PERF_REG_X86_ZMMH5),
	SMPL_REG4_EXT(ZMMH6, PERF_REG_X86_ZMMH6),
	SMPL_REG4_EXT(ZMMH7, PERF_REG_X86_ZMMH7),
	SMPL_REG4_EXT(ZMMH8, PERF_REG_X86_ZMMH8),
	SMPL_REG4_EXT(ZMMH9, PERF_REG_X86_ZMMH9),
	SMPL_REG4_EXT(ZMMH10, PERF_REG_X86_ZMMH10),
	SMPL_REG4_EXT(ZMMH11, PERF_REG_X86_ZMMH11),
	SMPL_REG4_EXT(ZMMH12, PERF_REG_X86_ZMMH12),
	SMPL_REG4_EXT(ZMMH13, PERF_REG_X86_ZMMH13),
	SMPL_REG4_EXT(ZMMH14, PERF_REG_X86_ZMMH14),
	SMPL_REG4_EXT(ZMMH15, PERF_REG_X86_ZMMH15),

	SMPL_REG8_EXT(ZMM16, PERF_REG_X86_ZMM16),
	SMPL_REG8_EXT(ZMM17, PERF_REG_X86_ZMM17),
	SMPL_REG8_EXT(ZMM18, PERF_REG_X86_ZMM18),
	SMPL_REG8_EXT(ZMM19, PERF_REG_X86_ZMM19),
	SMPL_REG8_EXT(ZMM20, PERF_REG_X86_ZMM20),
	SMPL_REG8_EXT(ZMM21, PERF_REG_X86_ZMM21),
	SMPL_REG8_EXT(ZMM22, PERF_REG_X86_ZMM22),
	SMPL_REG8_EXT(ZMM23, PERF_REG_X86_ZMM23),
	SMPL_REG8_EXT(ZMM24, PERF_REG_X86_ZMM24),
	SMPL_REG8_EXT(ZMM25, PERF_REG_X86_ZMM25),
	SMPL_REG8_EXT(ZMM26, PERF_REG_X86_ZMM26),
	SMPL_REG8_EXT(ZMM27, PERF_REG_X86_ZMM27),
	SMPL_REG8_EXT(ZMM28, PERF_REG_X86_ZMM28),
	SMPL_REG8_EXT(ZMM29, PERF_REG_X86_ZMM29),
	SMPL_REG8_EXT(ZMM30, PERF_REG_X86_ZMM30),
	SMPL_REG8_EXT(ZMM31, PERF_REG_X86_ZMM31),

	SMPL_REG_EXT(OPMASK0, PERF_REG_X86_OPMASK0),
	SMPL_REG_EXT(OPMASK1, PERF_REG_X86_OPMASK1),
	SMPL_REG_EXT(OPMASK2, PERF_REG_X86_OPMASK2),
	SMPL_REG_EXT(OPMASK3, PERF_REG_X86_OPMASK3),
	SMPL_REG_EXT(OPMASK4, PERF_REG_X86_OPMASK4),
	SMPL_REG_EXT(OPMASK5, PERF_REG_X86_OPMASK5),
	SMPL_REG_EXT(OPMASK6, PERF_REG_X86_OPMASK6),
	SMPL_REG_EXT(OPMASK7, PERF_REG_X86_OPMASK7),

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

const struct sample_reg *arch__sample_reg_masks(void)
{
	return sample_reg_masks;
}

static void check_intr_reg_ext_mask(struct perf_event_attr *attr, int idx,
				    u64 fmask, unsigned long *mask)
{
	u64 src_mask[PERF_NUM_INTR_REGS] = { 0 };
	int fd;

	attr->sample_regs_intr = 0;
	attr->sample_regs_intr_ext[idx] = fmask;
	src_mask[idx + 1] = fmask;

	fd = sys_perf_event_open(attr, 0, -1, -1, 0);
	if (fd != -1) {
		close(fd);
		bitmap_or(mask, mask, (unsigned long *)src_mask,
			  PERF_NUM_INTR_REGS * 64);
	}
}

#define PERF_REG_EXTENDED_YMMH_MASK	GENMASK_ULL(31, 0)
#define PERF_REG_EXTENDED_ZMMH_1ST_MASK	GENMASK_ULL(63, 32)
#define PERF_REG_EXTENDED_ZMMH_2ND_MASK	GENMASK_ULL(31, 0)
#define PERF_REG_EXTENDED_ZMM_1ST_MASK	GENMASK_ULL(63, 32)
#define PERF_REG_EXTENDED_ZMM_2ND_MASK	GENMASK_ULL(63, 0)
#define PERF_REG_EXTENDED_ZMM_3RD_MASK	GENMASK_ULL(31, 0)
#define PERF_REG_EXTENDED_OPMASK_MASK	GENMASK_ULL(39, 32)

void arch__intr_reg_mask(unsigned long *mask)
{
	struct perf_event_attr attr = {
		.type			= PERF_TYPE_HARDWARE,
		.config			= PERF_COUNT_HW_CPU_CYCLES,
		.sample_type		= PERF_SAMPLE_REGS_INTR,
		.sample_regs_intr	= PERF_REG_EXTENDED_MASK,
		.precise_ip		= 1,
		.disabled 		= 1,
		.exclude_kernel		= 1,
	};
	int fd;

	*(u64 *)mask = PERF_REGS_MASK;

	/*
	 * In an unnamed union, init it here to build on older gcc versions
	 */
	attr.sample_period = 1;

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
		*(u64 *)mask |= PERF_REG_EXTENDED_MASK;
	}

	/* Check YMMH regs */
	check_intr_reg_ext_mask(&attr, 0, PERF_REG_EXTENDED_YMMH_MASK, mask);
	/* Check ZMMLH0-15 regs */
	check_intr_reg_ext_mask(&attr, 0, PERF_REG_EXTENDED_ZMMH_1ST_MASK, mask);
	check_intr_reg_ext_mask(&attr, 1, PERF_REG_EXTENDED_ZMMH_2ND_MASK, mask);
	/* Check ZMM16-31 regs */
	check_intr_reg_ext_mask(&attr, 1, PERF_REG_EXTENDED_ZMM_1ST_MASK, mask);
	check_intr_reg_ext_mask(&attr, 2, PERF_REG_EXTENDED_ZMM_2ND_MASK, mask);
	check_intr_reg_ext_mask(&attr, 3, PERF_REG_EXTENDED_ZMM_3RD_MASK, mask);
	/* Check OPMASK regs */
	check_intr_reg_ext_mask(&attr, 3, PERF_REG_EXTENDED_OPMASK_MASK, mask);
}

uint64_t arch__user_reg_mask(void)
{
	return PERF_REGS_MASK;
}
