/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PERF_REGS_H
#define _LINUX_PERF_REGS_H

#include <linux/sched/task_stack.h>

struct perf_regs {
	__u64		abi;
	struct pt_regs	*regs;
};

int perf_simd_reg_validate(u16 vec_qwords, u64 vec_mask,
			   u16 pred_qwords, u32 pred_mask);
u64 perf_simd_reg_value(struct pt_regs *regs, int idx,
			u16 qwords_idx, bool pred);
/*
 * Check and update the configuration of the requested SIMD registers
 *
 * regs: Used to locate the SIMD registers
 * ignore: A mask to ignore the check of some configuration
 * mask: The requested vector mask
 * nr_vectors: Number of the vector registers
 * vec_qwords: The QWORD of the vector registers
 * pred_mask: The requested predicate mask
 * nr_pred: Number of the predicate registers
 * pred_qwords: The QWORD of the predicate registers
 *
 * It's possible (e.g., ARM) that the number and width of the dumped
 * SIMD registers are a little different from the request.
 * The function is to calculate the real number and width before dumping
 * the data.
 */
void perf_simd_reg_check(struct pt_regs *regs, u64 ignore,
			 u64 mask, u16 *nr_vectors, u16 *vec_qwords,
			 u16 pred_mask, u16 *nr_pred, u16 *pred_qwords);


#ifdef CONFIG_HAVE_PERF_REGS
#include <asm/perf_regs.h>

#ifndef PERF_REG_EXTENDED_MASK
#define PERF_REG_EXTENDED_MASK	0
#endif

u64 perf_reg_value(struct pt_regs *regs, int idx);
int perf_reg_validate(u64 mask);
u64 perf_reg_abi(struct task_struct *task);
void perf_get_regs_user(struct perf_regs *regs_user,
			struct pt_regs *regs);
#else

#define PERF_REG_EXTENDED_MASK	0

static inline u64 perf_reg_value(struct pt_regs *regs, int idx)
{
	return 0;
}

static inline int perf_reg_validate(u64 mask)
{
	return mask ? -ENOSYS : 0;
}

static inline u64 perf_reg_abi(struct task_struct *task)
{
	return PERF_SAMPLE_REGS_ABI_NONE;
}

static inline void perf_get_regs_user(struct perf_regs *regs_user,
				      struct pt_regs *regs)
{
	regs_user->regs = task_pt_regs(current);
	regs_user->abi = perf_reg_abi(current);
}
#endif /* CONFIG_HAVE_PERF_REGS */
#endif /* _LINUX_PERF_REGS_H */
