/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PKS_H
#define _ASM_X86_PKS_H

#ifdef CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS

struct extended_pt_regs {
	u32 thread_pkrs;
	/* Keep stack 8 byte aligned */
	u32 pad;
	struct pt_regs pt_regs;
};

void setup_pks(void);

static inline struct extended_pt_regs *extended_pt_regs(struct pt_regs *regs)
{
	return container_of(regs, struct extended_pt_regs, pt_regs);
}

void show_extended_regs_oops(struct pt_regs *regs, unsigned error_code);

#else /* !CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS */

static inline void setup_pks(void) { }
static inline void show_extended_regs_oops(struct pt_regs *regs,
					   unsigned error_code) { }

#endif /* CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS */

#endif /* _ASM_X86_PKS_H */
