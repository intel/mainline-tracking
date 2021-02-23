/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PKS_H
#define _ASM_X86_PKS_H

#ifdef CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS

/*  PKS supports 16 keys. Key 0 is reserved for the kernel. */
#define        PKS_KERN_DEFAULT_KEY    0
#define        PKS_NUM_KEYS            16

struct extended_pt_regs {
	u32 thread_pkrs;
	u32 pkrs_ref;
	struct pt_regs pt_regs;
};

void setup_pks(void);

static inline struct extended_pt_regs *extended_pt_regs(struct pt_regs *regs)
{
	return container_of(regs, struct extended_pt_regs, pt_regs);
}

void show_extended_regs_oops(struct pt_regs *regs, unsigned error_code);
bool handle_pks(struct pt_regs *regs, unsigned long error_code,
		unsigned long address);

#else /* !CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS */

static inline void setup_pks(void) { }
static inline void show_extended_regs_oops(struct pt_regs *regs,
					   unsigned error_code) { }

static inline bool handle_pks(struct pt_regs *regs, unsigned long error_code,
			      unsigned long address)
{
	return false;
}

#endif /* CONFIG_ARCH_ENABLE_SUPERVISOR_PKEYS */


#ifdef CONFIG_PKS_TEST

#define __static_or_pks_test

bool handle_pks_test(unsigned long hw_error_code, struct pt_regs *regs);
bool pks_test_callback(struct pt_regs *regs);

#else /* !CONFIG_PKS_TEST */

#define __static_or_pks_test static

static inline bool handle_pks_test(unsigned long hw_error_code, struct pt_regs *regs)
{
	return false;
}

#endif /* CONFIG_PKS_TEST */

#endif /* _ASM_X86_PKS_H */
