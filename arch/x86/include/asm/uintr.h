/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_UINTR_H
#define _ASM_X86_UINTR_H

#ifdef CONFIG_X86_USER_INTERRUPTS

bool uintr_arch_enabled(void);
int do_uintr_register_handler(u64 handler);
int do_uintr_unregister_handler(void);

/* TODO: Inline the context switch related functions */
void switch_uintr_prepare(struct task_struct *prev);
void switch_uintr_return(void);

#else /* !CONFIG_X86_USER_INTERRUPTS */

static inline void switch_uintr_prepare(struct task_struct *prev) {}
static inline void switch_uintr_return(void) {}

#endif /* CONFIG_X86_USER_INTERRUPTS */

#endif /* _ASM_X86_UINTR_H */
