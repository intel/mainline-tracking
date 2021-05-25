/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_UINTR_H
#define _ASM_X86_UINTR_H

#ifdef CONFIG_X86_USER_INTERRUPTS

bool uintr_arch_enabled(void);
int do_uintr_register_handler(u64 handler);
int do_uintr_unregister_handler(void);

#endif /* CONFIG_X86_USER_INTERRUPTS */

#endif /* _ASM_X86_UINTR_H */
