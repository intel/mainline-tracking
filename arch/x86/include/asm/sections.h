/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_SECTIONS_H
#define _ASM_X86_SECTIONS_H

#include <asm-generic/sections.h>
#include <asm/extable.h>

#ifdef CONFIG_X86_FRED
extern char __fred_entry_text_start[], __fred_entry_text_end[];

#define arch_in_event_entry_text arch_in_event_entry_text

/*
 * FRED delivers events to entry points in .noinstr.text, which
 * __irqentry_text_start..__irqentry_text_end does not cover.  See
 * __fred_entry_text_start in entry_64_fred.S.
 *
 * Note that the range includes the ring 3 entry point, which also
 * delivers syscalls, so it cannot tell interrupt entry and syscall
 * entry apart.  That is fine for the only consumer,
 * filter_irq_stacks(): it cuts at the innermost match, and for an
 * entry from ring 3 the entry frame is already the outermost frame
 * of the trace, so matching it is a no-op.
 */
static inline bool arch_in_event_entry_text(unsigned long addr)
{
	return addr >= (unsigned long)__fred_entry_text_start &&
	       addr < (unsigned long)__fred_entry_text_end;
}
#endif

extern char __relocate_kernel_start[], __relocate_kernel_end[];
extern char __brk_base[], __brk_limit[];
extern char __end_rodata_aligned[];

#if defined(CONFIG_X86_64)
extern char __end_rodata_hpage_align[];
#endif

extern char __end_of_kernel_reserve[];

extern unsigned long _brk_start, _brk_end;

#endif	/* _ASM_X86_SECTIONS_H */
