/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_MMAN_H
#define _ASM_X86_MMAN_H

#include <linux/mm.h>
#include <uapi/asm/mman.h>

#ifdef CONFIG_X86_INTEL_MEMORY_PROTECTION_KEYS
/*
 * Take the 4 protection key bits out of the vma->vm_flags
 * value and turn them in to the bits that we can put in
 * to a pte.
 *
 * Only override these if Protection Keys are available
 * (which is only on 64-bit).
 */
#define arch_vm_get_page_prot(vm_flags)	__pgprot(	\
		((vm_flags) & VM_PKEY_BIT0 ? _PAGE_PKEY_BIT0 : 0) |	\
		((vm_flags) & VM_PKEY_BIT1 ? _PAGE_PKEY_BIT1 : 0) |	\
		((vm_flags) & VM_PKEY_BIT2 ? _PAGE_PKEY_BIT2 : 0) |	\
		((vm_flags) & VM_PKEY_BIT3 ? _PAGE_PKEY_BIT3 : 0))

#define pkey_vm_prot_bits(prot, key) (			\
		((key) & 0x1 ? VM_PKEY_BIT0 : 0) |      \
		((key) & 0x2 ? VM_PKEY_BIT1 : 0) |      \
		((key) & 0x4 ? VM_PKEY_BIT2 : 0) |      \
		((key) & 0x8 ? VM_PKEY_BIT3 : 0))
#else
#define pkey_vm_prot_bits(prot, key) (0)
#endif

static inline unsigned long arch_calc_vm_prot_bits(unsigned long prot,
	unsigned long pkey)
{
	unsigned long vm_prot_bits = pkey_vm_prot_bits(prot, pkey);

	if (!(prot & PROT_WRITE) && (prot & PROT_SHSTK))
		vm_prot_bits |= VM_SHSTK;

	return vm_prot_bits;
}
#define arch_calc_vm_prot_bits(prot, pkey) arch_calc_vm_prot_bits(prot, pkey)

#ifdef CONFIG_X86_CET_USER
static inline bool arch_validate_prot(unsigned long prot, unsigned long addr)
{
	unsigned long valid = PROT_READ | PROT_WRITE | PROT_EXEC | PROT_SEM;

	if (prot & ~(valid | PROT_SHSTK))
		return false;

	if (prot & PROT_SHSTK) {
		struct vm_area_struct *vma;

		if (!current->thread.cet.shstk_size)
			return false;

		/*
		 * A shadow stack mapping is indirectly writable by only
		 * the CALL and WRUSS instructions, but not other write
		 * instructions).  PROT_SHSTK and PROT_WRITE are mutually
		 * exclusive.
		 */
		if (prot & PROT_WRITE)
			return false;

		vma = find_vma(current->mm, addr);
		if (!vma)
			return false;

		/*
		 * Shadow stack cannot be backed by a file or shared.
		 */
		if (vma->vm_file || (vma->vm_flags & VM_SHARED))
			return false;
	}

	return true;
}
#define arch_validate_prot arch_validate_prot
#endif

#endif /* _ASM_X86_MMAN_H */
