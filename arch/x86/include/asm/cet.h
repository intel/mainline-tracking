/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_CET_H
#define _ASM_X86_CET_H

#ifndef __ASSEMBLY__
#include <linux/types.h>

struct task_struct;
struct sc_ext;

/*
 * Per-thread CET status
 */
struct cet_status {
	unsigned long	shstk_base;
	unsigned long	shstk_size;
};

#ifdef CONFIG_X86_CET_USER
int cet_setup_shstk(void);
int cet_setup_thread_shstk(struct task_struct *p, unsigned long clone_flags);
void cet_disable_shstk(void);
void cet_free_shstk(struct task_struct *p);
int cet_verify_rstor_token(bool ia32, unsigned long ssp, unsigned long *new_ssp);
void cet_restore_signal(struct sc_ext *sc);
int cet_setup_signal(bool ia32, unsigned long rstor, struct sc_ext *sc);
#else
static inline int cet_setup_thread_shstk(struct task_struct *p,
					 unsigned long clone_flags) { return 0; }
static inline void cet_disable_shstk(void) {}
static inline void cet_free_shstk(struct task_struct *p) {}
static inline void cet_restore_signal(struct sc_ext *sc) { return; }
static inline int cet_setup_signal(bool ia32, unsigned long rstor,
				   struct sc_ext *sc) { return -EINVAL; }
#endif

#endif /* __ASSEMBLY__ */

#endif /* _ASM_X86_CET_H */
