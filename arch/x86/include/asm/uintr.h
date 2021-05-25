/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_UINTR_H
#define _ASM_X86_UINTR_H

#ifdef CONFIG_X86_USER_INTERRUPTS

struct uintr_upid_ctx {
	struct task_struct *task;	/* Receiver task */
	struct uintr_upid *upid;
	refcount_t refs;
};

struct uintr_receiver_info {
	struct uintr_upid_ctx *upid_ctx;	/* UPID context */
	struct callback_head twork;		/* Task work head */
	u64 uvec;				/* Vector number */
};

bool uintr_arch_enabled(void);
int do_uintr_register_handler(u64 handler);
int do_uintr_unregister_handler(void);
int do_uintr_register_vector(struct uintr_receiver_info *r_info);
void do_uintr_unregister_vector(struct uintr_receiver_info *r_info);

void uintr_free(struct task_struct *task);

/* TODO: Inline the context switch related functions */
void switch_uintr_prepare(struct task_struct *prev);
void switch_uintr_return(void);

#else /* !CONFIG_X86_USER_INTERRUPTS */

static inline void uintr_free(struct task_struct *task) {}
static inline void switch_uintr_prepare(struct task_struct *prev) {}
static inline void switch_uintr_return(void) {}

#endif /* CONFIG_X86_USER_INTERRUPTS */

#endif /* _ASM_X86_UINTR_H */
