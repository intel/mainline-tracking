// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 * Jacob Pan <jacob.jun.pan@linux.intel.com>
 */
#define pr_fmt(fmt)    "uintr: " fmt

#include <linux/refcount.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/task_work.h>
#include <linux/uaccess.h>

#include <asm/apic.h>
#include <asm/fpu/internal.h>
#include <asm/irq_vectors.h>
#include <asm/msr.h>
#include <asm/msr-index.h>
#include <asm/uintr.h>

/*
 * Each UITT entry is 16 bytes in size.
 * Current UITT table size is set as 4KB (256 * 16 bytes)
 */
#define UINTR_MAX_UITT_NR 256
#define UINTR_MAX_UVEC_NR 64

/* User Posted Interrupt Descriptor (UPID) */
struct uintr_upid {
	struct {
		u8 status;	/* bit 0: ON, bit 1: SN, bit 2-7: reserved */
		u8 reserved1;	/* Reserved */
		u8 nv;		/* Notification vector */
		u8 reserved2;	/* Reserved */
		u32 ndst;	/* Notification destination */
	} nc __packed;		/* Notification control */
	u64 puir;		/* Posted user interrupt requests */
} __aligned(64);

/* UPID Notification control status */
#define UPID_ON		0x0	/* Outstanding notification */
#define UPID_SN		0x1	/* Suppressed notification */

struct uintr_receiver {
	struct uintr_upid_ctx *upid_ctx;
	u64 uvec_mask;	/* track active vector per bit */
};

/* User Interrupt Target Table Entry (UITTE) */
struct uintr_uitt_entry {
	u8	valid;			/* bit 0: valid, bit 1-7: reserved */
	u8	user_vec;
	u8	reserved[6];
	u64	target_upid_addr;
} __packed __aligned(16);

struct uintr_uitt_ctx {
	struct uintr_uitt_entry *uitt;
	/* Protect UITT */
	spinlock_t uitt_lock;
	refcount_t refs;
};

struct uintr_sender {
	struct uintr_uitt_ctx *uitt_ctx;
	/* track active uitt entries per bit */
	u64 uitt_mask[BITS_TO_U64(UINTR_MAX_UITT_NR)];
};

inline bool uintr_arch_enabled(void)
{
	return static_cpu_has(X86_FEATURE_UINTR);
}

static inline bool is_uintr_receiver(struct task_struct *t)
{
	return !!t->thread.ui_recv;
}

static inline bool is_uintr_sender(struct task_struct *t)
{
	return !!t->thread.ui_send;
}

static inline bool is_uintr_task(struct task_struct *t)
{
	return(is_uintr_receiver(t) || is_uintr_sender(t));
}

static inline bool is_uitt_empty(struct task_struct *t)
{
	return !!bitmap_empty((unsigned long *)t->thread.ui_send->uitt_mask,
			      UINTR_MAX_UITT_NR);
}

/*
 * No lock is needed to read the active flag. Writes only happen from
 * r_info->task that owns the UPID. Everyone else would just read this flag.
 *
 * This only provides a static check. The receiver may become inactive right
 * after this check. The primary reason to have this check is to prevent future
 * senders from connecting with this UPID, since the receiver task has already
 * made this UPID inactive.
 */
static bool uintr_is_receiver_active(struct uintr_receiver_info *r_info)
{
	return r_info->upid_ctx->receiver_active;
}

static inline u32 cpu_to_ndst(int cpu)
{
	u32 apicid = (u32)apic->cpu_present_to_apicid(cpu);

	WARN_ON_ONCE(apicid == BAD_APICID);

	if (!x2apic_enabled())
		return (apicid << 8) & 0xFF00;

	return apicid;
}

static void free_upid(struct uintr_upid_ctx *upid_ctx)
{
	put_task_struct(upid_ctx->task);
	kfree(upid_ctx->upid);
	upid_ctx->upid = NULL;
	kfree(upid_ctx);
}

/* TODO: UPID needs to be allocated by a KPTI compatible allocator */
static struct uintr_upid_ctx *alloc_upid(void)
{
	struct uintr_upid_ctx *upid_ctx;
	struct uintr_upid *upid;

	upid_ctx = kzalloc(sizeof(*upid_ctx), GFP_KERNEL);
	if (!upid_ctx)
		return NULL;

	upid = kzalloc(sizeof(*upid), GFP_KERNEL);

	if (!upid) {
		kfree(upid_ctx);
		return NULL;
	}

	upid_ctx->upid = upid;
	refcount_set(&upid_ctx->refs, 1);
	upid_ctx->task = get_task_struct(current);
	upid_ctx->receiver_active = true;

	return upid_ctx;
}

static void put_upid_ref(struct uintr_upid_ctx *upid_ctx)
{
	if (refcount_dec_and_test(&upid_ctx->refs))
		free_upid(upid_ctx);
}

static struct uintr_upid_ctx *get_upid_ref(struct uintr_upid_ctx *upid_ctx)
{
	refcount_inc(&upid_ctx->refs);
	return upid_ctx;
}

static void free_uitt(struct uintr_uitt_ctx *uitt_ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&uitt_ctx->uitt_lock, flags);
	kfree(uitt_ctx->uitt);
	uitt_ctx->uitt = NULL;
	spin_unlock_irqrestore(&uitt_ctx->uitt_lock, flags);

	kfree(uitt_ctx);
}

/* TODO: Replace UITT allocation with KPTI compatible memory allocator */
static struct uintr_uitt_ctx *alloc_uitt(void)
{
	struct uintr_uitt_ctx *uitt_ctx;
	struct uintr_uitt_entry *uitt;

	uitt_ctx = kzalloc(sizeof(*uitt_ctx), GFP_KERNEL);
	if (!uitt_ctx)
		return NULL;

	uitt = kzalloc(sizeof(*uitt) * UINTR_MAX_UITT_NR, GFP_KERNEL);
	if (!uitt) {
		kfree(uitt_ctx);
		return NULL;
	}

	uitt_ctx->uitt = uitt;
	spin_lock_init(&uitt_ctx->uitt_lock);
	refcount_set(&uitt_ctx->refs, 1);

	return uitt_ctx;
}

static void put_uitt_ref(struct uintr_uitt_ctx *uitt_ctx)
{
	if (refcount_dec_and_test(&uitt_ctx->refs))
		free_uitt(uitt_ctx);
}

static struct uintr_uitt_ctx *get_uitt_ref(struct uintr_uitt_ctx *uitt_ctx)
{
	refcount_inc(&uitt_ctx->refs);
	return uitt_ctx;
}

static inline void mark_uitte_invalid(struct uintr_sender_info *s_info)
{
	struct uintr_uitt_entry *uitte;
	unsigned long flags;

	spin_lock_irqsave(&s_info->uitt_ctx->uitt_lock, flags);
	uitte = &s_info->uitt_ctx->uitt[s_info->uitt_index];
	uitte->valid = 0;
	spin_unlock_irqrestore(&s_info->uitt_ctx->uitt_lock, flags);
}

static void __clear_vector_from_upid(u64 uvec, struct uintr_upid *upid)
{
	clear_bit(uvec, (unsigned long *)&upid->puir);
}

static void __clear_vector_from_task(u64 uvec)
{
	struct task_struct *t = current;

	pr_debug("recv: task=%d free vector %llu\n", t->pid, uvec);

	if (!(BIT_ULL(uvec) & t->thread.ui_recv->uvec_mask))
		return;

	clear_bit(uvec, (unsigned long *)&t->thread.ui_recv->uvec_mask);

	if (!t->thread.ui_recv->uvec_mask)
		pr_debug("recv: task=%d unregistered all user vectors\n", t->pid);
}

/* Callback to clear the vector structures when a vector is unregistered. */
static void receiver_clear_uvec(struct callback_head *head)
{
	struct uintr_receiver_info *r_info;
	struct uintr_upid_ctx *upid_ctx;
	struct task_struct *t = current;
	u64 uvec;

	r_info = container_of(head, struct uintr_receiver_info, twork);
	uvec = r_info->uvec;
	upid_ctx = r_info->upid_ctx;

	/*
	 * If a task has unregistered the interrupt handler the vector
	 * structures would have already been cleared.
	 */
	if (is_uintr_receiver(t)) {
		/*
		 * The UPID context in the callback might differ from the one
		 * on the task if the task unregistered its interrupt handler
		 * and then registered itself again. The vector structures
		 * related to the previous UPID would have already been cleared
		 * in that case.
		 */
		if (t->thread.ui_recv->upid_ctx != upid_ctx) {
			pr_debug("recv: task %d is now using a different UPID\n",
				 t->pid);
			goto out_free;
		}

		/*
		 * If the vector has been recognized in the UIRR don't modify
		 * it. We need to disable User Interrupts before modifying the
		 * UIRR. It might be better to just let that interrupt get
		 * delivered.
		 */
		__clear_vector_from_upid(uvec, upid_ctx->upid);
		__clear_vector_from_task(uvec);
	}

out_free:
	put_upid_ref(upid_ctx);
	kfree(r_info);
}

static void teardown_uitt(void)
{
	struct task_struct *t = current;
	struct fpu *fpu = &t->thread.fpu;
	u64 msr64;

	put_uitt_ref(t->thread.ui_send->uitt_ctx);
	kfree(t->thread.ui_send);
	t->thread.ui_send = NULL;

	fpregs_lock();

	if (fpregs_state_valid(fpu, smp_processor_id())) {
		/* Modify only the relevant bits of the MISC MSR */
		rdmsrl(MSR_IA32_UINTR_MISC, msr64);
		msr64 &= GENMASK_ULL(63, 32);
		wrmsrl(MSR_IA32_UINTR_MISC, msr64);
		wrmsrl(MSR_IA32_UINTR_TT, 0ULL);
	} else {
		struct uintr_state *p;

		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_UINTR);
		if (p) {
			p->uitt_size = 0;
			p->uitt_addr = 0;
		}
	}

	fpregs_unlock();
}

static int init_uitt(void)
{
	struct task_struct *t = current;
	struct fpu *fpu = &t->thread.fpu;
	struct uintr_sender *ui_send;
	u64 msr64;

	ui_send = kzalloc(sizeof(*t->thread.ui_send), GFP_KERNEL);
	if (!ui_send)
		return -ENOMEM;

	ui_send->uitt_ctx = alloc_uitt();
	if (!ui_send->uitt_ctx) {
		pr_debug("send: Alloc UITT failed for task=%d\n", t->pid);
		kfree(ui_send);
		return -ENOMEM;
	}

	fpregs_lock();

	if (fpregs_state_valid(fpu, smp_processor_id())) {
		wrmsrl(MSR_IA32_UINTR_TT, (u64)ui_send->uitt_ctx->uitt | 1);
		/* Modify only the relevant bits of the MISC MSR */
		rdmsrl(MSR_IA32_UINTR_MISC, msr64);
		msr64 &= GENMASK_ULL(63, 32);
		msr64 |= UINTR_MAX_UITT_NR;
		wrmsrl(MSR_IA32_UINTR_MISC, msr64);
	} else {
		struct xregs_state *xsave;
		struct uintr_state *p;

		xsave = &fpu->state.xsave;
		xsave->header.xfeatures |= XFEATURE_MASK_UINTR;
		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_UINTR);
		if (p) {
			p->uitt_size = UINTR_MAX_UITT_NR;
			p->uitt_addr = (u64)ui_send->uitt_ctx->uitt | 1;
		}
	}

	fpregs_unlock();

	pr_debug("send: Setup a new UITT=%px for task=%d with size %d\n",
		 ui_send->uitt_ctx->uitt, t->pid, UINTR_MAX_UITT_NR * 16);

	t->thread.ui_send = ui_send;

	return 0;
}

static void __free_uitt_entry(unsigned int entry)
{
	struct task_struct *t = current;
	unsigned long flags;

	if (entry >= UINTR_MAX_UITT_NR)
		return;

	if (!is_uintr_sender(t))
		return;

	pr_debug("send: Freeing UITTE entry %d for task=%d\n", entry, t->pid);

	spin_lock_irqsave(&t->thread.ui_send->uitt_ctx->uitt_lock, flags);
	memset(&t->thread.ui_send->uitt_ctx->uitt[entry], 0,
	       sizeof(struct uintr_uitt_entry));
	spin_unlock_irqrestore(&t->thread.ui_send->uitt_ctx->uitt_lock, flags);

	clear_bit(entry, (unsigned long *)t->thread.ui_send->uitt_mask);

	if (is_uitt_empty(t)) {
		pr_debug("send: UITT mask is empty. Dereference and teardown UITT\n");
		teardown_uitt();
	}
}

static void sender_free_uitte(struct callback_head *head)
{
	struct uintr_sender_info *s_info;

	s_info = container_of(head, struct uintr_sender_info, twork);

	__free_uitt_entry(s_info->uitt_index);
	put_uitt_ref(s_info->uitt_ctx);
	put_upid_ref(s_info->r_upid_ctx);
	put_task_struct(s_info->task);
	kfree(s_info);
}

void do_uintr_unregister_sender(struct uintr_receiver_info *r_info,
				struct uintr_sender_info *s_info)
{
	int ret;

	/*
	 * To make sure any new senduipi result in a #GP fault.
	 * The task work might take non-zero time to kick the process out.
	 */
	mark_uitte_invalid(s_info);

	pr_debug("send: Adding Free UITTE %d task work for task=%d\n",
		 s_info->uitt_index, s_info->task->pid);

	init_task_work(&s_info->twork, sender_free_uitte);
	ret = task_work_add(s_info->task, &s_info->twork, true);
	if (ret) {
		/*
		 * Dereferencing the UITT and UPID here since the task has
		 * exited.
		 */
		pr_debug("send: Free UITTE %d task=%d has already exited\n",
			 s_info->uitt_index, s_info->task->pid);
		put_upid_ref(s_info->r_upid_ctx);
		put_uitt_ref(s_info->uitt_ctx);
		put_task_struct(s_info->task);
		kfree(s_info);
		return;
	}
}

int do_uintr_register_sender(struct uintr_receiver_info *r_info,
			     struct uintr_sender_info *s_info)
{
	struct uintr_uitt_entry *uitte = NULL;
	struct uintr_sender *ui_send;
	struct task_struct *t = current;
	unsigned long flags;
	int entry;
	int ret;

	/*
	 * Only a static check. Receiver could exit anytime after this check.
	 * This check only prevents connections using uintr_fd after the
	 * receiver has already exited/unregistered.
	 */
	if (!uintr_is_receiver_active(r_info))
		return -ESHUTDOWN;

	if (is_uintr_sender(t)) {
		entry = find_first_zero_bit((unsigned long *)t->thread.ui_send->uitt_mask,
					    UINTR_MAX_UITT_NR);
		if (entry >= UINTR_MAX_UITT_NR)
			return -ENOSPC;
	} else {
		BUILD_BUG_ON(UINTR_MAX_UITT_NR < 1);
		entry = 0;
		ret = init_uitt();
		if (ret)
			return ret;
	}

	ui_send = t->thread.ui_send;

	set_bit(entry, (unsigned long *)ui_send->uitt_mask);

	spin_lock_irqsave(&ui_send->uitt_ctx->uitt_lock, flags);
	uitte = &ui_send->uitt_ctx->uitt[entry];
	pr_debug("send: sender=%d receiver=%d UITTE entry %d address %px\n",
		 current->pid, r_info->upid_ctx->task->pid, entry, uitte);

	uitte->user_vec = r_info->uvec;
	uitte->target_upid_addr = (u64)r_info->upid_ctx->upid;
	uitte->valid = 1;
	spin_unlock_irqrestore(&ui_send->uitt_ctx->uitt_lock, flags);

	s_info->r_upid_ctx = get_upid_ref(r_info->upid_ctx);
	s_info->uitt_ctx = get_uitt_ref(ui_send->uitt_ctx);
	s_info->task = get_task_struct(current);
	s_info->uitt_index = entry;

	return 0;
}

int do_uintr_unregister_handler(void)
{
	struct task_struct *t = current;
	struct fpu *fpu = &t->thread.fpu;
	struct uintr_receiver *ui_recv;
	u64 msr64;

	if (!is_uintr_receiver(t))
		return -EINVAL;

	pr_debug("recv: Unregister handler and clear MSRs for task=%d\n",
		 t->pid);

	/*
	 * TODO: Evaluate usage of fpregs_lock() and get_xsave_addr(). Bugs
	 * have been reported recently for PASID and WRPKRU.
	 *
	 * UPID and ui_recv will be referenced during context switch. Need to
	 * disable preemption while modifying the MSRs, UPID and ui_recv thread
	 * struct.
	 */
	fpregs_lock();

	/* Clear only the receiver specific state. Sender related state is not modified */
	if (fpregs_state_valid(fpu, smp_processor_id())) {
		/* Modify only the relevant bits of the MISC MSR */
		rdmsrl(MSR_IA32_UINTR_MISC, msr64);
		msr64 &= ~GENMASK_ULL(39, 32);
		wrmsrl(MSR_IA32_UINTR_MISC, msr64);
		wrmsrl(MSR_IA32_UINTR_PD, 0ULL);
		wrmsrl(MSR_IA32_UINTR_RR, 0ULL);
		wrmsrl(MSR_IA32_UINTR_STACKADJUST, 0ULL);
		wrmsrl(MSR_IA32_UINTR_HANDLER, 0ULL);
	} else {
		struct uintr_state *p;

		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_UINTR);
		if (p) {
			p->handler = 0;
			p->stack_adjust = 0;
			p->upid_addr = 0;
			p->uinv = 0;
			p->uirr = 0;
		}
	}

	ui_recv = t->thread.ui_recv;
	ui_recv->upid_ctx->receiver_active = false;

	/*
	 * Suppress notifications so that no further interrupts are generated
	 * based on this UPID.
	 */
	set_bit(UPID_SN, (unsigned long *)&ui_recv->upid_ctx->upid->nc.status);

	put_upid_ref(ui_recv->upid_ctx);
	kfree(ui_recv);
	t->thread.ui_recv = NULL;

	fpregs_unlock();

	return 0;
}

int do_uintr_register_handler(u64 handler)
{
	struct uintr_receiver *ui_recv;
	struct uintr_upid *upid;
	struct task_struct *t = current;
	struct fpu *fpu = &t->thread.fpu;
	u64 misc_msr;
	int cpu;

	if (is_uintr_receiver(t))
		return -EBUSY;

	ui_recv = kzalloc(sizeof(*ui_recv), GFP_KERNEL);
	if (!ui_recv)
		return -ENOMEM;

	ui_recv->upid_ctx = alloc_upid();
	if (!ui_recv->upid_ctx) {
		kfree(ui_recv);
		pr_debug("recv: alloc upid failed for task=%d\n", t->pid);
		return -ENOMEM;
	}

	/*
	 * TODO: Evaluate usage of fpregs_lock() and get_xsave_addr(). Bugs
	 * have been reported recently for PASID and WRPKRU.
	 *
	 * UPID and ui_recv will be referenced during context switch. Need to
	 * disable preemption while modifying the MSRs, UPID and ui_recv thread
	 * struct.
	 */
	fpregs_lock();

	cpu = smp_processor_id();
	upid = ui_recv->upid_ctx->upid;
	upid->nc.nv = UINTR_NOTIFICATION_VECTOR;
	upid->nc.ndst = cpu_to_ndst(cpu);

	t->thread.ui_recv = ui_recv;

	if (fpregs_state_valid(fpu, cpu)) {
		wrmsrl(MSR_IA32_UINTR_HANDLER, handler);
		wrmsrl(MSR_IA32_UINTR_PD, (u64)ui_recv->upid_ctx->upid);

		/* Set value as size of ABI redzone */
		wrmsrl(MSR_IA32_UINTR_STACKADJUST, 128);

		/* Modify only the relevant bits of the MISC MSR */
		rdmsrl(MSR_IA32_UINTR_MISC, misc_msr);
		misc_msr |= (u64)UINTR_NOTIFICATION_VECTOR << 32;
		wrmsrl(MSR_IA32_UINTR_MISC, misc_msr);
	} else {
		struct xregs_state *xsave;
		struct uintr_state *p;

		xsave = &fpu->state.xsave;
		xsave->header.xfeatures |= XFEATURE_MASK_UINTR;
		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_UINTR);
		if (p) {
			p->handler = handler;
			p->upid_addr = (u64)ui_recv->upid_ctx->upid;
			p->stack_adjust = 128;
			p->uinv = UINTR_NOTIFICATION_VECTOR;
		}
	}

	fpregs_unlock();

	pr_debug("recv: task=%d register handler=%llx upid %px\n",
		 t->pid, handler, upid);

	return 0;
}

void do_uintr_unregister_vector(struct uintr_receiver_info *r_info)
{
	int ret;

	pr_debug("recv: Adding task work to clear vector %llu added for task=%d\n",
		 r_info->uvec, r_info->upid_ctx->task->pid);

	init_task_work(&r_info->twork, receiver_clear_uvec);
	ret = task_work_add(r_info->upid_ctx->task, &r_info->twork, true);
	if (ret) {
		pr_debug("recv: Clear vector task=%d has already exited\n",
			 r_info->upid_ctx->task->pid);
		put_upid_ref(r_info->upid_ctx);
		kfree(r_info);
		return;
	}
}

int do_uintr_register_vector(struct uintr_receiver_info *r_info)
{
	struct uintr_receiver *ui_recv;
	struct task_struct *t = current;

	/*
	 * A vector should only be registered by a task that
	 * has an interrupt handler registered.
	 */
	if (!is_uintr_receiver(t))
		return -EINVAL;

	if (r_info->uvec >= UINTR_MAX_UVEC_NR)
		return -ENOSPC;

	ui_recv = t->thread.ui_recv;

	if (ui_recv->uvec_mask & BIT_ULL(r_info->uvec))
		return -EBUSY;

	ui_recv->uvec_mask |= BIT_ULL(r_info->uvec);
	pr_debug("recv: task %d new uvec=%llu, new mask %llx\n",
		 t->pid, r_info->uvec, ui_recv->uvec_mask);

	r_info->upid_ctx = get_upid_ref(ui_recv->upid_ctx);

	return 0;
}

/* Suppress notifications since this task is being context switched out */
void switch_uintr_prepare(struct task_struct *prev)
{
	struct uintr_upid *upid;

	if (is_uintr_receiver(prev)) {
		upid = prev->thread.ui_recv->upid_ctx->upid;
		set_bit(UPID_SN, (unsigned long *)&upid->nc.status);
	}
}

/*
 * Do this right before we are going back to userspace after the FPU has been
 * reloaded i.e. TIF_NEED_FPU_LOAD is clear.
 * Called from arch_exit_to_user_mode_prepare() with interrupts disabled.
 */
void switch_uintr_return(void)
{
	struct uintr_upid *upid;
	u64 misc_msr;

	if (is_uintr_receiver(current)) {
		/*
		 * The XSAVES instruction clears the UINTR notification
		 * vector(UINV) in the UINT_MISC MSR when user context gets
		 * saved. Before going back to userspace we need to restore the
		 * notification vector. XRSTORS would automatically restore the
		 * notification but we can't be sure that XRSTORS will always
		 * be called when going back to userspace. Also if XSAVES gets
		 * called twice the UINV stored in the Xstate buffer will be
		 * overwritten. Threfore, before going back to userspace we
		 * always check if the UINV is set and reprogram if needed.
		 *
		 * Alternatively, we could combine this with
		 * switch_fpu_return() and program the MSR whenever we are
		 * skipping the XRSTORS. We need special precaution to make
		 * sure the UINV value in the XSTATE buffer doesn't get
		 * overwritten by calling XSAVES twice.
		 */
		WARN_ON_ONCE(test_thread_flag(TIF_NEED_FPU_LOAD));

		/* Modify only the relevant bits of the MISC MSR */
		rdmsrl(MSR_IA32_UINTR_MISC, misc_msr);
		if (!(misc_msr & GENMASK_ULL(39, 32))) {
			misc_msr |= (u64)UINTR_NOTIFICATION_VECTOR << 32;
			wrmsrl(MSR_IA32_UINTR_MISC, misc_msr);
		}

		/*
		 * It is necessary to clear the SN bit after we set UINV and
		 * NDST to avoid incorrect interrupt routing.
		 */
		upid = current->thread.ui_recv->upid_ctx->upid;
		upid->nc.ndst = cpu_to_ndst(smp_processor_id());
		clear_bit(UPID_SN, (unsigned long *)&upid->nc.status);

		/*
		 * Interrupts might have accumulated in the UPID while the
		 * thread was preempted. In this case invoke the hardware
		 * detection sequence manually by sending a self IPI with UINV.
		 * Since UINV is set and SN is cleared, any new UINTR
		 * notifications due to the self IPI or otherwise would result
		 * in the hardware updating the UIRR directly.
		 * No real interrupt would be generated as a result of this.
		 *
		 * The alternative is to atomically read and clear the UPID and
		 * program the UIRR. In that case the kernel would need to
		 * carefully manage the race with the hardware if the UPID gets
		 * updated after the read.
		 */
		if (READ_ONCE(upid->puir))
			apic->send_IPI_self(UINTR_NOTIFICATION_VECTOR);
	}
}

/*
 * This should only be called from exit_thread().
 * exit_thread() can happen in current context when the current thread is
 * exiting or it can happen for a new thread that is being created.
 * For new threads is_uintr_task() should fail.
 */
void uintr_free(struct task_struct *t)
{
	struct uintr_receiver *ui_recv;
	struct fpu *fpu;

	if (!static_cpu_has(X86_FEATURE_UINTR) || !is_uintr_task(t))
		return;

	if (WARN_ON_ONCE(t != current))
		return;

	fpu = &t->thread.fpu;

	fpregs_lock();

	if (fpregs_state_valid(fpu, smp_processor_id())) {
		wrmsrl(MSR_IA32_UINTR_MISC, 0ULL);
		wrmsrl(MSR_IA32_UINTR_TT, 0ULL);
		wrmsrl(MSR_IA32_UINTR_PD, 0ULL);
		wrmsrl(MSR_IA32_UINTR_RR, 0ULL);
		wrmsrl(MSR_IA32_UINTR_STACKADJUST, 0ULL);
		wrmsrl(MSR_IA32_UINTR_HANDLER, 0ULL);
	} else {
		struct uintr_state *p;

		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_UINTR);
		if (p) {
			p->handler = 0;
			p->uirr = 0;
			p->upid_addr = 0;
			p->stack_adjust = 0;
			p->uinv = 0;
			p->uitt_addr = 0;
			p->uitt_size = 0;
		}
	}

	/* Check: Can a thread be context switched while it is exiting? */
	if (is_uintr_receiver(t)) {
		ui_recv = t->thread.ui_recv;

		/*
		 * Suppress notifications so that no further interrupts are
		 * generated based on this UPID.
		 */
		set_bit(UPID_SN, (unsigned long *)&ui_recv->upid_ctx->upid->nc.status);
		ui_recv->upid_ctx->receiver_active = false;
		put_upid_ref(ui_recv->upid_ctx);
		kfree(ui_recv);
		t->thread.ui_recv = NULL;
	}

	fpregs_unlock();

	if (is_uintr_sender(t)) {
		put_uitt_ref(t->thread.ui_send->uitt_ctx);
		kfree(t->thread.ui_send);
		t->thread.ui_send = NULL;
	}
}
