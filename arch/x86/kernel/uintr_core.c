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
#include <linux/uaccess.h>

#include <asm/apic.h>
#include <asm/fpu/internal.h>
#include <asm/irq_vectors.h>
#include <asm/msr.h>
#include <asm/msr-index.h>
#include <asm/uintr.h>

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

struct uintr_upid_ctx {
	struct uintr_upid *upid;
	refcount_t refs;
};

struct uintr_receiver {
	struct uintr_upid_ctx *upid_ctx;
};

inline bool uintr_arch_enabled(void)
{
	return static_cpu_has(X86_FEATURE_UINTR);
}

static inline bool is_uintr_receiver(struct task_struct *t)
{
	return !!t->thread.ui_recv;
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

	return upid_ctx;
}

static void put_upid_ref(struct uintr_upid_ctx *upid_ctx)
{
	if (refcount_dec_and_test(&upid_ctx->refs))
		free_upid(upid_ctx);
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
