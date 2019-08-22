// SPDX-License-Identifier: GPL-2.0
/*
 * cet.c - Control-flow Enforcement (CET)
 *
 * Copyright (c) 2019, Intel Corporation.
 * Yu-cheng Yu <yu-cheng.yu@intel.com>
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sched/signal.h>
#include <linux/compat.h>
#include <asm/msr.h>
#include <asm/user.h>
#include <asm/fpu/internal.h>
#include <asm/fpu/xstate.h>
#include <asm/fpu/types.h>
#include <asm/cet.h>

static void start_update_msrs(void)
{
	fpregs_lock();
	if (test_thread_flag(TIF_NEED_FPU_LOAD))
		__fpregs_load_activate();
}

static void end_update_msrs(void)
{
	fpregs_unlock();
}

static unsigned long cet_get_shstk_addr(void)
{
	struct fpu *fpu = &current->thread.fpu;
	unsigned long ssp = 0;

	fpregs_lock();

	if (fpregs_state_valid(fpu, smp_processor_id())) {
		rdmsrl(MSR_IA32_PL3_SSP, ssp);
	} else {
		struct cet_user_state *p;

		p = get_xsave_addr(&fpu->state.xsave, XFEATURE_CET_USER);
		if (p)
			ssp = p->user_ssp;
	}

	fpregs_unlock();
	return ssp;
}

static unsigned long alloc_shstk(unsigned long size, int flags)
{
	struct mm_struct *mm = current->mm;
	unsigned long addr, populate;

	/* VM_SHSTK requires MAP_ANONYMOUS, MAP_PRIVATE */
	flags |= MAP_ANONYMOUS | MAP_PRIVATE;

	mmap_write_lock(mm);
	addr = do_mmap(NULL, 0, size, PROT_READ, flags, VM_SHSTK, 0,
		       &populate, NULL);
	mmap_write_unlock(mm);

	if (populate)
		mm_populate(addr, populate);

	return addr;
}

int cet_setup_shstk(void)
{
	unsigned long addr, size;
	struct cet_status *cet = &current->thread.cet;

	if (!static_cpu_has(X86_FEATURE_SHSTK))
		return -EOPNOTSUPP;

	size = round_up(min(rlimit(RLIMIT_STACK), 1UL << 32), PAGE_SIZE);
	addr = alloc_shstk(size, 0);

	if (IS_ERR_VALUE(addr))
		return PTR_ERR((void *)addr);

	cet->shstk_base = addr;
	cet->shstk_size = size;

	start_update_msrs();
	wrmsrl(MSR_IA32_PL3_SSP, addr + size);
	wrmsrl(MSR_IA32_U_CET, CET_SHSTK_EN);
	end_update_msrs();
	return 0;
}

void cet_disable_shstk(void)
{
	struct cet_status *cet = &current->thread.cet;
	u64 msr_val;

	if (!static_cpu_has(X86_FEATURE_SHSTK) ||
	    !cet->shstk_size || !cet->shstk_base)
		return;

	start_update_msrs();
	rdmsrl(MSR_IA32_U_CET, msr_val);
	wrmsrl(MSR_IA32_U_CET, msr_val & ~CET_SHSTK_EN);
	wrmsrl(MSR_IA32_PL3_SSP, 0);
	end_update_msrs();

	cet_free_shstk(current);
}

void cet_free_shstk(struct task_struct *tsk)
{
	struct cet_status *cet = &tsk->thread.cet;

	if (!static_cpu_has(X86_FEATURE_SHSTK) ||
	    !cet->shstk_size || !cet->shstk_base)
		return;

	if (!tsk->mm || (tsk->mm != current->mm))
		return;

	while (1) {
		int r;

		r = vm_munmap(cet->shstk_base, cet->shstk_size);

		/*
		 * Retry if mmap_lock is not available.
		 */
		if (r == -EINTR) {
			cond_resched();
			continue;
		}

		WARN_ON_ONCE(r);
		break;
	}

	cet->shstk_base = 0;
	cet->shstk_size = 0;
}
