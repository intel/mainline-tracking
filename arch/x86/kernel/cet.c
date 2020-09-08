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
#include <linux/vmalloc.h>
#include <linux/bitops.h>
#include <asm/msr.h>
#include <asm/user.h>
#include <asm/fpu/internal.h>
#include <asm/fpu/xstate.h>
#include <asm/fpu/types.h>
#include <asm/cet.h>
#include <asm/special_insns.h>
#include <uapi/asm/sigcontext.h>

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

#define TOKEN_MODE_MASK	3UL
#define TOKEN_MODE_64	1UL
#define IS_TOKEN_64(token) (((token) & TOKEN_MODE_MASK) == TOKEN_MODE_64)
#define IS_TOKEN_32(token) (((token) & TOKEN_MODE_MASK) == 0)

/*
 * Verify the restore token at the address of 'ssp' is
 * valid and then set shadow stack pointer according to the
 * token.
 */
int cet_verify_rstor_token(bool ia32, unsigned long ssp,
			   unsigned long *new_ssp)
{
	unsigned long token;

	*new_ssp = 0;

	if (!IS_ALIGNED(ssp, 8))
		return -EINVAL;

	if (get_user(token, (unsigned long __user *)ssp))
		return -EFAULT;

	/* Is 64-bit mode flag correct? */
	if (!ia32 && !IS_TOKEN_64(token))
		return -EINVAL;
	else if (ia32 && !IS_TOKEN_32(token))
		return -EINVAL;

	token &= ~TOKEN_MODE_MASK;

	/*
	 * Restore address properly aligned?
	 */
	if ((!ia32 && !IS_ALIGNED(token, 8)) || !IS_ALIGNED(token, 4))
		return -EINVAL;

	/*
	 * Token was placed properly?
	 */
	if (((ALIGN_DOWN(token, 8) - 8) != ssp) || (token >= TASK_SIZE_MAX))
		return -EINVAL;

	*new_ssp = token;
	return 0;
}

/*
 * Create a restore token on the shadow stack.
 * A token is always 8-byte and aligned to 8.
 */
static int create_rstor_token(bool ia32, unsigned long ssp,
			      unsigned long *new_ssp)
{
	unsigned long addr;

	*new_ssp = 0;

	if ((!ia32 && !IS_ALIGNED(ssp, 8)) || !IS_ALIGNED(ssp, 4))
		return -EINVAL;

	addr = ALIGN_DOWN(ssp, 8) - 8;

	/* Is the token for 64-bit? */
	if (!ia32)
		ssp |= TOKEN_MODE_64;

	if (write_user_shstk_64(addr, ssp))
		return -EFAULT;

	*new_ssp = addr;
	return 0;
}

unsigned long cet_alloc_shstk(unsigned long len, int flags)
{
	unsigned long token;
	unsigned long addr, ssp;

	addr = alloc_shstk(round_up(len, PAGE_SIZE), flags);

	if (IS_ERR_VALUE(addr))
		return addr;

	/* Restore token is 8 bytes and aligned to 8 bytes */
	ssp = addr + len;
	token = ssp;

	if (!in_ia32_syscall())
		token |= TOKEN_MODE_64;
	ssp -= 8;

	if (write_user_shstk_64(ssp, token)) {
		vm_munmap(addr, len);
		return -EINVAL;
	}

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

int cet_setup_thread_shstk(struct task_struct *tsk, unsigned long clone_flags)
{
	unsigned long addr, size;
	struct cet_user_state *state;
	struct cet_status *cet = &tsk->thread.cet;

	if (!cet->shstk_size)
		return 0;

	if ((clone_flags & (CLONE_VFORK | CLONE_VM)) != CLONE_VM)
		return 0;

	state = get_xsave_addr(&tsk->thread.fpu.state.xsave,
			       XFEATURE_CET_USER);

	if (!state)
		return -EINVAL;

	/* Cap shadow stack size to 4 GB */
	size = min(rlimit(RLIMIT_STACK), 1UL << 32);

	/*
	 * Compat-mode pthreads share a limited address space.
	 * If each function call takes an average of four slots
	 * stack space, allocate 1/4 of stack size for shadow stack.
	 */
	if (in_compat_syscall())
		size /= 4;
	size = round_up(size, PAGE_SIZE);
	addr = alloc_shstk(size, 0);

	if (IS_ERR_VALUE(addr)) {
		cet->shstk_base = 0;
		cet->shstk_size = 0;
		return PTR_ERR((void *)addr);
	}

	fpu__prepare_write(&tsk->thread.fpu);
	state->user_ssp = (u64)(addr + size);
	cet->shstk_base = addr;
	cet->shstk_size = size;
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

/*
 * Called from __fpu__restore_sig() and XSAVES buffer is protected by
 * set_thread_flag(TIF_NEED_FPU_LOAD) in the slow path.
 */
void cet_restore_signal(struct sc_ext *sc_ext)
{
	struct cet_user_state *cet_user_state;
	struct cet_status *cet = &current->thread.cet;
	u64 msr_val = 0;

	if (!static_cpu_has(X86_FEATURE_SHSTK))
		return;

	cet_user_state = get_xsave_addr(&current->thread.fpu.state.xsave,
					XFEATURE_CET_USER);
	if (!cet_user_state)
		return;

	if (cet->shstk_size) {
		if (test_thread_flag(TIF_NEED_FPU_LOAD))
			cet_user_state->user_ssp = sc_ext->ssp;
		else
			wrmsrl(MSR_IA32_PL3_SSP, sc_ext->ssp);

		msr_val |= CET_SHSTK_EN;
	}

	if (cet->ibt_enabled) {
		msr_val |= (CET_ENDBR_EN | CET_NO_TRACK_EN);

		if (sc_ext->wait_endbr)
			msr_val |= CET_WAIT_ENDBR;
	}

	if (test_thread_flag(TIF_NEED_FPU_LOAD))
		cet_user_state->user_cet = msr_val;
	else
		wrmsrl(MSR_IA32_U_CET, msr_val);
}

/*
 * Setup the shadow stack for the signal handler: first,
 * create a restore token to keep track of the current ssp,
 * and then the return address of the signal handler.
 */
int cet_setup_signal(bool ia32, unsigned long rstor_addr, struct sc_ext *sc_ext)
{
	struct cet_status *cet = &current->thread.cet;
	unsigned long ssp = 0, new_ssp = 0;
	int err;

	if (cet->shstk_size) {
		if (!rstor_addr)
			return -EINVAL;

		ssp = cet_get_shstk_addr();
		err = create_rstor_token(ia32, ssp, &new_ssp);
		if (err)
			return err;

		if (ia32) {
			ssp = new_ssp - sizeof(u32);
			err = write_user_shstk_32(ssp, (unsigned int)rstor_addr);
		} else {
			ssp = new_ssp - sizeof(u64);
			err = write_user_shstk_64(ssp, rstor_addr);
		}

		if (err)
			return err;

		sc_ext->ssp = new_ssp;
	}

	if (ssp || cet->ibt_enabled) {

		start_update_msrs();

		if (ssp)
			wrmsrl(MSR_IA32_PL3_SSP, ssp);

		if (cet->ibt_enabled) {
			u64 r;

			rdmsrl(MSR_IA32_U_CET, r);

			if (r & CET_WAIT_ENDBR) {
				sc_ext->wait_endbr = 1;
				r &= ~CET_WAIT_ENDBR;
				wrmsrl(MSR_IA32_U_CET, r);
			}
		}

		end_update_msrs();
	}

	return 0;
}

int cet_setup_ibt(void)
{
	u64 msr_val;

	if (!static_cpu_has(X86_FEATURE_IBT))
		return -EOPNOTSUPP;

	start_update_msrs();
	rdmsrl(MSR_IA32_U_CET, msr_val);
	msr_val |= (CET_ENDBR_EN | CET_NO_TRACK_EN);
	wrmsrl(MSR_IA32_U_CET, msr_val);
	end_update_msrs();
	current->thread.cet.ibt_enabled = 1;
	return 0;
}

void cet_disable_ibt(void)
{
	u64 msr_val;

	if (!static_cpu_has(X86_FEATURE_IBT))
		return;

	start_update_msrs();
	rdmsrl(MSR_IA32_U_CET, msr_val);
	msr_val &= ~CET_ENDBR_EN;
	wrmsrl(MSR_IA32_U_CET, msr_val);
	end_update_msrs();
	current->thread.cet.ibt_enabled = 0;
}
