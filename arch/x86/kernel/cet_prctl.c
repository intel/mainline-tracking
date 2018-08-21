/* SPDX-License-Identifier: GPL-2.0 */

#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/prctl.h>
#include <linux/compat.h>
#include <linux/mman.h>
#include <linux/elfcore.h>
#include <asm/processor.h>
#include <asm/prctl.h>
#include <asm/cet.h>

/* See Documentation/x86/intel_cet.rst. */

static int handle_get_status(u64 arg2)
{
	struct cet_status *cet = &current->thread.cet;
	u64 buf[3] = {0, 0, 0};

	if (cet->shstk_size) {
		buf[0] |= GNU_PROPERTY_X86_FEATURE_1_SHSTK;
		buf[1] = (u64)cet->shstk_base;
		buf[2] = (u64)cet->shstk_size;
	}

	if (cet->ibt_enabled)
		buf[0] |= GNU_PROPERTY_X86_FEATURE_1_IBT;

	return copy_to_user((u64 __user *)arg2, buf, sizeof(buf));
}

static int handle_alloc_shstk(u64 arg2)
{
	int err = 0;
	unsigned long arg;
	unsigned long addr = 0;
	unsigned long size = 0;

	if (get_user(arg, (unsigned long __user *)arg2))
		return -EFAULT;

	size = arg;
	err = cet_alloc_shstk(&arg);
	if (err)
		return err;

	addr = arg;
	if (put_user((u64)addr, (u64 __user *)arg2)) {
		vm_munmap(addr, size);
		return -EFAULT;
	}

	return 0;
}

int prctl_cet(int option, u64 arg2)
{
	struct cet_status *cet;

	if (!IS_ENABLED(CONFIG_X86_INTEL_CET))
		return -EINVAL;

	if (option == ARCH_X86_CET_STATUS)
		return handle_get_status(arg2);

	if (!static_cpu_has(X86_FEATURE_SHSTK) &&
	    !static_cpu_has(X86_FEATURE_IBT))
		return -EINVAL;

	cet = &current->thread.cet;

	switch (option) {
	case ARCH_X86_CET_DISABLE:
		if (cet->locked)
			return -EPERM;
		if (arg2 & GNU_PROPERTY_X86_FEATURE_1_SHSTK)
			cet_disable_free_shstk(current);
		if (arg2 & GNU_PROPERTY_X86_FEATURE_1_IBT)
			cet_disable_ibt();

		return 0;

	case ARCH_X86_CET_LOCK:
		cet->locked = 1;
		return 0;

	case ARCH_X86_CET_ALLOC_SHSTK:
		return handle_alloc_shstk(arg2);

	default:
		return -EINVAL;
	}
}
