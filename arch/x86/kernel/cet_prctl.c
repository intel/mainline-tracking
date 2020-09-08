// SPDX-License-Identifier: GPL-2.0

#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/prctl.h>
#include <linux/compat.h>
#include <linux/mman.h>
#include <linux/elfcore.h>
#include <linux/processor.h>
#include <asm/prctl.h>
#include <asm/cet.h>

/* See Documentation/x86/intel_cet.rst. */

static int cet_copy_status_to_user(struct cet_status *cet, u64 __user *ubuf)
{
	u64 buf[3] = {};

	if (cet->shstk_size) {
		buf[0] |= GNU_PROPERTY_X86_FEATURE_1_SHSTK;
		buf[1] = cet->shstk_base;
		buf[2] = cet->shstk_size;
	}

	if (cet->ibt_enabled)
		buf[0] |= GNU_PROPERTY_X86_FEATURE_1_IBT;

	return copy_to_user(ubuf, buf, sizeof(buf));
}

static int handle_alloc_shstk(u64 arg2)
{
	unsigned long addr, size;

	if (get_user(size, (unsigned long __user *)arg2))
		return -EFAULT;

	addr = cet_alloc_shstk(size, 0);
	if (IS_ERR_VALUE(addr))
		return PTR_ERR((void *)addr);

	if (put_user((u64)addr, (u64 __user *)arg2)) {
		vm_munmap(addr, size);
		return -EFAULT;
	}

	return 0;
}

int prctl_cet(int option, u64 arg2)
{
	struct cet_status *cet;

	if (!cpu_feature_enabled(X86_FEATURE_CET))
		return -ENOTSUPP;

	cet = &current->thread.cet;

	if (option == ARCH_X86_CET_STATUS)
		return cet_copy_status_to_user(cet, (u64 __user *)arg2);

	switch (option) {
	case ARCH_X86_CET_DISABLE:
		if (cet->locked)
			return -EPERM;

		if (arg2 & ~GNU_PROPERTY_X86_FEATURE_1_VALID)
			return -EINVAL;
		if (arg2 & GNU_PROPERTY_X86_FEATURE_1_SHSTK)
			cet_disable_shstk();
		if (arg2 & GNU_PROPERTY_X86_FEATURE_1_IBT)
			cet_disable_ibt();
		return 0;

	case ARCH_X86_CET_LOCK:
		if (arg2)
			return -EINVAL;
		cet->locked = 1;
		return 0;

	case ARCH_X86_CET_ALLOC_SHSTK:
		return handle_alloc_shstk(arg2);

	default:
		return -ENOSYS;
	}
}
