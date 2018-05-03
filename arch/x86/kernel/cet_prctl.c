// SPDX-License-Identifier: GPL-2.0

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

static int copy_status_to_user(struct cet_status *cet, u64 arg2)
{
	u64 buf[3] = {0, 0, 0};

	if (cet->shstk_size) {
		buf[0] |= GNU_PROPERTY_X86_FEATURE_1_SHSTK;
		buf[1] = (u64)cet->shstk_base;
		buf[2] = (u64)cet->shstk_size;
	}

	return copy_to_user((u64 __user *)arg2, buf, sizeof(buf));
}

int prctl_cet(int option, u64 arg2)
{
	struct cet_status *cet;
	unsigned int features;

	/*
	 * GLIBC's ENOTSUPP == EOPNOTSUPP == 95, and it does not recognize
	 * the kernel's ENOTSUPP (524).  So return EOPNOTSUPP here.
	 */
	if (!IS_ENABLED(CONFIG_X86_CET_USER))
		return -EOPNOTSUPP;

	cet = &current->thread.cet;

	if (option == ARCH_X86_CET_STATUS)
		return copy_status_to_user(cet, arg2);

	if (!static_cpu_has(X86_FEATURE_CET))
		return -EOPNOTSUPP;

	switch (option) {
	case ARCH_X86_CET_DISABLE:
		if (cet->locked)
			return -EPERM;

		features = (unsigned int)arg2;

		if (features & GNU_PROPERTY_X86_FEATURE_1_INVAL)
			return -EINVAL;
		if (features & GNU_PROPERTY_X86_FEATURE_1_SHSTK)
			cet_disable_shstk();
		return 0;

	case ARCH_X86_CET_LOCK:
		cet->locked = 1;
		return 0;

	default:
		return -ENOSYS;
	}
}
