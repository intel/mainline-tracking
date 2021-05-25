// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021, Intel Corporation.
 *
 * Sohil Mehta <sohil.mehta@intel.com>
 */
#define pr_fmt(fmt)	"uintr: " fmt

#include <linux/anon_inodes.h>
#include <linux/fdtable.h>
#include <linux/sched.h>
#include <linux/syscalls.h>

#include <asm/uintr.h>

struct uintrfd_ctx {
	struct uintr_receiver_info *r_info;
	/* Protect sender_list */
	spinlock_t sender_lock;
	struct list_head sender_list;
};

#ifdef CONFIG_PROC_FS
static void uintrfd_show_fdinfo(struct seq_file *m, struct file *file)
{
	struct uintrfd_ctx *uintrfd_ctx = file->private_data;

	/* Check: Should we print the receiver and sender info here? */
	seq_printf(m, "user_vector:%llu\n", uintrfd_ctx->r_info->uvec);
}
#endif

static int uintrfd_release(struct inode *inode, struct file *file)
{
	struct uintrfd_ctx *uintrfd_ctx = file->private_data;
	struct uintr_sender_info *s_info, *tmp;
	unsigned long flags;

	pr_debug("recv: Release uintrfd for r_task %d uvec %llu\n",
		 uintrfd_ctx->r_info->upid_ctx->task->pid,
		 uintrfd_ctx->r_info->uvec);

	spin_lock_irqsave(&uintrfd_ctx->sender_lock, flags);
	list_for_each_entry_safe(s_info, tmp, &uintrfd_ctx->sender_list, node) {
		list_del(&s_info->node);
		do_uintr_unregister_sender(uintrfd_ctx->r_info, s_info);
	}
	spin_unlock_irqrestore(&uintrfd_ctx->sender_lock, flags);

	do_uintr_unregister_vector(uintrfd_ctx->r_info);
	kfree(uintrfd_ctx);

	return 0;
}

static const struct file_operations uintrfd_fops = {
#ifdef CONFIG_PROC_FS
	.show_fdinfo	= uintrfd_show_fdinfo,
#endif
	.release	= uintrfd_release,
	.llseek		= noop_llseek,
};

/*
 * sys_uintr_create_fd - Create a uintr_fd for the registered interrupt vector.
 */
SYSCALL_DEFINE2(uintr_create_fd, u64, vector, unsigned int, flags)
{
	struct uintrfd_ctx *uintrfd_ctx;
	int uintrfd;
	int ret;

	if (!uintr_arch_enabled())
		return -EOPNOTSUPP;

	if (flags)
		return -EINVAL;

	uintrfd_ctx = kzalloc(sizeof(*uintrfd_ctx), GFP_KERNEL);
	if (!uintrfd_ctx)
		return -ENOMEM;

	uintrfd_ctx->r_info = kzalloc(sizeof(*uintrfd_ctx->r_info), GFP_KERNEL);
	if (!uintrfd_ctx->r_info) {
		ret = -ENOMEM;
		goto out_free_ctx;
	}

	uintrfd_ctx->r_info->uvec = vector;
	ret = do_uintr_register_vector(uintrfd_ctx->r_info);
	if (ret) {
		kfree(uintrfd_ctx->r_info);
		goto out_free_ctx;
	}

	INIT_LIST_HEAD(&uintrfd_ctx->sender_list);
	spin_lock_init(&uintrfd_ctx->sender_lock);

	/* TODO: Get user input for flags - UFD_CLOEXEC */
	/* Check: Do we need O_NONBLOCK? */
	uintrfd = anon_inode_getfd("[uintrfd]", &uintrfd_fops, uintrfd_ctx,
				   O_RDONLY | O_CLOEXEC | O_NONBLOCK);

	if (uintrfd < 0) {
		ret = uintrfd;
		goto out_free_uvec;
	}

	pr_debug("recv: Alloc vector success uintrfd %d uvec %llu for task=%d\n",
		 uintrfd, uintrfd_ctx->r_info->uvec, current->pid);

	return uintrfd;

out_free_uvec:
	do_uintr_unregister_vector(uintrfd_ctx->r_info);
out_free_ctx:
	kfree(uintrfd_ctx);
	pr_debug("recv: Alloc vector failed for task=%d ret %d\n",
		 current->pid, ret);
	return ret;
}

/*
 * sys_uintr_register_handler - setup user interrupt handler for receiver.
 */
SYSCALL_DEFINE2(uintr_register_handler, u64 __user *, handler, unsigned int, flags)
{
	int ret;

	if (!uintr_arch_enabled())
		return -EOPNOTSUPP;

	if (flags)
		return -EINVAL;

	/* TODO: Validate the handler address */
	if (!handler)
		return -EFAULT;

	ret = do_uintr_register_handler((u64)handler);

	pr_debug("recv: register handler task=%d flags %d handler %lx ret %d\n",
		 current->pid, flags, (unsigned long)handler, ret);

	return ret;
}

/*
 * sys_uintr_unregister_handler - Teardown user interrupt handler for receiver.
 */
SYSCALL_DEFINE1(uintr_unregister_handler, unsigned int, flags)
{
	int ret;

	if (!uintr_arch_enabled())
		return -EOPNOTSUPP;

	if (flags)
		return -EINVAL;

	ret = do_uintr_unregister_handler();

	pr_debug("recv: unregister handler task=%d flags %d ret %d\n",
		 current->pid, flags, ret);

	return ret;
}

/*
 * sys_uintr_register_sender - setup user inter-processor interrupt sender.
 */
SYSCALL_DEFINE2(uintr_register_sender, int, uintrfd, unsigned int, flags)
{
	struct uintr_sender_info *s_info;
	struct uintrfd_ctx *uintrfd_ctx;
	unsigned long lock_flags;
	struct file *uintr_f;
	struct fd f;
	int ret = 0;

	if (!uintr_arch_enabled())
		return -EOPNOTSUPP;

	if (flags)
		return -EINVAL;

	f = fdget(uintrfd);
	uintr_f = f.file;
	if (!uintr_f)
		return -EBADF;

	if (uintr_f->f_op != &uintrfd_fops) {
		ret = -EOPNOTSUPP;
		goto out_fdput;
	}

	uintrfd_ctx = (struct uintrfd_ctx *)uintr_f->private_data;

	spin_lock_irqsave(&uintrfd_ctx->sender_lock, lock_flags);
	list_for_each_entry(s_info, &uintrfd_ctx->sender_list, node) {
		if (s_info->task == current) {
			ret = -EISCONN;
			break;
		}
	}
	spin_unlock_irqrestore(&uintrfd_ctx->sender_lock, lock_flags);

	if (ret)
		goto out_fdput;

	s_info = kzalloc(sizeof(*s_info), GFP_KERNEL);
	if (!s_info) {
		ret = -ENOMEM;
		goto out_fdput;
	}

	ret = do_uintr_register_sender(uintrfd_ctx->r_info, s_info);
	if (ret) {
		kfree(s_info);
		goto out_fdput;
	}

	spin_lock_irqsave(&uintrfd_ctx->sender_lock, lock_flags);
	list_add(&s_info->node, &uintrfd_ctx->sender_list);
	spin_unlock_irqrestore(&uintrfd_ctx->sender_lock, lock_flags);

	ret = s_info->uitt_index;

out_fdput:
	pr_debug("send: register sender task=%d flags %d ret(uipi_id)=%d\n",
		 current->pid, flags, ret);

	fdput(f);
	return ret;
}

/*
 * sys_uintr_unregister_sender - Unregister user inter-processor interrupt sender.
 */
SYSCALL_DEFINE2(uintr_unregister_sender, int, uintrfd, unsigned int, flags)
{
	struct uintr_sender_info *s_info;
	struct uintrfd_ctx *uintrfd_ctx;
	struct file *uintr_f;
	unsigned long lock_flags;
	struct fd f;
	int ret;

	if (!uintr_arch_enabled())
		return -EOPNOTSUPP;

	if (flags)
		return -EINVAL;

	f = fdget(uintrfd);
	uintr_f = f.file;
	if (!uintr_f)
		return -EBADF;

	if (uintr_f->f_op != &uintrfd_fops) {
		ret = -EOPNOTSUPP;
		goto out_fdput;
	}

	uintrfd_ctx = (struct uintrfd_ctx *)uintr_f->private_data;

	ret = -EINVAL;
	spin_lock_irqsave(&uintrfd_ctx->sender_lock, lock_flags);
	list_for_each_entry(s_info, &uintrfd_ctx->sender_list, node) {
		if (s_info->task == current) {
			ret = 0;
			list_del(&s_info->node);
			do_uintr_unregister_sender(uintrfd_ctx->r_info, s_info);
			break;
		}
	}
	spin_unlock_irqrestore(&uintrfd_ctx->sender_lock, lock_flags);

	pr_debug("send: unregister sender uintrfd %d for task=%d ret %d\n",
		 uintrfd, current->pid, ret);

out_fdput:
	fdput(f);
	return ret;
}

/*
 * sys_uintr_wait - Wait for a user interrupt
 */
SYSCALL_DEFINE1(uintr_wait, unsigned int, flags)
{
	if (!uintr_arch_enabled())
		return -EOPNOTSUPP;

	if (flags)
		return -EINVAL;

	/* TODO: Add a timeout option */
	return uintr_receiver_wait();
}
