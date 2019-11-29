// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
// Copyright(c) 2016-18 Intel Corporation.

#include <linux/acpi.h>
#include <linux/cdev.h>
#include <linux/mman.h>
#include <linux/platform_device.h>
#include <linux/security.h>
#include <linux/suspend.h>
#include <asm/traps.h>
#include "driver.h"
#include "encl.h"

MODULE_DESCRIPTION("Intel SGX Enclave Driver");
MODULE_AUTHOR("Jarkko Sakkinen <jarkko.sakkinen@linux.intel.com>");
MODULE_LICENSE("Dual BSD/GPL");

struct workqueue_struct *sgx_encl_wq;
u64 sgx_encl_size_max_32;
u64 sgx_encl_size_max_64;
u32 sgx_misc_reserved_mask;
u64 sgx_attributes_reserved_mask;
u64 sgx_xfrm_reserved_mask = ~0x3;
u32 sgx_xsave_size_tbl[64];

static int sgx_open(struct inode *inode, struct file *file)
{
	struct sgx_encl *encl;
	int ret;

	encl = kzalloc(sizeof(*encl), GFP_KERNEL);
	if (!encl)
		return -ENOMEM;

	atomic_set(&encl->flags, 0);
	kref_init(&encl->refcount);
	INIT_LIST_HEAD(&encl->va_pages);
	INIT_RADIX_TREE(&encl->page_tree, GFP_KERNEL);
	mutex_init(&encl->lock);
	INIT_LIST_HEAD(&encl->mm_list);
	spin_lock_init(&encl->mm_lock);

	ret = init_srcu_struct(&encl->srcu);
	if (ret) {
		kfree(encl);
		return ret;
	}

	file->private_data = encl;

	return 0;
}

static int sgx_release(struct inode *inode, struct file *file)
{
	struct sgx_encl *encl = file->private_data;
	struct sgx_encl_mm *encl_mm;

	for ( ; ; )  {
		spin_lock(&encl->mm_lock);

		if (list_empty(&encl->mm_list)) {
			encl_mm = NULL;
		} else {
			encl_mm = list_first_entry(&encl->mm_list,
						   struct sgx_encl_mm, list);
			list_del_rcu(&encl_mm->list);
		}

		spin_unlock(&encl->mm_lock);

		/* The list is empty, ready to go. */
		if (!encl_mm)
			break;

		synchronize_srcu(&encl->srcu);
		mmu_notifier_unregister(&encl_mm->mmu_notifier, encl_mm->mm);
		kfree(encl_mm);
	};

	mutex_lock(&encl->lock);
	atomic_or(SGX_ENCL_DEAD, &encl->flags);
	mutex_unlock(&encl->lock);

	kref_put(&encl->refcount, sgx_encl_release);
	return 0;
}

#ifdef CONFIG_COMPAT
static long sgx_compat_ioctl(struct file *filep, unsigned int cmd,
			      unsigned long arg)
{
	return sgx_ioctl(filep, cmd, arg);
}
#endif

static int sgx_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct sgx_encl *encl = file->private_data;
	int ret;

	ret = sgx_encl_may_map(encl, vma->vm_start, vma->vm_end,
			       vma->vm_flags & (VM_READ | VM_WRITE | VM_EXEC));
	if (ret)
		return ret;

	ret = sgx_encl_mm_add(encl, vma->vm_mm);
	if (ret)
		return ret;

	vma->vm_ops = &sgx_vm_ops;
	vma->vm_flags |= VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP | VM_IO;
	vma->vm_private_data = encl;

	return 0;
}

static unsigned long sgx_get_unmapped_area(struct file *file,
					   unsigned long addr,
					   unsigned long len,
					   unsigned long pgoff,
					   unsigned long flags)
{
	if (flags & MAP_PRIVATE)
		return -EINVAL;

	if (flags & MAP_FIXED)
		return addr;

	return current->mm->get_unmapped_area(file, addr, len, pgoff, flags);
}

static const struct file_operations sgx_encl_fops = {
	.owner			= THIS_MODULE,
	.open			= sgx_open,
	.release		= sgx_release,
	.unlocked_ioctl		= sgx_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= sgx_compat_ioctl,
#endif
	.mmap			= sgx_mmap,
	.get_unmapped_area	= sgx_get_unmapped_area,
};

const struct file_operations sgx_provision_fops = {
	.owner			= THIS_MODULE,
};

static struct bus_type sgx_bus_type = {
	.name	= "sgx",
};

static struct device sgx_encl_dev;
static struct cdev sgx_encl_cdev;
static struct device sgx_provision_dev;
static struct cdev sgx_provision_cdev;
static dev_t sgx_devt;

static void sgx_dev_release(struct device *dev)
{
}

static __init int sgx_dev_init(const char *name, struct device *dev,
			       struct cdev *cdev,
			       const struct file_operations *fops, int minor)
{
	int ret;

	device_initialize(dev);

	dev->bus = &sgx_bus_type;
	dev->devt = MKDEV(MAJOR(sgx_devt), minor);
	dev->release = sgx_dev_release;

	ret = dev_set_name(dev, name);
	if (ret) {
		put_device(dev);
		return ret;
	}

	cdev_init(cdev, fops);
	cdev->owner = THIS_MODULE;
	return 0;
}

int __init sgx_drv_init(void)
{
	unsigned int eax, ebx, ecx, edx;
	u64 attr_mask, xfrm_mask;
	int ret;
	int i;

	if (!boot_cpu_has(X86_FEATURE_SGX_LC)) {
		pr_info("The public key MSRs are not writable\n");
		return -ENODEV;
	}

	ret = bus_register(&sgx_bus_type);
	if (ret)
		return ret;

	ret = alloc_chrdev_region(&sgx_devt, 0, SGX_DRV_NR_DEVICES, "sgx");
	if (ret < 0)
		goto err_bus;

	cpuid_count(SGX_CPUID, 0, &eax, &ebx, &ecx, &edx);
	sgx_misc_reserved_mask = ~ebx | SGX_MISC_RESERVED_MASK;
	sgx_encl_size_max_64 = 1ULL << ((edx >> 8) & 0xFF);
	sgx_encl_size_max_32 = 1ULL << (edx & 0xFF);

	cpuid_count(SGX_CPUID, 1, &eax, &ebx, &ecx, &edx);

	attr_mask = (((u64)ebx) << 32) + (u64)eax;
	sgx_attributes_reserved_mask = ~attr_mask | SGX_ATTR_RESERVED_MASK;

	if (boot_cpu_has(X86_FEATURE_OSXSAVE)) {
		xfrm_mask = (((u64)edx) << 32) + (u64)ecx;

		for (i = 2; i < 64; i++) {
			cpuid_count(0x0D, i, &eax, &ebx, &ecx, &edx);
			if ((1 << i) & xfrm_mask)
				sgx_xsave_size_tbl[i] = eax + ebx;
		}

		sgx_xfrm_reserved_mask = ~xfrm_mask;
	}

	ret = sgx_dev_init("sgx/enclave", &sgx_encl_dev, &sgx_encl_cdev,
			   &sgx_encl_fops, 0);
	if (ret)
		goto err_chrdev_region;

	ret = sgx_dev_init("sgx/provision", &sgx_provision_dev,
			   &sgx_provision_cdev, &sgx_provision_fops, 1);
	if (ret)
		goto err_encl_dev;

	sgx_encl_wq = alloc_workqueue("sgx-encl-wq",
				      WQ_UNBOUND | WQ_FREEZABLE, 1);
	if (!sgx_encl_wq) {
		ret = -ENOMEM;
		goto err_provision_dev;
	}

	ret = cdev_device_add(&sgx_encl_cdev, &sgx_encl_dev);
	if (ret)
		goto err_encl_wq;

	ret = cdev_device_add(&sgx_provision_cdev, &sgx_provision_dev);
	if (ret)
		goto err_encl_cdev;

	return 0;

err_encl_cdev:
	cdev_device_del(&sgx_encl_cdev, &sgx_encl_dev);

err_encl_wq:
	destroy_workqueue(sgx_encl_wq);

err_provision_dev:
	put_device(&sgx_provision_dev);

err_encl_dev:
	put_device(&sgx_encl_dev);

err_chrdev_region:
	unregister_chrdev_region(sgx_devt, SGX_DRV_NR_DEVICES);

err_bus:
	bus_unregister(&sgx_bus_type);

	return ret;
}
