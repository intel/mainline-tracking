// SPDX-License-Identifier: GPL-2.0
/*
 * intel_tdx_attest.c - TDX guest attestation interface driver.
 *
 * Implements user interface to trigger attestation process and
 * read the TD Quote result.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * Author:
 *     Kuppuswamy Sathyanarayanan <sathyanarayanan.kuppuswamy@linux.intel.com>
 */

#define pr_fmt(fmt) "x86/tdx: attest: " fmt

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/set_memory.h>
#include <linux/io.h>
#include <asm/apic.h>
#include <asm/tdx.h>
#include <asm/irq_vectors.h>
#include <uapi/misc/tdx.h>

#define VERSION				"1.0"

/* Used in Quote memory allocation */
#define QUOTE_SIZE			(2 * PAGE_SIZE)

/* Mutex to synchronize attestation requests */
static DEFINE_MUTEX(attestation_lock);
/* Completion object to track attestation status */
static DECLARE_COMPLETION(attestation_done);

static void attestation_callback_handler(void)
{
	complete(&attestation_done);
}

static long tdg_attest_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	u8 *reportdata;
	long ret = 0;
	u64 data = virt_to_phys(file->private_data);
	void __user *argp = (void __user *)arg;

	if (!data)
		return -EINVAL;

	reportdata = kzalloc(TDX_TDREPORT_LEN, GFP_KERNEL);
	if (!reportdata)
		return -ENOMEM;

	mutex_lock(&attestation_lock);

	switch (cmd) {
	case TDX_CMD_GET_TDREPORT:
		if (copy_from_user(reportdata, argp, TDX_REPORT_DATA_LEN)) {
			ret = -EFAULT;
			break;
		}

		/* Generate TDREPORT_STRUCT */
		if (tdx_mcall_tdreport(data, virt_to_phys(reportdata))) {
			ret = -EIO;
			break;
		}

		if (copy_to_user(argp, file->private_data, TDX_TDREPORT_LEN))
			ret = -EFAULT;
		break;
	case TDX_CMD_GEN_QUOTE:
		if (copy_from_user(reportdata, argp, TDX_REPORT_DATA_LEN)) {
			ret = -EFAULT;
			break;
		}

		/* Generate TDREPORT_STRUCT */
		if (tdx_mcall_tdreport(data, virt_to_phys(reportdata))) {
			ret = -EIO;
			break;
		}

		ret = set_memory_decrypted((unsigned long)file->private_data,
					   1UL << get_order(QUOTE_SIZE));
		if (ret)
			break;

		/* Submit GetQuote Request */
		if (tdx_hcall_get_quote(data)) {
			ret = -EIO;
			break;
		}

		apic->send_IPI_all(HYPERVISOR_CALLBACK_VECTOR);

		/* Wait for attestation completion */
		wait_for_completion_interruptible(&attestation_done);

		ret = set_memory_encrypted((unsigned long)file->private_data,
					   1UL << get_order(QUOTE_SIZE));

		break;
	case TDX_CMD_GET_QUOTE_SIZE:
		if (put_user(QUOTE_SIZE, (u64 __user *)argp))
			ret = -EFAULT;

		break;
	default:
		pr_err("cmd %d not supported\n", cmd);
		break;
	}

	mutex_unlock(&attestation_lock);

	kfree(reportdata);

	return ret;
}

static vm_fault_t tdg_attest_vm_fault(struct vm_fault *vmf)
{
	struct page *page;

	if (vmf->vma->vm_private_data) {
		page = virt_to_page(vmf->vma->vm_private_data);
		get_page(page);
		vmf->page = page;
	}

	return 0;
}

static const struct vm_operations_struct tdg_attest_vm_ops = {
	.fault = tdg_attest_vm_fault,
};

static int tdg_attest_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_ops = &tdg_attest_vm_ops;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_private_data = file->private_data;

	return 0;
}

static int tdg_attest_open(struct inode *inode, struct file *file)
{
	tdg_event_notify_handler = attestation_callback_handler;

	file->private_data = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
						      get_order(QUOTE_SIZE));

	return !file->private_data ? -ENOMEM : 0;
}

static int tdg_attest_release(struct inode *inode, struct file *file)
{
	tdg_event_notify_handler = NULL;
	free_pages((unsigned long)file->private_data, get_order(QUOTE_SIZE));
	file->private_data = NULL;

	return 0;
}

static const struct file_operations tdg_attest_fops = {
	.owner		= THIS_MODULE,
	.open		= tdg_attest_open,
	.release	= tdg_attest_release,
	.mmap		= tdg_attest_mmap,
	.unlocked_ioctl	= tdg_attest_ioctl,
	.llseek		= no_llseek,
};

static struct miscdevice tdg_attest_device = {
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = "tdx-attest",
	.fops           = &tdg_attest_fops,
};
module_misc_device(tdg_attest_device);

MODULE_AUTHOR("Kuppuswamy Sathyanarayanan <sathyanarayanan.kuppuswamy@linux.intel.com>");
MODULE_DESCRIPTION("TDX attestation driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
