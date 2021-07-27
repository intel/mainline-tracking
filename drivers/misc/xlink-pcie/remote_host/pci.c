// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel XPCIe XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#ifdef XLINK_PCIE_RH_DRV_AER
#include <linux/aer.h>
#endif

#include "pci.h"

#include "../common/core.h"
#include "../common/util.h"

#define MAX_SW_DEVID_RETRIES 10

static int aspm_enable;
module_param(aspm_enable, int, 0664);
MODULE_PARM_DESC(aspm_enable, "enable ASPM");

static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_mutex);

struct xpcie_dev *intel_xpcie_get_device_by_id(u32 id)
{
	struct xpcie_dev *xdev;

	mutex_lock(&dev_list_mutex);

	if (list_empty(&dev_list)) {
		mutex_unlock(&dev_list_mutex);
		return NULL;
	}

	list_for_each_entry(xdev, &dev_list, list) {
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
		if (xdev->sw_devid == id || xdev->devid == id) {
#else
		if (xdev->devid == id) {
#endif
			mutex_unlock(&dev_list_mutex);
				return xdev;
		}
		mutex_unlock(&dev_list_mutex);
	}
	return NULL;
}

struct xpcie_dev *intel_xpcie_create_device(u32 sw_device_id,
					    struct pci_dev *pdev)
{
	struct xpcie_dev *xdev = kzalloc(sizeof(*xdev), GFP_KERNEL);

	if (!xdev)
		return NULL;

	xdev->devid = sw_device_id;
	snprintf(xdev->name, XPCIE_MAX_NAME_LEN, "%02x:%02x.%x",
		 pdev->bus->number,
		 PCI_SLOT(pdev->devfn),
		 PCI_FUNC(pdev->devfn));

	mutex_init(&xdev->lock);
	return xdev;
}

void intel_xpcie_remove_device(struct xpcie_dev *xdev)
{
	mutex_destroy(&xdev->lock);
	kfree(xdev);
}

void intel_xpcie_list_add_device(struct xpcie_dev *xdev)
{
	mutex_lock(&dev_list_mutex);

	list_add_tail(&xdev->list, &dev_list);

	mutex_unlock(&dev_list_mutex);
}

void intel_xpcie_list_del_device(struct xpcie_dev *xdev)
{
	mutex_lock(&dev_list_mutex);

	list_del(&xdev->list);

	mutex_unlock(&dev_list_mutex);
}

static void intel_xpcie_pci_set_aspm(struct xpcie_dev *xdev, int aspm)
{
	u16 link_control;
	u8 cap_exp;

	cap_exp = pci_find_capability(xdev->pci, PCI_CAP_ID_EXP);
	if (!cap_exp) {
		dev_err(&xdev->pci->dev, "failed to find pcie capability\n");
		return;
	}

	pci_read_config_word(xdev->pci, cap_exp + PCI_EXP_LNKCTL,
			     &link_control);
	link_control &= ~(PCI_EXP_LNKCTL_ASPMC);
	link_control |= (aspm & PCI_EXP_LNKCTL_ASPMC);
	pci_write_config_word(xdev->pci, cap_exp + PCI_EXP_LNKCTL,
			      link_control);
}

#ifdef XLINK_PCIE_RH_DRV_AER
static int intel_xpcie_mask_surprise_link_down(struct xpcie_dev *xdev,
					       bool mask)
{
	struct pci_dev *pdev = xdev->pci;
	u32 reg32;
	int pos;

	pos = pdev->aer_cap;
	if (!pos)
		return -ENODEV;

	pci_read_config_dword(pdev, pos + PCI_ERR_UNCOR_MASK, &reg32);
	if (mask)
		reg32 |= PCI_ERR_UNC_SURPDN;
	else
		reg32 &= ~PCI_ERR_UNC_SURPDN;
	pci_write_config_dword(pdev, pos + PCI_ERR_UNCOR_MASK, reg32);

	return 0;
}
#endif

static void intel_xpcie_pci_unmap_bar(struct xpcie_dev *xdev)
{
	if (xdev->xpcie.bar0) {
		iounmap((void __iomem *)xdev->xpcie.bar0);
		xdev->xpcie.bar0 = NULL;
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
		xdev->xpcie.doorbell_base = xdev->xpcie.bar0;
#endif
	}

	if (xdev->xpcie.io_comm) {
		iounmap(xdev->xpcie.io_comm);
		xdev->xpcie.io_comm = NULL;
	}

	if (xdev->xpcie.mmio)
		xdev->xpcie.mmio = NULL;

	if (xdev->xpcie.bar4) {
		iounmap((void __iomem *)xdev->xpcie.bar4);
		xdev->xpcie.bar4 = NULL;
	}
}

static int intel_xpcie_pci_map_bar(struct xpcie_dev *xdev)
{
	if (pci_resource_len(xdev->pci, 2) < XPCIE_IO_COMM_SIZE) {
		dev_err(&xdev->pci->dev, "device BAR region is too small\n");
		return -EIO;
	}

	xdev->xpcie.bar0 = (void __force *)pci_ioremap_bar(xdev->pci, 0);
	if (!xdev->xpcie.bar0) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR0\n");
		goto bar_error;
	}
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	xdev->xpcie.doorbell_base = xdev->xpcie.bar0;
#endif
	xdev->xpcie.io_comm = (void __force *)pci_ioremap_bar(xdev->pci, 2);
	if (!xdev->xpcie.io_comm) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR2\n");
		goto bar_error;
	}
	xdev->xpcie.mmio = (void __force *)
			   (xdev->xpcie.io_comm + XPCIE_MMIO_OFFSET);
	if (!xdev->xpcie.mmio) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR2\n");
		goto bar_error;
	}

	xdev->xpcie.bar4 = (void __force *)pci_ioremap_wc_bar(xdev->pci, 4);
	if (!xdev->xpcie.bar4) {
		dev_err(&xdev->pci->dev, "failed to ioremap BAR4\n");
		goto bar_error;
	}

	return 0;

bar_error:
	intel_xpcie_pci_unmap_bar(xdev);
	return -EIO;
}

static irqreturn_t intel_xpcie_core_interrupt(int irq, void *args)
{
	struct xpcie_dev *xdev = args;
	enum xpcie_stage stage;
	u8 event;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	if (xdev->xpcie.status != XPCIE_STATUS_READY &&
	    xdev->xpcie.status != XPCIE_STATUS_RUN) {
		stage = intel_xpcie_check_magic(xdev);
		if (stage == STAGE_ROM ||
		    stage == STAGE_UBOOT ||
		    stage == STAGE_RECOV)
			schedule_work(&xdev->irq_event);
	}
#endif

	event = intel_xpcie_get_doorbell(&xdev->xpcie, FROM_DEVICE, DEV_EVENT);
	if (event == DEV_SHUTDOWN || event == 0xFF) {
		pr_info("%s: shutdown_event (event=0x%x)\n", __func__, event);
		schedule_delayed_work(&xdev->shutdown_event, 0);
		return IRQ_HANDLED;
	}
	if (event == PREP_FLR_RESET_ACK || event == FLR_RESET_ACK) {
		schedule_work(&xdev->flr_event);
		return IRQ_HANDLED;
	}

	if (likely(xdev->core_irq_callback))
		return xdev->core_irq_callback(irq, args);

	return IRQ_HANDLED;
}

int intel_xpcie_pci_register_irq(struct xpcie_dev *xdev, irq_handler_t irq_handler)
{
	if (xdev->xpcie.status != XPCIE_STATUS_READY)
		return -EINVAL;

	xdev->core_irq_callback = irq_handler;

	return 0;
}

static void intel_xpcie_pci_irq_cleanup(struct xpcie_dev *xdev)
{
	int irq = pci_irq_vector(xdev->pci, 0);

	if (irq < 0)
		return;

	synchronize_irq(irq);
	free_irq(irq, xdev);
	pci_free_irq_vectors(xdev->pci);
}

static int intel_xpcie_pci_irq_init(struct xpcie_dev *xdev)
{
	int rc, irq;

	rc = pci_alloc_irq_vectors(xdev->pci, 1, 1, PCI_IRQ_MSI);
	if (rc < 0) {
		dev_err(&xdev->pci->dev,
			"failed to allocate %d MSI vectors\n", 1);
		return rc;
	}

	irq = pci_irq_vector(xdev->pci, 0);
	if (irq < 0) {
		dev_err(&xdev->pci->dev, "failed to get irq\n");
		rc = irq;
		goto error_irq;
	}
	rc = request_irq(irq, &intel_xpcie_core_interrupt, 0,
			 XPCIE_DRIVER_NAME, xdev);
	if (rc) {
		dev_err(&xdev->pci->dev, "failed to request irq\n");
		goto error_irq;
	}

	return 0;

error_irq:
	pci_free_irq_vectors(xdev->pci);
	return rc;
}

static void xpcie_device_poll(struct work_struct *work)
{
	struct xpcie_dev *xdev = container_of(work, struct xpcie_dev,
					      wait_event.work);
	enum xpcie_stage stage = intel_xpcie_check_magic(xdev);
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	u8 max_functions, event, fn_no;
	static u8 sw_devid_retries;
	bool poll_again = true;
	u32 devid;
#endif
	if (stage == STAGE_RECOV) {
		if (xdev->xpcie.status != XPCIE_STATUS_RECOVERY)
			xdev->xpcie.status = XPCIE_STATUS_RECOVERY;
	} else if (stage == STAGE_OS) {
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
		event = intel_xpcie_get_doorbell(&xdev->xpcie,
						 FROM_DEVICE, DEV_EVENT);
		if (event == PHY_ID_RECIEVED_ACK) {
			intel_xpcie_set_doorbell(&xdev->xpcie, FROM_DEVICE,
						 DEV_EVENT, NO_OP);
			xdev->xpcie.status = XPCIE_STATUS_READY;
			intel_xpcie_pci_notify_event(xdev,
						     NOTIFY_DEVICE_CONNECTED);
			sw_devid_retries = 0;
			poll_again = false;
		} else {
			if (sw_devid_retries++ == MAX_SW_DEVID_RETRIES) {
				sw_devid_retries = 0;
				dev_err(&xdev->pci->dev,
					"ACK for sw_devid %x not rx'ed\n",
					xdev->sw_devid);
				poll_again = false;
			}

			fn_no = PCI_FUNC(xdev->pci->devfn);
			devid = PCI_BUS_NUM(xdev->devid) << 8 |
				PCI_SLOT(xdev->pci->devfn);
			max_functions =
				intel_xpcie_get_max_functions(&xdev->xpcie);
			xdev->sw_devid =
				intel_xpcie_create_sw_device_id(fn_no,
								devid,
								max_functions);
			dev_info(&xdev->pci->dev,
				 "sw_devid=%x, fn=%d, max_functions=%d\n",
				 xdev->sw_devid, fn_no, max_functions);
			intel_xpcie_set_sw_device_id(&xdev->xpcie,
						     xdev->sw_devid);
			intel_xpcie_pci_raise_irq(xdev, PHY_ID_UPDATED, 1);
		}
#else
		xdev->xpcie.status = XPCIE_STATUS_READY;
		poll_again = false;
#endif
	}

	if (poll_again)
		schedule_delayed_work(&xdev->wait_event,
				      msecs_to_jiffies(2000));
}

static int intel_xpcie_pci_prepare_dev_reset(struct xpcie_dev *xdev,
					     bool notify,
					     enum xpcie_event_type type)
{
	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	if (xdev->core_irq_callback) {
		xdev->core_irq_callback = NULL;
		intel_xpcie_core_cleanup(&xdev->xpcie);
	}
	xdev->xpcie.status = XPCIE_STATUS_OFF;
	if (notify)
		intel_xpcie_pci_raise_irq(xdev, DEV_EVENT, type);

	mutex_unlock(&xdev->lock);

	return 0;
}

static void xpcie_device_shutdown(struct work_struct *work)
{
	struct xpcie_dev *xdev = container_of(work, struct xpcie_dev,
					      shutdown_event.work);

	intel_xpcie_pci_prepare_dev_reset(xdev, false, REQUEST_RESET);
}

static void intel_xpcie_handle_flr_work(struct work_struct *work)
{
	struct xpcie_dev *xdev = container_of(work, struct xpcie_dev,
					      flr_event);
	u8 event;

	event = intel_xpcie_get_doorbell(&xdev->xpcie, FROM_DEVICE, DEV_EVENT);
	if (event == PREP_FLR_RESET_ACK) {
		pr_info("FLR Reset Initiated ...\n");
		intel_xpcie_set_doorbell(&xdev->xpcie, FROM_DEVICE,
					 DEV_EVENT, NO_OP);
		pci_reset_function(xdev->pci);
	}
	if (event == FLR_RESET_ACK) {
		intel_xpcie_set_doorbell(&xdev->xpcie, FROM_DEVICE,
					 DEV_EVENT, NO_OP);
		xdev->xpcie.status = XPCIE_STATUS_READY;
		pr_info("FLR Reset Successful\n");
		intel_xpcie_pci_notify_event(xdev, NOTIFY_DEVICE_CONNECTED);
	}
}

static ssize_t flr_store(struct device *dev,
			 struct device_attribute *mattr,
			 const char *data, size_t count)
{
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);
	struct xpcie_dev *xdev = pci_get_drvdata(pdev);

	intel_xpcie_pci_flr_reset(xdev->sw_devid);

	return count;
}
static DEVICE_ATTR_WO(flr);

static const struct attribute *xpcie_sysfs_attrs[] = {
	&dev_attr_flr.attr,
	NULL,
};

static const struct attribute_group xpcie_dev_sysfs_attrs = {
	.attrs = (struct attribute **)xpcie_sysfs_attrs,
};

static int xpcie_device_init(struct xpcie_dev *xdev)
{
	int rc;

	INIT_DELAYED_WORK(&xdev->wait_event, xpcie_device_poll);
	INIT_DELAYED_WORK(&xdev->shutdown_event, xpcie_device_shutdown);
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	INIT_WORK(&xdev->irq_event, xpcie_device_irq);
	INIT_WORK(&xdev->flr_event, intel_xpcie_handle_flr_work);
#endif
	rc = intel_xpcie_pci_irq_init(xdev);
	if (rc)
		return rc;

	rc = sysfs_create_group(&xdev->pci->dev.kobj, &xpcie_dev_sysfs_attrs);
	if (rc) {
		dev_err(&xdev->pci->dev, "Failed to create sysfs entries\n");
		return rc;
	}

	pci_set_master(xdev->pci);

	xdev->xpcie.status = XPCIE_STATUS_UNINIT;

	init_waitqueue_head(&xdev->waitqueue);
	schedule_delayed_work(&xdev->wait_event, 0);

	return 0;
}

int intel_xpcie_pci_init(struct xpcie_dev *xdev, struct pci_dev *pdev)
{
	int rc;

	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	xdev->pci = pdev;
	pci_set_drvdata(pdev, xdev);

	rc = pci_enable_device_mem(xdev->pci);
	if (rc) {
		dev_err(&pdev->dev, "failed to enable pci device\n");
		goto error_exit;
	}

	rc = pci_request_regions(xdev->pci, XPCIE_DRIVER_NAME);
	if (rc) {
		dev_err(&pdev->dev, "failed to request mmio regions\n");
		goto error_req_mem;
	}

	rc = intel_xpcie_pci_map_bar(xdev);
	if (rc)
		goto error_map;

	rc = dma_set_mask_and_coherent(&xdev->pci->dev, DMA_BIT_MASK(64));
	if (rc) {
		dev_err(&pdev->dev, "failed to set dma mask\n");
		goto error_dma_mask;
	}

	intel_xpcie_pci_set_aspm(xdev, aspm_enable);

	rc = xpcie_device_init(xdev);
	if (!rc)
		goto init_exit;

#ifdef XLINK_PCIE_RH_DRV_AER
	intel_xpcie_mask_surprise_link_down(xdev, true);

	rc = pci_enable_pcie_error_reporting(xdev->pci);
	if (rc)
		dev_warn(&pdev->dev,
			 "failed to configure AER with rc %d\n", rc);
#endif

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	xdev->fl_vbuf = NULL;
	xdev->fl_buf_size = 0;
#endif

error_dma_mask:
	intel_xpcie_pci_unmap_bar(xdev);

error_map:
	pci_release_regions(xdev->pci);

error_req_mem:
	pci_disable_device(xdev->pci);

error_exit:
	xdev->xpcie.status = XPCIE_STATUS_ERROR;

init_exit:
	mutex_unlock(&xdev->lock);
	if (rc)
		mutex_destroy(&xdev->lock);
	return rc;
}

int intel_xpcie_pci_cleanup(struct xpcie_dev *xdev)
{
	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	sysfs_remove_group(&xdev->pci->dev.kobj, &xpcie_dev_sysfs_attrs);
	cancel_delayed_work(&xdev->wait_event);
	cancel_delayed_work(&xdev->shutdown_event);
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	cancel_work_sync(&xdev->irq_event);
#endif
	xdev->core_irq_callback = NULL;
	intel_xpcie_pci_irq_cleanup(xdev);

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	if (xdev->fl_vbuf) {
		dma_free_coherent(&xdev->pci->dev, xdev->fl_buf_size,
				  xdev->fl_vbuf, xdev->fl_phys_addr);
		xdev->fl_vbuf = NULL;
		xdev->fl_buf_size = 0;
	}
#endif
	intel_xpcie_core_cleanup(&xdev->xpcie);

#ifdef XLINK_PCIE_RH_DRV_AER
	pci_disable_pcie_error_reporting(xdev->pci);
	intel_xpcie_mask_surprise_link_down(xdev, false);
#endif
	intel_xpcie_pci_unmap_bar(xdev);
	pci_release_regions(xdev->pci);
	pci_disable_device(xdev->pci);
	pci_set_drvdata(xdev->pci, NULL);
	xdev->xpcie.status = XPCIE_STATUS_OFF;
	xdev->irq_enabled = false;

	mutex_unlock(&xdev->lock);

	return 0;
}

int intel_xpcie_pci_raise_irq(struct xpcie_dev *xdev,
			      enum xpcie_doorbell_type type,
			      u8 value)
{
	u16 pci_status;

	intel_xpcie_set_doorbell(&xdev->xpcie, TO_DEVICE, type, value);
	pci_read_config_word(xdev->pci, PCI_STATUS, &pci_status);
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	iowrite32(1, xdev->xpcie.doorbell_base);
#endif
	return 0;
}

u32 intel_xpcie_get_device_num(u32 *id_list)
{
	struct xpcie_dev *p;
	u32 num = 0;

	mutex_lock(&dev_list_mutex);

	if (list_empty(&dev_list)) {
		mutex_unlock(&dev_list_mutex);
		return 0;
	}

	list_for_each_entry(p, &dev_list, list) {
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
		*id_list++ = p->sw_devid;
#else
		*id_list++ = p->devid;
#endif
		num++;
	}
	mutex_unlock(&dev_list_mutex);

	return num;
}

int intel_xpcie_get_device_name_by_id(u32 id,
				      char *device_name, size_t name_size)
{
	struct xpcie_dev *xdev;
	size_t size;

	xdev = intel_xpcie_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	mutex_lock(&xdev->lock);

	size = (name_size > XPCIE_MAX_NAME_LEN) ?
		XPCIE_MAX_NAME_LEN : name_size;
	memcpy(device_name, xdev->name, size);

	mutex_unlock(&xdev->lock);

	return 0;
}

int intel_xpcie_get_device_status_by_id(u32 id, u32 *status)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;

	mutex_lock(&xdev->lock);
	*status = xdev->xpcie.status;
	mutex_unlock(&xdev->lock);

	return 0;
}

int intel_xpcie_pci_connect_device(u32 id)
{
	struct xpcie_dev *xdev;
	int rc = 0;

	xdev = intel_xpcie_get_device_by_id(id);
	if (!xdev)
		return -ENODEV;

	if (mutex_lock_interruptible(&xdev->lock))
		return -EINTR;

	if (xdev->xpcie.status == XPCIE_STATUS_RUN)
		goto connect_cleanup;

	if (xdev->xpcie.status == XPCIE_STATUS_OFF) {
		rc = -ENODEV;
		goto connect_cleanup;
	}

	if (xdev->xpcie.status != XPCIE_STATUS_READY) {
		rc = -EBUSY;
		goto connect_cleanup;
	}

	rc = intel_xpcie_core_init(&xdev->xpcie);
	if (rc < 0) {
		dev_err(&xdev->pci->dev, "failed to sync with device\n");
		goto connect_cleanup;
	}

connect_cleanup:
	mutex_unlock(&xdev->lock);
	return rc;
}

int intel_xpcie_pci_read(u32 id, void *data, size_t *size, u32 timeout)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;

	return intel_xpcie_core_read(&xdev->xpcie, data, size, timeout);
}

int intel_xpcie_pci_write(u32 id, void *data, size_t *size, u32 timeout)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(id);

	if (!xdev)
		return -ENODEV;

	return intel_xpcie_core_write(&xdev->xpcie, data, size, timeout);
}

int intel_xpcie_pci_reset_device(u32 id)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(id);

	if (!xdev)
		return -ENOMEM;

	return intel_xpcie_pci_prepare_dev_reset(xdev, true, REQUEST_RESET);
}

int intel_xpcie_pci_register_device_event(u32 sw_device_id,
					  xlink_device_event event_notif_fn)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(sw_device_id);

	if (!xdev)
		return -ENOMEM;

	xdev->event_fn = event_notif_fn;

	return 0;
}

int intel_xpcie_pci_unregister_device_event(u32 sw_device_id)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(sw_device_id);

	if (!xdev)
		return -ENOMEM;

	xdev->event_fn = NULL;

	return 0;
}

void intel_xpcie_pci_notify_event(struct xpcie_dev *xdev,
				  enum xlink_device_event_type event_type)
{
	if (event_type >= NUM_EVENT_TYPE)
		return;

	if (xdev->event_fn) {
		dev_info(&xdev->pci->dev,
			 "sw_devid=0x%x, event_type=%d\n",
			 xdev->sw_devid, event_type);

		xdev->event_fn(xdev->devid, event_type);
	}
}

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))

int intel_xpcie_pci_flr_reset(u32 id)
{
	struct xpcie_dev *xdev = intel_xpcie_get_device_by_id(id);
	int rc;

	if (!xdev)
		return -ENOMEM;

	rc = intel_xpcie_pci_prepare_dev_reset(xdev, true, PREP_FLR_RESET);
	if (!rc)
		intel_xpcie_pci_notify_event(xdev, NOTIFY_DEVICE_DISCONNECTED);

	return rc;
}

int intel_xpcie_pci_ack_flr_reset(u32 id)
{
	return 0;
}

struct xpcie_dev *intel_xpcie_get_device_by_name(const char *name)
{
	struct xpcie_dev *p;
	bool found = false;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(p, &dev_list, list) {
		if (!strncmp(p->name, name, XPCIE_MAX_NAME_LEN)) {
			found = true;
			break;
		}
	}
	mutex_unlock(&dev_list_mutex);

	if (!found)
		p = NULL;

	return p;
}

struct xpcie_dev *intel_xpcie_get_device_by_phys_id(u32 phys_id)
{
	struct xpcie_dev *xdev;

	mutex_lock(&dev_list_mutex);

	if (list_empty(&dev_list)) {
		mutex_unlock(&dev_list_mutex);
		return NULL;
	}

	list_for_each_entry(xdev, &dev_list, list) {
		if (xdev->devid == phys_id) {
			mutex_unlock(&dev_list_mutex);
			return xdev;
		}
	}
	mutex_unlock(&dev_list_mutex);

	return NULL;
}
#endif
