// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel Keem Bay XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/compiler.h>
#include <linux/pci_ids.h>
#include <linux/reboot.h>
#include <linux/xlink_drv_inf.h>
#include "../common/xpcie.h"
#include "../common/core.h"
#include "../common/util.h"
#include "../common/boot.h"
#include "struct.h"
#include "dma.h"

#define BAR2_MIN_SIZE SZ_16K
#define BAR4_MIN_SIZE SZ_16K
#define KMB_EP_OUTBOUND_MAP_MIN_SIZE SZ_16K

#define PCIE_REGS_PCIE_SYS_CFG_CORE 0x7c
#define PCIE_CFG_PBUS_NUM_OFFSET 8
#define PCIE_CFG_PBUS_NUM_MASK 0xFF
#define PCIE_CFG_PBUS_DEV_NUM_OFFSET 16
#define PCIE_CFG_PBUS_DEV_NUM_MASK 0x1F

#define PCIE_REGS_PCIE_INTR_ENABLE 0x18
#define PCIE_REGS_PCIE_INTR_FLAGS 0x1c
#define LBC_CII_EVENT_FLAG BIT(18)
#define PCIE_REGS_MEM_ACCESS_IRQ_VECTOR	0x180
#define PCIE_REGS_PCIE_ERR_INTR_FLAGS 0x24
#define LINK_REQ_RST_FLG BIT(15)

static struct pci_epf_header xpcie_header = {
	.vendorid = PCI_VENDOR_ID_INTEL,
	.deviceid = PCI_DEVICE_ID_INTEL_KEEMBAY,
	.baseclass_code = PCI_BASE_CLASS_MULTIMEDIA,
	.subclass_code = 0x0,
	.subsys_vendor_id = 0x0,
	.subsys_id = 0x0,
};

static const struct pci_epf_device_id xpcie_epf_ids[] = {
	{
		.name = "mxlk_pcie_epf",
	},
	{},
};

u32 xlink_sw_id;

static irqreturn_t intel_xpcie_err_interrupt(int irq, void *args)
{
	struct xpcie *xpcie = args;
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	u32 val;

	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);
	if (val & LINK_REQ_RST_FLG)
		intel_xpcie_ep_dma_reset(xpcie_epf->epf);

	iowrite32(val, xpcie_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);

	return IRQ_HANDLED;
}

static irqreturn_t intel_xpcie_host_interrupt(int irq, void *args)
{
	struct xpcie *xpcie = args;
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	u32 val;
	u8 event;

	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	if (val & LBC_CII_EVENT_FLAG) {
		iowrite32(LBC_CII_EVENT_FLAG,
			  xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);

		event = intel_xpcie_get_doorbell(xpcie, TO_DEVICE, DEV_EVENT);
		if (unlikely(event != NO_OP)) {
			intel_xpcie_set_doorbell(xpcie, TO_DEVICE,
						 DEV_EVENT, NO_OP);
			if (event == REQUEST_RESET)
				orderly_reboot();
			return IRQ_HANDLED;
		}

		if (likely(xpcie_epf->core_irq_callback))
			xpcie_epf->core_irq_callback(irq, xpcie);
	}

	return IRQ_HANDLED;
}

void intel_xpcie_register_host_irq(struct xpcie *xpcie, irq_handler_t func)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);

	xpcie_epf->core_irq_callback = func;
}

int intel_xpcie_raise_irq(struct xpcie *xpcie, enum xpcie_doorbell_type type)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, type, 1);

	return pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
}

static void __iomem *intel_xpcie_epc_alloc_addr(struct pci_epc *epc,
						phys_addr_t *phys_addr,
						size_t size)
{
	void __iomem *virt_addr;
	unsigned long flags;

	spin_lock_irqsave(&epc->lock, flags);
	virt_addr = pci_epc_mem_alloc_addr(epc, phys_addr, size);
	spin_unlock_irqrestore(&epc->lock, flags);

	return virt_addr;
}

static void intel_xpcie_epc_free_addr(struct pci_epc *epc,
				      phys_addr_t phys_addr,
				      void __iomem *virt_addr, size_t size)
{
	unsigned long flags;

	spin_lock_irqsave(&epc->lock, flags);
	pci_epc_mem_free_addr(epc, phys_addr, virt_addr, size);
	spin_unlock_irqrestore(&epc->lock, flags);
}

int intel_xpcie_copy_from_host_ll(struct xpcie *xpcie, int chan, int descs_num)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	return intel_xpcie_ep_dma_read_ll(epf, chan, descs_num);
}

int intel_xpcie_copy_to_host_ll(struct xpcie *xpcie, int chan, int descs_num)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	return intel_xpcie_ep_dma_write_ll(epf, chan, descs_num);
}

static int intel_xpcie_check_bar(struct pci_epf *epf,
				 struct pci_epf_bar *epf_bar,
				 enum pci_barno barno,
				 size_t size, u8 reserved_bar)
{
	if (reserved_bar & (1 << barno)) {
		dev_err(&epf->dev, "BAR%d is already reserved\n", barno);
		return -EFAULT;
	}

	if (epf_bar->size != 0 && epf_bar->size < size) {
		dev_err(&epf->dev, "BAR%d fixed size is not enough\n", barno);
		return -ENOMEM;
	}

	return 0;
}

static int intel_xpcie_configure_bar(struct pci_epf *epf,
				     const struct pci_epc_features
					*epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;
	int ret;

	for (i = BAR_0; i <= BAR_5; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			epf_bar->size = epc_features->bar_fixed_size[i];

		if (i == BAR_2) {
			ret = intel_xpcie_check_bar(epf, epf_bar, BAR_2,
						    BAR2_MIN_SIZE,
						    epc_features->reserved_bar);
			if (ret)
				return ret;
		}

		if (i == BAR_4) {
			ret = intel_xpcie_check_bar(epf, epf_bar, BAR_4,
						    BAR4_MIN_SIZE,
						    epc_features->reserved_bar);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void intel_xpcie_cleanup_bar(struct pci_epf *epf, enum pci_barno barno)
{
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	if (xpcie_epf->vaddr[barno]) {
		pci_epc_clear_bar(epc, epf->func_no, &epf->bar[barno]);
		pci_epf_free_space(epf, xpcie_epf->vaddr[barno], barno);
	}

	xpcie_epf->vaddr[barno] = NULL;
}

static void intel_xpcie_cleanup_bars(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	intel_xpcie_cleanup_bar(epf, BAR_2);
	intel_xpcie_cleanup_bar(epf, BAR_4);
	xpcie_epf->xpcie.io_comm = NULL;
	xpcie_epf->xpcie.mmio = NULL;
	xpcie_epf->xpcie.bar4 = NULL;
}

static int intel_xpcie_setup_bar(struct pci_epf *epf, enum pci_barno barno,
				 size_t min_size, size_t align)
{
	int ret;
	void *vaddr = NULL;
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epf_bar *bar = &epf->bar[barno];

	bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	if (!bar->size)
		bar->size = min_size;

	if (barno == BAR_4)
		bar->flags |= PCI_BASE_ADDRESS_MEM_PREFETCH;

	vaddr = pci_epf_alloc_space(epf, bar->size, barno, align);

	if (!vaddr) {
		dev_err(&epf->dev, "Failed to map BAR%d\n", barno);
		return -ENOMEM;
	}

	ret = pci_epc_set_bar(epc, epf->func_no, bar);

	if (ret) {
		pci_epf_free_space(epf, vaddr, barno);
		dev_err(&epf->dev, "Failed to set BAR%d\n", barno);
		return ret;
	}

	xpcie_epf->vaddr[barno] = vaddr;

	return 0;
}

static int intel_xpcie_setup_bars(struct pci_epf *epf, size_t align)
{
	int ret;

	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	ret = intel_xpcie_setup_bar(epf, BAR_2, BAR2_MIN_SIZE, align);
	if (ret)
		return ret;

	ret = intel_xpcie_setup_bar(epf, BAR_4, BAR4_MIN_SIZE, align);
	if (ret) {
		intel_xpcie_cleanup_bar(epf, BAR_2);
		return ret;
	}

	xpcie_epf->comm_bar = BAR_2;
	xpcie_epf->xpcie.io_comm = xpcie_epf->vaddr[BAR_2];
	xpcie_epf->xpcie.mmio = (void *)xpcie_epf->xpcie.io_comm +
				XPCIE_MMIO_OFFSET;

	xpcie_epf->bar4 = BAR_4;
	xpcie_epf->xpcie.bar4 = xpcie_epf->vaddr[BAR_4];

	return 0;
}

static int intel_xpcie_epf_bind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keembay_pcie *keembay = to_keembay_pcie(pci);
	const struct pci_epc_features *features;
	bool msi_capable = true;
	size_t align = 0;
	int ret;
	u32 bus_num = 0;
	u32 dev_num = 0;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	features = pci_epc_get_features(epc, epf->func_no);
	xpcie_epf->epc_features = features;
	if (features) {
		msi_capable = features->msi_capable;
		align = features->align;
		ret = intel_xpcie_configure_bar(epf, features);
		if (ret)
			return ret;
	}

	ret = intel_xpcie_setup_bars(epf, align);
	if (ret) {
		dev_err(&epf->dev, "BAR initialization failed\n");
		return ret;
	}

	xpcie_epf->irq = keembay->ev_irq;
	xpcie_epf->irq_dma = keembay->irq;
	xpcie_epf->irq_err = keembay->err_irq;
	xpcie_epf->apb_base = keembay->base;
	if (!strcmp(keembay->stepping, "A0")) {
		xpcie_epf->xpcie.legacy_a0 = true;
		xpcie_epf->xpcie.mmio->legacy_a0 = 1;
	} else {
		xpcie_epf->xpcie.legacy_a0 = false;
		xpcie_epf->xpcie.mmio->legacy_a0 = 0;
	}

	ret = intel_xpcie_ep_dma_init(epf);
	if (ret) {
		dev_err(&epf->dev, "DMA initialization failed\n");
		goto bind_error;
	}

	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_READY);

	ret = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_SYS_CFG_CORE);
	bus_num = (ret >> PCIE_CFG_PBUS_NUM_OFFSET) & PCIE_CFG_PBUS_NUM_MASK;
	dev_num = (ret >> PCIE_CFG_PBUS_DEV_NUM_OFFSET) &
			PCIE_CFG_PBUS_DEV_NUM_MASK;

	xlink_sw_id = (XLINK_DEV_INF_PCIE << XLINK_DEV_INF_TYPE_SHIFT) |
		   ((bus_num << 8 | dev_num) << XLINK_DEV_PHYS_ID_SHIFT) |
		   (XLINK_DEV_TYPE_KMB << XLINK_DEV_TYPE_SHIFT) |
		   (XLINK_DEV_SLICE_0 << XLINK_DEV_SLICE_ID_SHIFT) |
		   (XLINK_DEV_FUNC_VPU << XLINK_DEV_FUNC_SHIFT);

	ret = intel_xpcie_core_init(&xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "Core component configuration failed\n");
		goto bind_error;
	}

	/* Enable interrupt */
	writel(LBC_CII_EVENT_FLAG,
	       xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_ENABLE);
	ret = request_irq(xpcie_epf->irq, &intel_xpcie_host_interrupt,
			  0, XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request irq\n");
		goto bind_error;
	}

	ret = request_irq(xpcie_epf->irq_err, &intel_xpcie_err_interrupt, 0,
			  XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request error irq\n");
		free_irq(xpcie_epf->irq, &xpcie_epf->xpcie);
		goto bind_error;
	}

	if (!intel_xpcie_ep_dma_enabled(xpcie_epf->epf))
		intel_xpcie_ep_dma_reset(xpcie_epf->epf);

	xpcie_epf->xpcie.mmio->host_status = XPCIE_STATUS_UNINIT;
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_RUN);
	intel_xpcie_set_doorbell(&xpcie_epf->xpcie, FROM_DEVICE,
				 DEV_EVENT, NO_OP);
	strncpy(xpcie_epf->xpcie.io_comm->magic, XPCIE_BOOT_MAGIC_YOCTO,
		strlen(XPCIE_BOOT_MAGIC_YOCTO));

	return 0;

bind_error:
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_ERROR);
	strncpy(xpcie_epf->xpcie.io_comm->magic, XPCIE_BOOT_MAGIC_YOCTO,
		strlen(XPCIE_BOOT_MAGIC_YOCTO));

	return ret;
}

static void intel_xpcie_epf_unbind(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	free_irq(xpcie_epf->irq, &xpcie_epf->xpcie);
	free_irq(xpcie_epf->irq_err, &xpcie_epf->xpcie);

	intel_xpcie_core_cleanup(&xpcie_epf->xpcie);
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_READY);

	intel_xpcie_ep_dma_uninit(epf);

	pci_epc_stop(epc);

	intel_xpcie_cleanup_bars(epf);
}

static void intel_xpcie_epf_linkup(struct pci_epf *epf)
{
}

static int intel_xpcie_epf_probe(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf;
	struct device *dev = &epf->dev;

	xpcie_epf = devm_kzalloc(dev, sizeof(*xpcie_epf), GFP_KERNEL);
	if (!xpcie_epf)
		return -ENOMEM;

	epf->header = &xpcie_header;
	xpcie_epf->epf = epf;

	epf_set_drvdata(epf, xpcie_epf);

	return 0;
}

static void intel_xpcie_epf_shutdown(struct device *dev)
{
	struct pci_epf *epf = to_pci_epf(dev);
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	/* Notify host in case PCIe hot plug not supported */
	if (xpcie_epf && xpcie_epf->xpcie.status == XPCIE_STATUS_RUN) {
		intel_xpcie_set_doorbell(&xpcie_epf->xpcie, FROM_DEVICE,
					 DEV_EVENT, DEV_SHUTDOWN);
		pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
	}
}

static struct pci_epf_ops ops = {
	.bind = intel_xpcie_epf_bind,
	.unbind = intel_xpcie_epf_unbind,
	.linkup = intel_xpcie_epf_linkup,
};

static struct pci_epf_driver xpcie_epf_driver = {
	.driver.name = "mxlk_pcie_epf",
	.driver.shutdown = intel_xpcie_epf_shutdown,
	.probe = intel_xpcie_epf_probe,
	.id_table = xpcie_epf_ids,
	.ops = &ops,
	.owner = THIS_MODULE,
};

static int __init intel_xpcie_epf_init(void)
{
	int ret = -EBUSY;

	ret = pci_epf_register_driver(&xpcie_epf_driver);
	if (ret) {
		pr_err("Failed to register xlink pcie epf driver: %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(intel_xpcie_epf_init);

static void __exit intel_xpcie_epf_exit(void)
{
	pci_epf_unregister_driver(&xpcie_epf_driver);
}
module_exit(intel_xpcie_epf_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel");
MODULE_DESCRIPTION(XPCIE_DRIVER_DESC);
MODULE_VERSION(XPCIE_DRIVER_VERSION);
