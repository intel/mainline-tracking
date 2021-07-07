// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel XPCIe XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>

#include "epf.h"
#include "../common/boot.h"

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
#define BAR0_MIN_SIZE			SZ_4K
#endif

#define BAR2_MIN_SIZE			SZ_16K

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
#define BAR4_MIN_SIZE			SZ_8K
#else
#define BAR4_MIN_SIZE			SZ_16K
#endif

#define PCIE_REGS_PCIE_INTR_ENABLE	0x18
#define PCIE_REGS_PCIE_INTR_FLAGS	0x1C
#define LBC_CII_EVENT_FLAG		BIT(18)
#define PCIE_REGS_PCIE_ERR_INTR_FLAGS	0x24
#define LINK_REQ_RST_FLG		BIT(15)

#define PCIE_REGS_PCIE_SYS_CFG_CORE	0x7C
#define PCIE_CFG_PBUS_NUM_OFFSET	8
#define PCIE_CFG_PBUS_NUM_MASK		0xFF
#define PCIE_CFG_PBUS_DEV_NUM_OFFSET	16
#define PCIE_CFG_PBUS_DEV_NUM_MASK	0x1F

static struct pci_epf_header xpcie_header = {
	.vendorid = PCI_VENDOR_ID_INTEL,
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	.deviceid = PCI_DEVICE_ID_INTEL_TBH_FULL,
#else
	.deviceid = PCI_DEVICE_ID_INTEL_KEEMBAY,
#endif
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

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
u32 xlink_sw_id;
#endif

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
#define XPCIE_MAX_NAME_LEN	(32)

static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_mutex);

u32 intel_xpcie_get_device_num(u32 *id_list)
{
	u32 num = 0;
	struct xpcie_epf *p;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(p, &dev_list, list) {
		if (p->sw_dev_id_updated) {
			*id_list++ = p->sw_devid;
			num++;
		}
	}
	mutex_unlock(&dev_list_mutex);
	return num;
}

struct xpcie_epf *intel_xpcie_get_device_by_id(u32 id)
{
	struct xpcie_epf *xdev;

	mutex_lock(&dev_list_mutex);

	list_for_each_entry(xdev, &dev_list, list) {
		if (xdev->sw_devid == id)
			break;
	}

	mutex_unlock(&dev_list_mutex);

	return xdev;
}

struct xpcie_epf *intel_xpcie_get_device_by_name(const char *name)
{
	struct xpcie_epf *p;

	mutex_lock(&dev_list_mutex);
	list_for_each_entry(p, &dev_list, list) {
		if (!strncmp(p->name, name, XPCIE_MAX_NAME_LEN))
			break;
	}
	mutex_unlock(&dev_list_mutex);
	return p;
}
#endif

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

void intel_xpcie_register_host_irq(struct xpcie *xpcie, irq_handler_t func)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);

	xpcie_epf->core_irq_callback = func;
}

int intel_xpcie_raise_irq(struct xpcie *xpcie, enum xpcie_doorbell_type type, u8 value)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, type, value);

	return pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
}

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
static irqreturn_t intel_xpcie_err_interrupt(int irq, void *args)
{
	struct xpcie_epf *xpcie_epf;
	struct xpcie *xpcie = args;
	u32 val;

	xpcie_epf = container_of(xpcie, struct xpcie_epf, xpcie);
	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);
	if (val & LINK_REQ_RST_FLG)
		intel_xpcie_ep_dma_reset(xpcie_epf->epf);

	iowrite32(val, xpcie_epf->apb_base + PCIE_REGS_PCIE_ERR_INTR_FLAGS);

	return IRQ_HANDLED;
}
#endif

void intel_xpcie_pci_notify_event(struct xpcie_epf *xdev,
				  enum xlink_device_event_type event_type)
{
	if (event_type >= NUM_EVENT_TYPE)
		return;

	if (xdev->event_fn)
		xdev->event_fn(xdev->sw_devid, event_type);
}

static irqreturn_t intel_xpcie_host_interrupt(int irq, void *args)
{
	struct xpcie *xpcie = args;
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	u8 event;
#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	u32 val;

	val = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	if (val & LBC_CII_EVENT_FLAG) {
		iowrite32(LBC_CII_EVENT_FLAG,
			  xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_FLAGS);
	}
#endif
	event = intel_xpcie_get_doorbell(xpcie, TO_DEVICE, DEV_EVENT);
	if (unlikely(event != NO_OP)) {
		intel_xpcie_set_doorbell(xpcie, TO_DEVICE, DEV_EVENT, NO_OP);
		if (event == REQUEST_RESET)
			orderly_reboot();

		if (event == PREP_FLR_RESET) {
			writel(0x1, xpcie->doorbell_clear);
			schedule_work(&xpcie_epf->flr_irq_event);
		}

		return IRQ_HANDLED;
	}

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	if (intel_xpcie_get_doorbell(xpcie, TO_DEVICE, PHY_ID_UPDATED)) {
		intel_xpcie_set_doorbell(xpcie, TO_DEVICE, PHY_ID_UPDATED, 0);
		if (!xpcie_epf->sw_dev_id_updated) {
			xpcie_epf->sw_devid =
				intel_xpcie_get_sw_device_id(xpcie);
			xpcie_epf->sw_dev_id_updated = true;
			dev_info(xpcie_to_dev(xpcie),
				 "pcie: func_no=%x swid updated=%x\n",
				 xpcie_epf->epf->func_no,
				 xpcie_epf->sw_devid);

			intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, DEV_EVENT,
						 PHY_ID_RECIEVED_ACK);
		}
	}
#endif
	if (likely(xpcie_epf->core_irq_callback))
		xpcie_epf->core_irq_callback(irq, xpcie);
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	else
		writel(0x1, xpcie->doorbell_clear); /* clearing the interrupt */
#endif

	return IRQ_HANDLED;
}

static void intel_xpcie_handle_flr_work(struct work_struct *work)
{
	struct xpcie_epf *xpcie_epf = container_of(work, struct xpcie_epf,
						   flr_irq_event);
	struct xpcie *xpcie = &xpcie_epf->xpcie;

	intel_xpcie_pci_notify_event(xpcie_epf, NOTIFY_DEVICE_DISCONNECTED);
	intel_xpcie_core_cleanup(&xpcie_epf->xpcie);
	intel_xpcie_ep_stop_dma(xpcie_epf->epf);
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_OFF);
	intel_xpcie_raise_irq(xpcie, DEV_EVENT, PREP_FLR_RESET_ACK);
	dev_info(&xpcie_epf->epf->dev, "FLR Initiated ..\n");
}

static irqreturn_t intel_xpcie_flr_interrupt(int irq_flr, void *args)
{
	struct xpcie *xpcie = args;
	struct xpcie_epf *xpcie_epf = container_of(xpcie, struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	if (!xpcie_epf->apb_base)
		return IRQ_HANDLED;

	writel(1 << (epf->func_no), xpcie_epf->apb_base + 0x20);
	readl(xpcie_epf->apb_base + 0x20);

	return IRQ_HANDLED;
}

int intel_xpcie_pci_flr_reset(u32 id)
{
	return 0;
}

int intel_xpcie_pci_ack_flr_reset(u32 id)
{
	struct xpcie_epf *xpcie_epf = intel_xpcie_get_device_by_id(id);
	struct xpcie *xpcie = &xpcie_epf->xpcie;

	if (intel_xpcie_get_device_status(xpcie) == XPCIE_STATUS_OFF) {
		msleep(1000);
		intel_xpcie_ep_start_dma(xpcie_epf->epf);
		intel_xpcie_core_init(xpcie);
		intel_xpcie_set_device_status(xpcie, XPCIE_STATUS_RUN);
		intel_xpcie_raise_irq(xpcie, DEV_EVENT, FLR_RESET_ACK);
		intel_xpcie_pci_notify_event(xpcie_epf,
					     NOTIFY_DEVICE_CONNECTED);
		dev_info(&xpcie_epf->epf->dev, "FLR Reset Successful\n");
	}

	return 0;
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
	int ret, i;

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
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	if (xpcie_epf->vaddr[barno]) {
		pci_epc_clear_bar(epc, epf->func_no, &epf->bar[barno]);
		pci_epf_free_space(epf, xpcie_epf->vaddr[barno], barno, PRIMARY_INTERFACE);
		xpcie_epf->vaddr[barno] = NULL;
	}
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

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
static int intel_xpcie_setup_bar(struct pci_epf *epf, enum pci_barno barno,
				 size_t size, size_t align)
{
	int ret;
	void __iomem *vaddr = NULL;
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epf_bar *bar = &epf->bar[barno];

	bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	bar->size = size;

	if (barno == 0) {
		bar->phys_addr =
			xpcie_epf->doorbell_base->start + (epf->func_no * 0x1000);
		vaddr = ioremap(bar->phys_addr, size);
	}

	if (barno == 2) {
		bar->phys_addr = xpcie_epf->mmr2.start;
		vaddr = ioremap(bar->phys_addr, size);
	}
	if (barno == 4) {
		bar->phys_addr = xpcie_epf->mmr4.start;
		vaddr = ioremap_cache(bar->phys_addr, size);
	}

	if (!vaddr) {
		dev_err(&epf->dev, "Failed to map BAR%d\n", barno);
		return -ENOMEM;
	}

	epf->bar[barno].phys_addr = bar->phys_addr;
	epf->bar[barno].size = size;
	epf->bar[barno].barno = barno;
	epf->bar[barno].flags |= upper_32_bits(size) ?
		PCI_BASE_ADDRESS_MEM_TYPE_64 :
		PCI_BASE_ADDRESS_MEM_TYPE_32;

	ret = pci_epc_set_bar(epc, epf->func_no, bar);

	if (ret) {
		pci_epf_free_space(epf, vaddr, barno, PRIMARY_INTERFACE);
		dev_err(&epf->dev, "Failed to set BAR%d\n", barno);
		return ret;
	}

	xpcie_epf->vaddr[barno] = vaddr;

	return 0;
}
#else
static int intel_xpcie_setup_bar(struct pci_epf *epf, enum pci_barno barno,
				 size_t min_size, size_t align)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epf_bar *bar = &epf->bar[barno];
	struct pci_epc *epc = epf->epc;
	void *vaddr;
	int ret;

	bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	if (!bar->size)
		bar->size = min_size;

	if (barno == BAR_4)
		bar->flags |= PCI_BASE_ADDRESS_MEM_PREFETCH;

	vaddr = pci_epf_alloc_space(epf, bar->size, barno, align, PRIMARY_INTERFACE);
	if (!vaddr) {
		dev_err(&epf->dev, "Failed to map BAR%d\n", barno);
		return -ENOMEM;
	}

	ret = pci_epc_set_bar(epc, epf->func_no, bar);
	if (ret) {
		pci_epf_free_space(epf, vaddr, barno, PRIMARY_INTERFACE);
		dev_err(&epf->dev, "Failed to set BAR%d\n", barno);
		return ret;
	}

	xpcie_epf->vaddr[barno] = vaddr;

	return 0;
}
#endif

static int intel_xpcie_setup_bars(struct pci_epf *epf, size_t align)
{
	int ret;

	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	ret = intel_xpcie_setup_bar(epf, BAR_0, BAR0_MIN_SIZE, align);
	if (ret)
		return ret;
	xpcie_epf->doorbell_bar = BAR_0;
	xpcie_epf->xpcie.doorbell_base = xpcie_epf->vaddr[BAR_0];
#endif
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

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
static void intel_xpcie_enable_multi_functions(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	unsigned long doorbell_clr_addr = 0;
	struct platform_device *pdev = NULL;
	struct pci_epc *epc = epf->epc;
	size_t doorbell_clr_size = 0;
	struct device *dev = NULL;
	int ret = 0;

	dev = epc->dev.parent;
	pdev = to_platform_device(dev);

	switch (epf->func_no) {
	case 0:
		if (xpcie_epf->tbh_half) {
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pdev->dev.of_node, 8);
		} else {
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pdev->dev.of_node, 16);
			}
		if (ret) {
			dev_warn(&epc->dev,
				 "of_reserved_mem_device_init_by_idx(): %d\n",
				 ret);
		}
		epf->dev.dma_mask = dev->dma_mask;
		epf->dev.coherent_dma_mask = dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pdev->dev.of_node, true);
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed of_dma_configure()ret=%d\n", ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed to set dma mask. ret=%d\n", ret);
		}
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start;
		doorbell_clr_size = 4;
		break;
	case 1:
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x8;
		doorbell_clr_size = 4;
		break;
	case 2:
		if (xpcie_epf->tbh_half)
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pdev->dev.of_node, 9);
		else
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pdev->dev.of_node, 17);
		if (ret) {
			dev_warn(&epc->dev,
				 "of_reserved_mem_device_init_by_idx(): %d\n",
				 ret);
		}
		epf->dev.dma_mask = dev->dma_mask;
		epf->dev.coherent_dma_mask = dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pdev->dev.of_node, true);
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed of_dma_configure()ret=%d\n", ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed to set dma mask. ret=%d\n", ret);
		}
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x14;
		doorbell_clr_size = 4;
		break;
	case 3:
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x1C;
		doorbell_clr_size = 4;
		break;
	case 4:
		ret = of_reserved_mem_device_init_by_idx
			(&epf->dev, pdev->dev.of_node, 18);
		if (ret) {
			dev_warn(&epc->dev,
				 "of_xx_mem_device_init_by_idx(): 18 ret=%d\n",
				 ret);
		}
		epf->dev.dma_mask = dev->dma_mask;
		epf->dev.coherent_dma_mask = dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pdev->dev.of_node, true);
		if (ret) {
			dev_warn(&epc->dev, "failed of_dma_configure(): %d\n",
				 ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev, "failed to set dma mask ret=%d\n",
				 ret);
		}
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x28;
		doorbell_clr_size = 4;
		break;
	case 5:
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x30;
		doorbell_clr_size = 4;
		break;
	case 6:
		ret = of_reserved_mem_device_init_by_idx
			(&epf->dev, pdev->dev.of_node, 19);
		if (ret) {
			dev_warn(&epc->dev,
				 "of_xx_mem_device_init_by_idx(): 19 ret=%d\n",
				 ret);
		}
		epf->dev.dma_mask = dev->dma_mask;
		epf->dev.coherent_dma_mask = dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pdev->dev.of_node, true);
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed of_dma_configure() ret:%d\n",
				 ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev, "failed to set dma mask ret=%d\n",
				 ret);
		}
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x3C;
		doorbell_clr_size = 4;
		break;
	case 7:
		doorbell_clr_addr = xpcie_epf->doorbell_clear->start + 0x44;
		doorbell_clr_size = 4;
		break;
	}

	xpcie_epf->xpcie.doorbell_clear =
				ioremap(doorbell_clr_addr, doorbell_clr_size);
	intel_xpcie_set_max_functions(&xpcie_epf->xpcie, epc->max_functions);
	list_add_tail(&xpcie_epf->list, &dev_list);
	snprintf(xpcie_epf->name, XPCIE_MAX_NAME_LEN, "%s_func%x", epf->name,
		 epf->func_no);
	ret = request_irq(xpcie_epf->irq_doorbell,
			  &intel_xpcie_host_interrupt, 0, XPCIE_DRIVER_NAME,
			  &xpcie_epf->xpcie);
	if (ret)
		dev_err(&epf->dev, "failed to request irq\n");

	ret = request_irq(xpcie_epf->irq_flr, &intel_xpcie_flr_interrupt, 0,
			  XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret)
		dev_err(&epf->dev, "failed to request FLR irq\n");
	INIT_WORK(&xpcie_epf->flr_irq_event, intel_xpcie_handle_flr_work);
}

static int intel_xpcie_epf_get_platform_data(struct device *dev,
					     struct xpcie_epf *xpcie_epf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pci_epf *epf = xpcie_epf->epf;
	struct pci_epc *epc = epf->epc;
	struct device_node *np;
	struct resource *res;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	xpcie_epf->dbi_base =
		devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->dbi_base))
		return PTR_ERR(xpcie_epf->dbi_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	xpcie_epf->apb_base =
		devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->apb_base))
		return PTR_ERR(xpcie_epf->apb_base);

	xpcie_epf->irq_doorbell = irq_of_parse_and_map(pdev->dev.of_node,
						       epf->func_no + 2);
	xpcie_epf->irq_wdma = irq_of_parse_and_map(pdev->dev.of_node,
						   epf->func_no + 10);
	xpcie_epf->irq_rdma = irq_of_parse_and_map(pdev->dev.of_node,
						   epf->func_no + 18);
	xpcie_epf->irq_flr = irq_of_parse_and_map(pdev->dev.of_node,
						  epf->func_no + 26);

	xpcie_epf->doorbell_base = platform_get_resource_byname(pdev,
								IORESOURCE_MEM,
								"doorbell");
	xpcie_epf->doorbell_clear = platform_get_resource_byname(pdev,
								 IORESOURCE_MEM,
								 "doorbellclr");

	np = of_parse_phandle(pdev->dev.of_node,
			      "memory-region", epf->func_no * 2);
	ret = of_address_to_resource(np, 0, &xpcie_epf->mmr2);

	np = of_parse_phandle(pdev->dev.of_node,
			      "memory-region", (epf->func_no * 2) + 1);
	ret = of_address_to_resource(np, 0, &xpcie_epf->mmr4);

	ret = of_property_read_u8(pdev->dev.of_node,
				  "max-functions",
				  &epc->max_functions);
	if (epc->max_functions == 8)
		xpcie_epf->tbh_half = 0;
	else if (epc->max_functions == 4)
		xpcie_epf->tbh_half = 1;

	return ret;
}
#else
static int intel_xpcie_epf_get_platform_data(struct device *dev,
					     struct xpcie_epf *xpcie_epf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *soc_node, *version_node;
	struct resource *res;
	const char *prop;
	int prop_size;

	xpcie_epf->irq_dma = platform_get_irq_byname(pdev, "intr");
	if (xpcie_epf->irq_dma < 0) {
		dev_err(&xpcie_epf->epf->dev, "failed to get IRQ: %d\n",
			xpcie_epf->irq_dma);
		return -EINVAL;
	}

	xpcie_epf->irq_err = platform_get_irq_byname(pdev, "err_intr");
	if (xpcie_epf->irq_err < 0) {
		dev_err(&xpcie_epf->epf->dev, "failed to get erroe IRQ: %d\n",
			xpcie_epf->irq_err);
		return -EINVAL;
	}

	xpcie_epf->irq = platform_get_irq_byname(pdev, "ev_intr");
	if (xpcie_epf->irq < 0) {
		dev_err(&xpcie_epf->epf->dev, "failed to get event IRQ: %d\n",
			xpcie_epf->irq);
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	xpcie_epf->apb_base =
		devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->apb_base))
		return PTR_ERR(xpcie_epf->apb_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	xpcie_epf->dbi_base =
		devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(xpcie_epf->dbi_base))
		return PTR_ERR(xpcie_epf->dbi_base);

	memcpy(xpcie_epf->stepping, "B0", 2);
	soc_node = of_get_parent(pdev->dev.of_node);
	if (soc_node) {
		version_node = of_get_child_by_name(soc_node, "version-info");
		if (version_node) {
			prop = of_get_property(version_node, "stepping",
					       &prop_size);
			if (prop && prop_size <= KEEMBAY_XPCIE_STEPPING_MAXLEN)
				memcpy(xpcie_epf->stepping, prop, prop_size);
			of_node_put(version_node);
		}
		of_node_put(soc_node);
	}

	return 0;
}
#endif

static ssize_t swdev_id_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct pci_epf *epf = container_of(dev, struct pci_epf, dev);
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	if (xpcie_epf->sw_dev_id_updated)
		snprintf(buf, 4096, "%s\n", "okay");
	else
		snprintf(buf, 4096, "%s\n", "disabled");

	return strlen(buf);
}
static DEVICE_ATTR_RO(swdev_id);

static ssize_t ack_flr_store(struct device *dev,
			     struct device_attribute *mattr,
			     const char *data, size_t count)
{
	struct pci_epf *epf = container_of(dev, struct pci_epf, dev);
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);

	intel_xpcie_pci_ack_flr_reset(xpcie_epf->sw_devid);

	return count;
}
static DEVICE_ATTR_WO(ack_flr);

static const struct attribute *xpcie_sysfs_attrs[] = {
	&dev_attr_swdev_id.attr,
	&dev_attr_ack_flr.attr,
	NULL,
};

static const struct attribute_group xpcie_epf_sysfs_attrs = {
	.attrs = (struct attribute **)xpcie_sysfs_attrs,
};

static int intel_xpcie_epf_bind(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *features;
	struct pci_epc *epc = epf->epc;
#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	u32 bus_num = 0, dev_num = 0;
#endif
	struct device *dev;
	size_t align = SZ_16K;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	if ((epf->func_no & 0x1))
		return 0;
#endif

	dev = epc->dev.parent;
	features = pci_epc_get_features(epc, epf->func_no);
	xpcie_epf->epc_features = features;
	if (features) {
		align = features->align;
		ret = intel_xpcie_configure_bar(epf, features);
		if (ret)
			return ret;
	}

	ret = intel_xpcie_epf_get_platform_data(dev, xpcie_epf);
	if (ret) {
		dev_err(&epf->dev, "Unable to get platform data\n");
		return -EINVAL;
	}

	ret = intel_xpcie_setup_bars(epf, align);
	if (ret) {
		dev_err(&epf->dev, "BAR initialization failed\n");
		return ret;
	}
#if (IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	intel_xpcie_enable_multi_functions(epf);
#else
	if (!strcmp(xpcie_epf->stepping, "A0")) {
		xpcie_epf->xpcie.legacy_a0 = true;
		intel_xpcie_iowrite32(1, xpcie_epf->xpcie.mmio +
					 XPCIE_MMIO_LEGACY_A0);
	} else {
		xpcie_epf->xpcie.legacy_a0 = false;
		intel_xpcie_iowrite32(0, xpcie_epf->xpcie.mmio +
					 XPCIE_MMIO_LEGACY_A0);
	}

	/* Enable interrupt */
	writel(LBC_CII_EVENT_FLAG,
	       xpcie_epf->apb_base + PCIE_REGS_PCIE_INTR_ENABLE);
	ret = devm_request_irq(&epf->dev, xpcie_epf->irq,
			       &intel_xpcie_host_interrupt, 0,
			       XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request irq\n");
		goto err_cleanup_bars;
	}

	ret = devm_request_irq(&epf->dev, xpcie_epf->irq_err,
			       &intel_xpcie_err_interrupt, 0,
			       XPCIE_DRIVER_NAME, &xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "failed to request error irq\n");
		goto err_cleanup_bars;
	}
#endif
	ret = intel_xpcie_ep_dma_init(epf);
	if (ret) {
		dev_err(&epf->dev, "DMA initialization failed\n");
		goto err_cleanup_bars;
	}

	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_READY);
#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	ret = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_SYS_CFG_CORE);
	bus_num = (ret >> PCIE_CFG_PBUS_NUM_OFFSET) & PCIE_CFG_PBUS_NUM_MASK;
	dev_num = (ret >> PCIE_CFG_PBUS_DEV_NUM_OFFSET) &
			PCIE_CFG_PBUS_DEV_NUM_MASK;
#endif

#if (!IS_ENABLED(CONFIG_ARCH_THUNDERBAY))
	xlink_sw_id = FIELD_PREP(XLINK_DEV_INF_TYPE_MASK,
				 XLINK_DEV_INF_PCIE) |
		      FIELD_PREP(XLINK_DEV_PHYS_ID_MASK,
				 bus_num << 8 | dev_num) |
		      FIELD_PREP(XLINK_DEV_TYPE_MASK, XLINK_DEV_TYPE_KMB) |
		      FIELD_PREP(XLINK_DEV_PCIE_ID_MASK, XLINK_DEV_PCIE_0) |
		      FIELD_PREP(XLINK_DEV_FUNC_MASK, XLINK_DEV_FUNC_VPU);
#endif

	ret = intel_xpcie_core_init(&xpcie_epf->xpcie);
	if (ret) {
		dev_err(&epf->dev, "Core component configuration failed\n");
		goto err_uninit_dma;
	}

	intel_xpcie_iowrite32(XPCIE_STATUS_UNINIT,
			      xpcie_epf->xpcie.mmio + XPCIE_MMIO_HOST_STATUS);
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_RUN);
	intel_xpcie_set_doorbell(&xpcie_epf->xpcie, FROM_DEVICE,
				 DEV_EVENT, NO_OP);
	memcpy(xpcie_epf->xpcie.io_comm + XPCIE_IO_COMM_MAGIC_OFF,
	       XPCIE_BOOT_MAGIC_YOCTO, strlen(XPCIE_BOOT_MAGIC_YOCTO));

	ret = sysfs_create_group(&epf->dev.kobj, &xpcie_epf_sysfs_attrs);
	if (ret) {
		dev_err(&epf->dev, "Failed to create sysfs entries\n");
		goto err_uninit_dma;
	}

	return 0;

err_uninit_dma:
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_ERROR);
	memcpy(xpcie_epf->xpcie.io_comm + XPCIE_IO_COMM_MAGIC_OFF,
	       XPCIE_BOOT_MAGIC_YOCTO, strlen(XPCIE_BOOT_MAGIC_YOCTO));

	intel_xpcie_ep_dma_uninit(epf);

err_cleanup_bars:
	intel_xpcie_cleanup_bars(epf);

	return ret;
}

static void intel_xpcie_epf_unbind(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	intel_xpcie_core_cleanup(&xpcie_epf->xpcie);
	intel_xpcie_set_device_status(&xpcie_epf->xpcie, XPCIE_STATUS_READY);

	sysfs_remove_group(&epf->dev.kobj, &xpcie_epf_sysfs_attrs);
	intel_xpcie_ep_dma_uninit(epf);

	pci_epc_stop(epc);

	intel_xpcie_cleanup_bars(epf);
}

static int intel_xpcie_epf_probe(struct pci_epf *epf)
{
	struct device *dev = &epf->dev;
	struct xpcie_epf *xpcie_epf;

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
	struct xpcie_epf *xpcie_epf;

	xpcie_epf = epf_get_drvdata(epf);

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
	int ret;

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
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION(XPCIE_DRIVER_DESC);
