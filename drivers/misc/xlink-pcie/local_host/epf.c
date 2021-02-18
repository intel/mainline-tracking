// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *
 * Intel XPCIe XLink PCIe Driver
 *
 * Copyright (C) 2020 Intel Corporation
 *
 ****************************************************************************/

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>

#include "epf.h"
#include "../common/boot.h"

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
#define BAR0_MIN_SIZE			SZ_4K
#endif

#define BAR2_MIN_SIZE			SZ_16K

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
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
#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
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

#if (!IS_ENABLED(CONFIG_PCIE_TBH_EP))
u32 xlink_sw_id;
#endif

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
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

int intel_xpcie_raise_irq(struct xpcie *xpcie, enum xpcie_doorbell_type type)
{
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	struct pci_epf *epf = xpcie_epf->epf;

	intel_xpcie_set_doorbell(xpcie, FROM_DEVICE, type, 1);

	return pci_epc_raise_irq(epf->epc, epf->func_no, PCI_EPC_IRQ_MSI, 1);
}

#if (!IS_ENABLED(CONFIG_PCIE_TBH_EP))
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

static irqreturn_t intel_xpcie_host_interrupt(int irq, void *args)
{
	struct xpcie *xpcie = args;
	struct xpcie_epf *xpcie_epf = container_of(xpcie,
						   struct xpcie_epf, xpcie);
	u8 event;
#if (!IS_ENABLED(CONFIG_PCIE_TBH_EP))
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
		return IRQ_HANDLED;
	}

	if (likely(xpcie_epf->core_irq_callback))
		xpcie_epf->core_irq_callback(irq, xpcie);

	return IRQ_HANDLED;
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
		pci_epf_free_space(epf, xpcie_epf->vaddr[barno], barno);
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

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
static int intel_xpcie_setup_bar(struct pci_epf *epf, enum pci_barno barno,
				 size_t size, size_t align)
{
	int ret;
	void __iomem *vaddr = NULL;
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct pci_epf_bar *bar = &epf->bar[barno];
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct thunderbay_pcie *thunderbay = to_thunderbay_pcie(pci);

	bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	bar->size = size;

	if (barno == 0) {
		bar->phys_addr =
			thunderbay->doorbell->start + (epf->func_no * 0x1000);
		vaddr = ioremap(bar->phys_addr, size);
	}

	if (barno == 2) {
		bar->phys_addr = thunderbay->mmr2[epf->func_no]->start;
		vaddr = ioremap(bar->phys_addr, size);
	}
	if (barno == 4) {
		bar->phys_addr = thunderbay->mmr4[epf->func_no]->start;
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

	thunderbay->setup_bar[epf->func_no] = true;
	ret = pci_epc_set_bar(epc, epf->func_no, bar);
	thunderbay->setup_bar[epf->func_no] = false;

	if (ret) {
		pci_epf_free_space(epf, vaddr, barno);
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
#endif

static int intel_xpcie_setup_bars(struct pci_epf *epf, size_t align)
{
	int ret;

	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
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

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
static void intel_xpcie_enable_multi_functions(struct pci_epf *epf)
{
	struct pci_epc *epc = epf->epc;
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct thunderbay_pcie *thunderbay = to_thunderbay_pcie(pci);
	size_t doorbell_clr_size = 0;
	unsigned long doorbell_clr_addr = 0;
	int ret = 0;

	xpcie_epf->irq_doorbell = thunderbay->irq_doorbell[epf->func_no];
	xpcie_epf->irq_rdma = thunderbay->irq_rdma[epf->func_no];
	xpcie_epf->irq_wdma = thunderbay->irq_wdma[epf->func_no];
	xpcie_epf->apb_base = thunderbay->slv_apb_base;

	switch (epf->func_no) {
	case 0:
		if (thunderbay->tbh_half) {
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pci->dev->of_node, 8);
		} else {
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pci->dev->of_node, 16);
			}
		if (ret) {
			dev_warn(&epc->dev,
				 "of_reserved_mem_device_init_by_idx(): %d\n",
				 ret);
		}
		epf->dev.dma_mask = pci->dev->dma_mask;
		epf->dev.coherent_dma_mask = pci->dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pci->dev->of_node, true);
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed of_dma_configure()ret=%d\n", ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed to set dma mask. ret=%d\n", ret);
		}
		doorbell_clr_addr = thunderbay->doorbell_clear->start;
		doorbell_clr_size = 4;
		break;
	case 1:
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x8;
		doorbell_clr_size = 4;
		break;
	case 2:
		if (thunderbay->tbh_half)
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pci->dev->of_node, 9);
		else
			ret = of_reserved_mem_device_init_by_idx
				(&epf->dev, pci->dev->of_node, 17);
		if (ret) {
			dev_warn(&epc->dev,
				 "of_reserved_mem_device_init_by_idx(): %d\n",
				 ret);
		}
		epf->dev.dma_mask = pci->dev->dma_mask;
		epf->dev.coherent_dma_mask = pci->dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pci->dev->of_node, true);
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed of_dma_configure()ret=%d\n", ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev,
				 "pcie:failed to set dma mask. ret=%d\n", ret);
		}
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x14;
		doorbell_clr_size = 4;
		break;
	case 3:
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x1C;
		doorbell_clr_size = 4;
		break;
	case 4:
		ret = of_reserved_mem_device_init_by_idx
			(&epf->dev, pci->dev->of_node, 18);
		if (ret) {
			dev_warn(&epc->dev,
				 "of_xx_mem_device_init_by_idx(): 18 ret=%d\n",
				 ret);
		}
		epf->dev.dma_mask = pci->dev->dma_mask;
		epf->dev.coherent_dma_mask = pci->dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pci->dev->of_node, true);
		if (ret) {
			dev_warn(&epc->dev, "failed of_dma_configure(): %d\n",
				 ret);
		}
		ret = dma_set_mask_and_coherent(&epf->dev, DMA_BIT_MASK(64));
		if (ret) {
			dev_warn(&epc->dev, "failed to set dma mask ret=%d\n",
				 ret);
		}
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x28;
		doorbell_clr_size = 4;
		break;
	case 5:
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x30;
		doorbell_clr_size = 4;
		break;
	case 6:
		ret = of_reserved_mem_device_init_by_idx
			(&epf->dev, pci->dev->of_node, 19);
		if (ret) {
			dev_warn(&epc->dev,
				 "of_xx_mem_device_init_by_idx(): 19 ret=%d\n",
				 ret);
		}
		epf->dev.dma_mask = pci->dev->dma_mask;
		epf->dev.coherent_dma_mask = pci->dev->coherent_dma_mask;
		ret = of_dma_configure(&epf->dev, pci->dev->of_node, true);
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
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x3C;
		doorbell_clr_size = 4;
		break;
	case 7:
		doorbell_clr_addr = thunderbay->doorbell_clear->start + 0x44;
		doorbell_clr_size = 4;
		break;
	}

	xpcie_epf->xpcie.doorbell_clear =
				ioremap(doorbell_clr_addr, doorbell_clr_size);
	dev_dbg(&epc->dev,
		"virtual doorbell_clear=%p,physical doorbell_clr_addr=%lx\n",
		 xpcie_epf->xpcie.doorbell_clear, doorbell_clr_addr);
	intel_xpcie_set_max_functions(&xpcie_epf->xpcie, epc->max_functions);
	list_add_tail(&xpcie_epf->list, &dev_list);
	snprintf(xpcie_epf->name, MXLK_MAX_NAME_LEN, "%s_func%x", epf->name,
		 epf->func_no);
	ret = request_irq(xpcie_epf->irq_doorbell,
			  &intel_xpcie_host_interrupt, 0, XPCIE_DRIVER_NAME,
			  &xpcie_epf->xpcie);
	if (ret)
		dev_err(&epf->dev, "failed to request irq\n");
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

static int intel_xpcie_epf_bind(struct pci_epf *epf)
{
	struct xpcie_epf *xpcie_epf = epf_get_drvdata(epf);
	const struct pci_epc_features *features;
	struct pci_epc *epc = epf->epc;
#if (!IS_ENABLED(CONFIG_PCIE_TBH_EP))
	u32 bus_num = 0, dev_num = 0;
#endif
	struct device *dev;
	size_t align = SZ_16K;
	int ret;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
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

	ret = intel_xpcie_setup_bars(epf, align);
	if (ret) {
		dev_err(&epf->dev, "BAR initialization failed\n");
		return ret;
	}
#if (IS_ENABLED(CONFIG_PCIE_TBH_EP))
	intel_xpcie_enable_multi_functions(epf);
#else
	ret = intel_xpcie_epf_get_platform_data(dev, xpcie_epf);
	if (ret) {
		dev_err(&epf->dev, "Unable to get platform data\n");
		return -EINVAL;
	}

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
			       &(intel_xpcie_err_interrupt), 0,
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
#if (!IS_ENABLED(CONFIG_PCIE_TBH_EP))
	ret = ioread32(xpcie_epf->apb_base + PCIE_REGS_PCIE_SYS_CFG_CORE);
	bus_num = (ret >> PCIE_CFG_PBUS_NUM_OFFSET) & PCIE_CFG_PBUS_NUM_MASK;
	dev_num = (ret >> PCIE_CFG_PBUS_DEV_NUM_OFFSET) &
			PCIE_CFG_PBUS_DEV_NUM_MASK;
#endif

#if (!IS_ENABLED(CONFIG_PCIE_TBH_EP))
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
