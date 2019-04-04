// SPDX-License-Identifier: GPL-2.0-only
/*
 * cnl-sst.c - DSP library functions for CNL platform
 *
 * Copyright (C) 2016-17, Intel Corporation.
 *
 * Author: Guneshwor Singh <guneshwor.o.singh@intel.com>
 *
 * Modified from:
 *	HDA DSP library functions for SKL platform
 *	Copyright (C) 2014-15, Intel Corporation.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <asm/set_memory.h>
#include <asm/cacheflush.h>
#include <sound/soc-acpi.h>

#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "../common/sst-ipc.h"
#include "cnl-sst-dsp.h"
#include "skl.h"
#include "skl-topology.h"

#define CNL_FW_ROM_INIT		0x1
#define CNL_FW_INIT		0x5
#define CNL_IPC_PURGE		0x01004000
#define CNL_INIT_TIMEOUT	300
#define CNL_BASEFW_TIMEOUT	3000

/* Firmware status window */
#define CNL_ADSP_FW_STATUS	BXT_ADSP_SRAM0_BASE
#define CNL_ADSP_ERROR_CODE	(CNL_ADSP_FW_STATUS + 0x4)

#define CNL_INSTANCE_ID		0
#define CNL_BASE_FW_MODULE_ID	0
#define CNL_ADSP_FW_HDR_OFFSET	0x2000
#define CNL_ROM_CTRL_DMA_ID	0x9

#ifdef CONFIG_X86_64
#define CNL_IMR_MEMSIZE					0x400000
#define CNL_IMR_PAGES	((CNL_IMR_MEMSIZE + PAGE_SIZE - 1) >> PAGE_SHIFT)
#define HDA_ADSP_REG_ADSPCS_IMR_CACHED_TLB_START	0x100
#define HDA_ADSP_REG_ADSPCS_IMR_UNCACHED_TLB_START	0x200
#define HDA_ADSP_REG_ADSPCS_IMR_SIZE			0x8

/* Needed for presilicon platform based on FPGA */
static int cnl_alloc_imr(struct sst_dsp *ctx)
{
	if (skl_alloc_dma_buf(ctx->dev, &ctx->imr_buf,
	     CNL_IMR_MEMSIZE) < 0) {
		dev_err(ctx->dev, "Alloc imr buffer failed\n");
		return -ENOMEM;
	}

	set_memory_uc((unsigned long)ctx->imr_buf.area, CNL_IMR_PAGES);
	writeq(virt_to_phys(ctx->imr_buf.area) + 1,
		 ctx->addr.shim + HDA_ADSP_REG_ADSPCS_IMR_CACHED_TLB_START);
	writeq(virt_to_phys(ctx->imr_buf.area) + 1,
		 ctx->addr.shim + HDA_ADSP_REG_ADSPCS_IMR_UNCACHED_TLB_START);

	writel(CNL_IMR_MEMSIZE, ctx->addr.shim
		+ HDA_ADSP_REG_ADSPCS_IMR_CACHED_TLB_START
		+ HDA_ADSP_REG_ADSPCS_IMR_SIZE);
	writel(CNL_IMR_MEMSIZE, ctx->addr.shim
		+ HDA_ADSP_REG_ADSPCS_IMR_UNCACHED_TLB_START
		+ HDA_ADSP_REG_ADSPCS_IMR_SIZE);

	memset(ctx->imr_buf.area, 0, CNL_IMR_MEMSIZE);

	return 0;
}

static inline void cnl_free_imr(struct sst_dsp *ctx)
{
	skl_free_dma_buf(ctx->dev, &ctx->imr_buf);
}
#endif

static int cnl_prepare_fw(struct sst_dsp *ctx, const void *fwdata, u32 fwsize)
{

	int ret, stream_tag;
#ifdef CONFIG_X86_64
	struct skl_dev *skl = get_skl_ctx(ctx->dev);
	struct skl_machine_pdata *pdata = (struct skl_machine_pdata *)
						skl->mach->pdata;
	if (pdata && pdata->imr_alloc && *(pdata->imr_alloc)) {
		ret = cnl_alloc_imr(ctx);
		if (ret < 0)
			return ret;
	}
#endif
	stream_tag = skl_dsp_prepare(ctx->dev, 0x40, fwsize, &ctx->dmab,
						SNDRV_PCM_STREAM_PLAYBACK);
	if (stream_tag <= 0) {
		dev_err(ctx->dev, "dma prepare failed: 0%#x\n", stream_tag);
		return stream_tag;
	}

	memcpy(ctx->dmab.area, fwdata, fwsize);

	clflush_cache_range(ctx->dmab.area, fwsize);
	/* purge FW request */
	sst_dsp_shim_write(ctx, CNL_ADSP_REG_HIPCIDR,
			   CNL_ADSP_REG_HIPCIDR_BUSY | (CNL_IPC_PURGE |
			   ((stream_tag - 1) << CNL_ROM_CTRL_DMA_ID)));

	ret = cnl_dsp_enable_core(ctx, SKL_DSP_CORE0_MASK);
	if (ret < 0) {
		dev_err(ctx->dev, "dsp boot core failed ret: %d\n", ret);
		ret = -EIO;
		goto base_fw_load_failed;
	}

	/* enable interrupt */
	cnl_ipc_int_enable(ctx);
	cnl_ipc_op_int_enable(ctx);

	ret = sst_dsp_register_poll(ctx, CNL_ADSP_FW_STATUS, CNL_FW_STS_MASK,
				    CNL_FW_ROM_INIT, CNL_INIT_TIMEOUT,
				    "rom load");
	if (ret < 0) {
		dev_err(ctx->dev, "rom init timeout, ret: %d\n", ret);
		goto base_fw_load_failed;
	}

	return stream_tag;

base_fw_load_failed:
	skl_dsp_cleanup(ctx->dev, &ctx->dmab, stream_tag,
						SNDRV_PCM_STREAM_PLAYBACK);
	cnl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
#ifdef CONFIG_X86_64
	if (pdata && pdata->imr_alloc && *(pdata->imr_alloc))
		cnl_free_imr(ctx);
#endif
	return ret;
}

static int sst_transfer_fw_host_dma(struct sst_dsp *ctx, int stream_tag)
{
	int ret;

	skl_dsp_trigger(ctx->dev, true, stream_tag,
						SNDRV_PCM_STREAM_PLAYBACK);
	ret = sst_dsp_register_poll(ctx, CNL_ADSP_FW_STATUS, CNL_FW_STS_MASK,
				    CNL_FW_INIT, CNL_BASEFW_TIMEOUT,
				    "firmware boot");

	skl_dsp_trigger(ctx->dev, false, stream_tag,
						SNDRV_PCM_STREAM_PLAYBACK);
	skl_dsp_cleanup(ctx->dev, &ctx->dmab, stream_tag,
						SNDRV_PCM_STREAM_PLAYBACK);

	return ret;
}

static int cnl_load_base_firmware(struct sst_dsp *ctx)
{
	struct firmware stripped_fw;
	struct skl_dev *cnl = ctx->thread_context;
	struct sst_pdata *pdata = ctx->pdata;
	int ret, i;

	if (!pdata->fw) {
		ret = request_firmware(&pdata->fw, ctx->fw_name, ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "request firmware failed: %d\n", ret);
			return ret;
		}
	}

	if (cnl->is_first_boot) {
		ret = snd_skl_parse_manifest(ctx, pdata->fw,
						CNL_ADSP_FW_HDR_OFFSET, 0);
		if (ret < 0)
			goto load_base_firmware_failed;
	}

	stripped_fw.data = pdata->fw->data;
	stripped_fw.size = pdata->fw->size;
	skl_dsp_strip_extended_manifest(&stripped_fw);

	ret = -ENOEXEC;
	for (i = 0; i < SST_FW_INIT_RETRY && ret < 0; i++) {
		ret = cnl_prepare_fw(ctx, stripped_fw.data, stripped_fw.size);
		if (ret < 0) {
			dev_dbg(ctx->dev, "prepare firmware failed: %d\n", ret);
			continue;
		}

		dev_dbg(ctx->dev, "ROM loaded successfully on iteration %d.\n", i);

		ret = sst_transfer_fw_host_dma(ctx, ret);
		if (ret < 0) {
			dev_dbg(ctx->dev, "transfer firmware failed: %d\n", ret);
			cnl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
		}
	}

	if (ret < 0)
		goto load_base_firmware_failed;
	dev_dbg(ctx->dev, "Firmware download successful.\n");

	ret = wait_event_timeout(cnl->boot_wait, cnl->boot_complete,
				 msecs_to_jiffies(SKL_IPC_BOOT_MSECS));
	if (ret == 0) {
		dev_err(ctx->dev, "FW ready timed-out\n");
		cnl_dsp_disable_core(ctx, SKL_DSP_CORE0_MASK);
		ret = -EIO;
		goto load_base_firmware_failed;
	}

	cnl->fw_loaded = true;

	return 0;

load_base_firmware_failed:
	dev_err(ctx->dev, "Firmware load failed: %d.\n", ret);
	release_firmware(pdata->fw);
	pdata->fw = NULL;

	return ret;
}

static int cnl_set_dsp_D0(struct sst_dsp *ctx, unsigned int core_id)
{
	struct skl_dev *cnl = ctx->thread_context;
	unsigned int core_mask = SKL_DSP_CORE_MASK(core_id);
	struct skl_ipc_dxstate_info dx;
	int ret;

	if (!cnl->fw_loaded) {
		cnl->boot_complete = false;
		ret = cnl_load_base_firmware(ctx);
		if (ret < 0) {
			dev_err(ctx->dev, "fw reload failed: %d\n", ret);
			return ret;
		}

		if (cnl->lib_count > 1) {
			ret = ctx->fw_ops.load_library(ctx, cnl->lib_info,
						cnl->lib_count);
			if (ret < 0) {
				dev_err(ctx->dev,
					"reload libs failed: %d\n", ret);
				return ret;
			}
		}

		cnl->cores.state[core_id] = SKL_DSP_RUNNING;
		return ret;
	}

	ret = cnl_dsp_enable_core(ctx, core_mask);
	if (ret < 0) {
		dev_err(ctx->dev, "enable dsp core %d failed: %d\n",
			core_id, ret);
		goto err;
	}

	if (core_id == SKL_DSP_CORE0_ID) {
		/* enable interrupt */
		cnl_ipc_int_enable(ctx);
		cnl_ipc_op_int_enable(ctx);
		cnl->boot_complete = false;

		ret = wait_event_timeout(cnl->boot_wait, cnl->boot_complete,
					 msecs_to_jiffies(SKL_IPC_BOOT_MSECS));
		if (ret == 0) {
			dev_err(ctx->dev,
				"dsp boot timeout, status=%#x error=%#x\n",
				sst_dsp_shim_read(ctx, CNL_ADSP_FW_STATUS),
				sst_dsp_shim_read(ctx, CNL_ADSP_ERROR_CODE));
			goto err;
		}
	} else {
		dx.core_mask = core_mask;
		dx.dx_mask = core_mask;

		ret = skl_ipc_set_dx(&cnl->ipc, CNL_INSTANCE_ID,
				     CNL_BASE_FW_MODULE_ID, &dx);
		if (ret < 0) {
			dev_err(ctx->dev, "set_dx failed, core: %d ret: %d\n",
				core_id, ret);
			goto err;
		}
	}
	cnl->cores.state[core_id] = SKL_DSP_RUNNING;

	return 0;
err:
	cnl_dsp_disable_core(ctx, core_mask);

	return ret;
}

static int cnl_set_dsp_D3(struct sst_dsp *ctx, unsigned int core_id)
{
	struct skl_dev *cnl = ctx->thread_context;
	unsigned int core_mask = SKL_DSP_CORE_MASK(core_id);
	struct skl_ipc_dxstate_info dx;
	int ret;

	dx.core_mask = core_mask;
	dx.dx_mask = SKL_IPC_D3_MASK;

	ret = skl_ipc_set_dx(&cnl->ipc, CNL_INSTANCE_ID,
			     CNL_BASE_FW_MODULE_ID, &dx);
	if (ret < 0) {
		dev_err(ctx->dev,
			"dsp core %d to d3 failed; continue reset\n",
			core_id);
		cnl->fw_loaded = false;
	}

	/* disable interrupts if core 0 */
	if (core_id == SKL_DSP_CORE0_ID) {
		cnl_ipc_op_int_disable(ctx);
		skl_ipc_int_disable(ctx);
	}

	ret = cnl_dsp_disable_core(ctx, core_mask);
	if (ret < 0) {
		dev_err(ctx->dev, "disable dsp core %d failed: %d\n",
			core_id, ret);
		return ret;
	}

	cnl->cores.state[core_id] = SKL_DSP_RESET;

	return ret;
}

static unsigned int cnl_get_errno(struct sst_dsp *ctx)
{
	return sst_dsp_shim_read(ctx, CNL_ADSP_ERROR_CODE);
}

static const struct skl_dsp_fw_ops cnl_fw_ops = {
	.set_state_D0 = cnl_set_dsp_D0,
	.set_state_D3 = cnl_set_dsp_D3,
	.set_state_D0i3 = bxt_schedule_dsp_D0i3,
	.set_state_D0i0 = bxt_set_dsp_D0i0,
	.load_fw = cnl_load_base_firmware,
	.get_fw_errcode = cnl_get_errno,
	.load_library = bxt_load_library,
	.enable_logs = bxt_enable_logs,
	.log_buffer_offset = skl_log_buffer_offset,
	.log_buffer_status = bxt_log_buffer_status,
};

#define CNL_IPC_GLB_NOTIFY_RSP_SHIFT	29
#define CNL_IPC_GLB_NOTIFY_RSP_MASK	0x1
#define CNL_IPC_GLB_NOTIFY_RSP_TYPE(x)	(((x) >> CNL_IPC_GLB_NOTIFY_RSP_SHIFT) \
					& CNL_IPC_GLB_NOTIFY_RSP_MASK)

static irqreturn_t cnl_dsp_irq_thread_handler(int irq, void *context)
{
	struct sst_dsp *dsp = context;
	struct skl_dev *cnl = sst_dsp_get_thread_context(dsp);
	struct sst_generic_ipc *ipc = &cnl->ipc;
	struct skl_ipc_header header = {0};
	u32 hipcida, hipctdr, hipctdd;
	int ipc_irq = 0;

	/* here we handle ipc interrupts only */
	if (!(dsp->intr_status & CNL_ADSPIS_IPC))
		return IRQ_NONE;

	hipcida = sst_dsp_shim_read_unlocked(dsp, CNL_ADSP_REG_HIPCIDA);
	hipctdr = sst_dsp_shim_read_unlocked(dsp, CNL_ADSP_REG_HIPCTDR);
	hipctdd = sst_dsp_shim_read_unlocked(dsp, CNL_ADSP_REG_HIPCTDD);

	/* reply message from dsp */
	if (hipcida & CNL_ADSP_REG_HIPCIDA_DONE) {
		sst_dsp_shim_update_bits(dsp, CNL_ADSP_REG_HIPCCTL,
			CNL_ADSP_REG_HIPCCTL_DONE, 0);

		/* clear done bit - tell dsp operation is complete */
		sst_dsp_shim_update_bits_forced(dsp, CNL_ADSP_REG_HIPCIDA,
			CNL_ADSP_REG_HIPCIDA_DONE, CNL_ADSP_REG_HIPCIDA_DONE);

		ipc_irq = 1;

		/* unmask done interrupt */
		sst_dsp_shim_update_bits(dsp, CNL_ADSP_REG_HIPCCTL,
			CNL_ADSP_REG_HIPCCTL_DONE, CNL_ADSP_REG_HIPCCTL_DONE);
	}

	/* new message from dsp */
	if (hipctdr & CNL_ADSP_REG_HIPCTDR_BUSY) {
		header.primary = hipctdr;
		header.extension = hipctdd;
		dev_dbg(dsp->dev, "IPC irq: Firmware respond primary:%x",
						header.primary);
		dev_dbg(dsp->dev, "IPC irq: Firmware respond extension:%x",
						header.extension);

		if (CNL_IPC_GLB_NOTIFY_RSP_TYPE(header.primary)) {
			/* Handle Immediate reply from DSP Core */
			skl_ipc_process_reply(ipc, header);
		} else {
			dev_dbg(dsp->dev, "IPC irq: Notification from firmware\n");
			skl_ipc_process_notification(ipc, header);
		}
		/* clear busy interrupt */
		sst_dsp_shim_update_bits_forced(dsp, CNL_ADSP_REG_HIPCTDR,
			CNL_ADSP_REG_HIPCTDR_BUSY, CNL_ADSP_REG_HIPCTDR_BUSY);

		/* set done bit to ack dsp */
		sst_dsp_shim_update_bits_forced(dsp, CNL_ADSP_REG_HIPCTDA,
			CNL_ADSP_REG_HIPCTDA_DONE, CNL_ADSP_REG_HIPCTDA_DONE);
		ipc_irq = 1;
	}

	if (ipc_irq == 0)
		return IRQ_NONE;

	cnl_ipc_int_enable(dsp);

	/* continue to send any remaining messages */
	schedule_work(&ipc->kwork);

	return IRQ_HANDLED;
}

static void cnl_ipc_tx_msg(struct sst_generic_ipc *ipc, struct ipc_message *msg)
{
	struct skl_ipc_header *header = (struct skl_ipc_header *)(&msg->tx.header);

	if (msg->tx.size)
		sst_dsp_outbox_write(ipc->dsp, msg->tx.data, msg->tx.size);
	sst_dsp_shim_write_unlocked(ipc->dsp, CNL_ADSP_REG_HIPCIDD,
				    header->extension);
	sst_dsp_shim_write_unlocked(ipc->dsp, CNL_ADSP_REG_HIPCIDR,
				header->primary | CNL_ADSP_REG_HIPCIDR_BUSY);
}

static bool cnl_ipc_is_dsp_busy(struct sst_dsp *dsp)
{
	u32 hipcidr;

	hipcidr = sst_dsp_shim_read_unlocked(dsp, CNL_ADSP_REG_HIPCIDR);

	return (hipcidr & CNL_ADSP_REG_HIPCIDR_BUSY);
}

static int cnl_ipc_init(struct device *dev, struct skl_dev *cnl)
{
	struct sst_generic_ipc *ipc;
	int err;

	ipc = &cnl->ipc;
	ipc->dsp = cnl->dsp;
	ipc->dev = dev;

	ipc->tx_data_max_size = SKL_MAILBOX_SIZE;
	ipc->rx_data_max_size = SKL_MAILBOX_SIZE;

	err = sst_ipc_init(ipc);
	if (err)
		return err;

	/*
	 * overriding tx_msg and is_dsp_busy since
	 * ipc registers are different for cnl
	 */
	ipc->ops.tx_msg = cnl_ipc_tx_msg;
	ipc->ops.tx_data_copy = skl_ipc_tx_data_copy;
	ipc->ops.is_dsp_busy = cnl_ipc_is_dsp_busy;

	return 0;
}

static int cnl_sst_init(struct sst_dsp *sst, struct sst_pdata *pdata)
{
	struct skl_dev *cnl = sst->thread_context;
	void __iomem *mmio;
	int ret;

	cnl->dsp = sst;
	sst->fw_ops = cnl_fw_ops;
	mmio = pci_ioremap_bar(cnl->pci, 4);
	if (!mmio)
		return -ENXIO;
	sst->addr.lpe = mmio;
	sst->addr.shim = mmio;
	sst->addr.sram0 = (mmio + BXT_ADSP_SRAM0_BASE);
	sst->addr.sram2 = (mmio + BXT_ADSP_SRAM2_BASE);

	sst_dsp_mailbox_init(sst,
		(BXT_ADSP_SRAM0_BASE + SKL_FW_REGS_SIZE), SKL_MAILBOX_SIZE,
		BXT_ADSP_SRAM1_BASE, SKL_MAILBOX_SIZE);

	ret = cnl_ipc_init(cnl->dev, cnl);
	if (ret) {
		skl_dsp_free(sst);
		return ret;
	}

	/* set the D0i3 check */
	cnl->ipc.ops.check_dsp_lp_on = skl_ipc_check_D0i0;
	cnl->boot_complete = false;
	init_waitqueue_head(&cnl->boot_wait);
	INIT_DELAYED_WORK(&cnl->d0i3.work, bxt_set_dsp_D0i3);
	cnl->d0i3.state = SKL_DSP_D0I3_NONE;

	return 0;
}

struct sst_ops cnl_sst_ops = {
	.irq_handler = cnl_dsp_sst_interrupt,
	.thread_fn = cnl_dsp_irq_thread_handler,
	.write = sst_shim32_write,
	.read = sst_shim32_read,
	.ram_read = sst_memcpy_fromio_32,
	.ram_write = sst_memcpy_toio_32,
	.init = cnl_sst_init,
	.free = cnl_dsp_free,
};

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Cannonlake IPC driver");
