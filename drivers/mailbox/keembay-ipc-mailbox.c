// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Keem Bay IPC mailbox driver.
 *
 * Copyright (c) 2020-2021 Intel Corporation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

/*
 * The IPC FIFO registers (offsets to the base address defined in device tree).
 */

/*
 * TIM_IPC_FIFO - Write a 32-bit entry to FIFO.
 *
 * The entry to be put in the FIFO must be written to this register.
 *
 * NOTE: the 6 least-significant bits are reserved for the writing processor
 * to include its processor ID, 0 <= x <= 62, so it can determine if the entry
 * was written correctly by checking the appropriate bit of register
 * TIM_IPC_FIFO_OF_FLAG[n].
 *
 * Internally, the hardware increments FIFO write pointer and fill level.
 *
 */
#define IPC_FIFO		0x00

/* The last 6 bits of an IPC entry are reserved. */
#define IPC_FIFO_ENTRY_RSVD_MASK	0x3f

/*
 * IPC_FIFO_ATM - Read from FIFO using ATM mode.
 *
 * If FIFO is empty, reading from this registers returns 0xFFFFFFFF, otherwise
 * returns the value from the FIFO with the 6 least-significant bits set to 0.
 *
 * Internally, the hardware increments FIFO read pointer and decrements fill
 * level.
 */
#define IPC_FIFO_ATM		0x04
#define IPC_FIFO_EMPTY		0xFFFFFFFF

/*
 * TIM_IPC_FIFO_OF_FLAG[n] - IPC FIFO overflow status for processor IDs 0-62.
 *
 * Read:
 *
 * A processor can check that its writes to the IPC FIFO were successful by
 * reading the value of TIM_IPC_FIFO_OF_FLAG0 or TIM_IPC_FIFO_OF_FLAG1
 * (depending on its processor ID).
 *
 * Bit x, 0 <= x <= 31, of TIM_IPC_FIFO_OF_FLAG0 is set high if a write
 * to TIM_IPC_FIFO by processor ID x failed because the FIFO was full.
 *
 * Bit x, 0 <= x <= 30, of TIM_IPC_FIFO_OF_FLAG1 is set high if a write
 * to TIM_IPC_FIFO by processor ID x+32 failed because the FIFO was
 * full.
 *
 * Processors are identified by the 6 least-significant bits of words
 * written to TIM_IPC_FIFO, i.e. x = TIM_IPC_FIFO[5:0].
 * Processor ID = 0x3F is reserved to indicate a read of an empty FIFO
 * has occurred.
 *
 * Write:
 *
 * Writing 1 to bit position x of TIM_IPC_FIFO_OF_FLAG0 clears the
 * overflow flag corresponding to processor ID x.  Writing 1 to bit
 * position x of TIM_IPC_FIFO_OF_FLAG1 clears the overflow flag
 * corresponding to processor ID x+32.
 *
 * Writing 0 to any bit position has not effect.
 */
#define IPC_FIFO_OF_FLAG0	0x10
#define IPC_FIFO_OF_FLAG1	0x14

/* The processor ID of the CPU. */
#define IPC_FIFO_ID_CPU		0

/**
 * struct keembay_ipc_mbox - Intel Keem Bay IPC mailbox controller.
 * @mbox:		Mailbox controller.
 * @mbox_chan:		The only channel supported by this controller.
 * @dev:		The device associated with this controller.
 * @cpu_fifo_base:	Base address of CPU FIFO registers.
 * @vpu_fifo_base:	Base address of VPU FIFO registers.
 * @txdone_tasklet:	Tasklet calling mbox_chan_txdone(). It's activated by
 *			the send_data() function, after the VPU FIFO has been
 *			written; a tasklet is used because send_data() cannot
 *			call mbox_chan_txdone() directly.
 * @txdone_result:	The result of the last TX. It's set by the send_data()
 *			function before activating the txdone_tasklet.
 */
struct keembay_ipc_mbox {
	struct mbox_controller	mbox;
	struct mbox_chan	mbox_chan;
	void __iomem		*cpu_fifo_base;
	void __iomem		*vpu_fifo_base;
	struct tasklet_struct	txdone_tasklet;
	int			txdone_result;
};

/* The IRQ handler servicing 'FIFO-not-empty' IRQs coming from the CPU FIFO. */
static irqreturn_t keembay_ipc_mailbox_irq_handler(int irq, void *data)
{
	struct keembay_ipc_mbox *kmb_ipc_mbox = data;
	u32 entry;

	/* Extract and process one entry from CPU FIFO. */
	entry = ioread32(kmb_ipc_mbox->cpu_fifo_base + IPC_FIFO_ATM);
	if (unlikely(entry == IPC_FIFO_EMPTY))
		return IRQ_NONE;

	/* Notify mailbox client of new data. */
	mbox_chan_received_data(&kmb_ipc_mbox->mbox_chan, (void *)&entry);

	return IRQ_HANDLED;
}

/*
 * The function implementing the txdone_tasklet.
 *
 * It calls mbox_chan_txdone() passing as arguments the only channel we have
 * and the result of the last TX (as stored in the keembay_ipc_mbox struct).
 */
static void txdone_tasklet_func(unsigned long kmb_ipc_mbox_ptr)
{
	struct keembay_ipc_mbox *kmb_ipc_mbox = (void *)kmb_ipc_mbox_ptr;

	/* Notify client that tx is completed and pass proper result code. */
	mbox_chan_txdone(&kmb_ipc_mbox->mbox_chan, kmb_ipc_mbox->txdone_result);
}

/*
 * Mailbox controller 'send_data()' function.
 *
 * This functions tries to put 'data' into the VPU FIFO. This is done by
 * writing to the IPC_FIFO VPU register and then checking if we overflew the
 * FIFO (by reading the IPC_FIFO_OF_FLAG0 VPU register).
 *
 * If we overflew the FIFO, the TX has failed and we notify the mailbox client
 * by passing -EBUSY to mbox_chan_txdone()); otherwise the TX succeeded (we
 * pass 0 to mbox_chan_txdone()). Note: mbox_chan_txdone() cannot be called
 * directly (since that would case a deadlock), therefore a tasklet is used to
 * defer the call.
 *
 * 'data' is meant to be a 32-bit unsigned integer with the least 6 significant
 * bits set to 0.
 */
static int keembay_ipc_mailbox_send_data(struct mbox_chan *chan, void *data)
{
	struct keembay_ipc_mbox *kmb_ipc_mbox = chan->con_priv;
	u32 entry, overflow;

	entry = *((u32 *)data);

	/* Ensure last 6-bits of entry are not used. */
	if (unlikely(entry & IPC_FIFO_ENTRY_RSVD_MASK)) {
		kmb_ipc_mbox->txdone_result = -EINVAL;
		goto exit;
	}

	/* Add processor ID to entry. */
	entry |= IPC_FIFO_ID_CPU & IPC_FIFO_ENTRY_RSVD_MASK;

	/* Write entry to VPU FIFO. */
	iowrite32(entry, kmb_ipc_mbox->vpu_fifo_base + IPC_FIFO);

	/* Check if we overflew the VPU FIFO. */
	overflow = ioread32(kmb_ipc_mbox->vpu_fifo_base + IPC_FIFO_OF_FLAG0) &
		   BIT(IPC_FIFO_ID_CPU);
	if (unlikely(overflow)) {
		/* Reset overflow register. */
		iowrite32(BIT(IPC_FIFO_ID_CPU),
			  kmb_ipc_mbox->vpu_fifo_base + IPC_FIFO_OF_FLAG0);
		kmb_ipc_mbox->txdone_result = -EBUSY;
		goto exit;
	}
	kmb_ipc_mbox->txdone_result = 0;

exit:
	/* Schedule tasklet to call mbox_chan_txdone(). */
	tasklet_schedule(&kmb_ipc_mbox->txdone_tasklet);

	return 0;
}

/* The mailbox channel ops for this controller. */
static const struct mbox_chan_ops keembay_ipc_mbox_chan_ops = {
	.send_data = keembay_ipc_mailbox_send_data,
};

static int keembay_ipc_mailbox_probe(struct platform_device *pdev)
{
	struct keembay_ipc_mbox *kmb_ipc_mbox;
	struct device *dev = &pdev->dev;
	void __iomem *base;
	int irq;
	int rc;

	kmb_ipc_mbox = devm_kzalloc(dev, sizeof(*kmb_ipc_mbox), GFP_KERNEL);
	if (!kmb_ipc_mbox)
		return -ENOMEM;

	/* Map CPU FIFO registers. */
	base = devm_platform_ioremap_resource_byname(pdev, "cpu_fifo");
	if (IS_ERR(base)) {
		dev_err(dev, "Failed to ioremap CPU FIFO registers\n");
		return PTR_ERR(base);
	}
	kmb_ipc_mbox->cpu_fifo_base = base;

	/* MAP VPU FIFO registers. */
	base = devm_platform_ioremap_resource_byname(pdev, "vpu_fifo");
	if (IS_ERR(base)) {
		dev_err(dev, "Failed to ioremap VPU FIFO registers\n");
		return PTR_ERR(base);
	}
	kmb_ipc_mbox->vpu_fifo_base = base;

	/* Initialize mailbox channels. */
	kmb_ipc_mbox->mbox_chan.con_priv = kmb_ipc_mbox;

	/* Initialize mailbox controller. */
	kmb_ipc_mbox->mbox.dev = dev;
	kmb_ipc_mbox->mbox.ops = &keembay_ipc_mbox_chan_ops;
	kmb_ipc_mbox->mbox.chans = &kmb_ipc_mbox->mbox_chan;
	kmb_ipc_mbox->mbox.num_chans = 1;
	/*
	 * Set txdone_irq; we don't have a HW IRQ, but we use a txdone tasklet
	 * to simulate it.
	 */
	kmb_ipc_mbox->mbox.txdone_irq = true;

	/* Init TX done tasklet. */
	tasklet_init(&kmb_ipc_mbox->txdone_tasklet, txdone_tasklet_func,
		     (uintptr_t)kmb_ipc_mbox);

	rc = devm_mbox_controller_register(dev, &kmb_ipc_mbox->mbox);
	if (rc) {
		dev_err(&pdev->dev,
			"Failed to register Keem Bay IPC controller\n");
		return rc;
	}

	/* Register interrupt handler for CPU FIFO. */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	rc = devm_request_irq(dev, irq, keembay_ipc_mailbox_irq_handler, 0,
			      dev_name(dev), kmb_ipc_mbox);
	if (rc)
		return rc;

	platform_set_drvdata(pdev, kmb_ipc_mbox);

	return 0;
}

static int keembay_ipc_mailbox_remove(struct platform_device *pdev)
{
	struct keembay_ipc_mbox *kmb_ipc_mbox = platform_get_drvdata(pdev);

	/*
	 * Just kill the tasklet as iomem and irq have been requested with
	 * devm_* functions and, therefore, are freed automatically.
	 */
	tasklet_kill(&kmb_ipc_mbox->txdone_tasklet);

	return 0;
}

static const struct of_device_id keembay_ipc_mailbox_of_match[] = {
	{
		.compatible = "intel,keembay-ipc-mailbox",
	},
	{}
};

static struct platform_driver keembay_ipc_mailbox_driver = {
	.driver = {
			.name = "keembay-ipc-mailbox",
			.of_match_table = keembay_ipc_mailbox_of_match,
		},
	.probe = keembay_ipc_mailbox_probe,
	.remove = keembay_ipc_mailbox_remove,
};
module_platform_driver(keembay_ipc_mailbox_driver);

MODULE_DESCRIPTION("Intel Keem Bay IPC mailbox driver");
MODULE_AUTHOR("Daniele Alessandrelli <daniele.alessandrelli@intel.com>");
MODULE_LICENSE("GPL");
