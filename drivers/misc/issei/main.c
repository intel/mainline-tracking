// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023-2025 Intel Corporation */
#include <linux/dev_printk.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include "issei_dev.h"
#include "fw_client.h"
#include "host_client.h"
#include "cdev.h"
#include "ham.h"

static void issei_rst_state_set(struct issei_device *idev, enum issei_rst_state state)
{
	idev->rst_state = state;
	/* wake up the thread */
	if (waitqueue_active(&idev->wait_rst_state))
		wake_up(&idev->wait_rst_state);
}

static int issei_reset(struct issei_device *idev)
{
	int ret;

	idev->ops->irq_clear(idev);

	issei_cl_all_disconnect(idev);
	issei_fw_cl_remove_all(idev);
	/* No need to check overflow here, the counter is used only for info */
	idev->all_reset_count++;
	ret = idev->ops->hw_reset(idev, !idev->power_down);
	issei_dmam_clean(idev);
	if (ret) {
		dev_err(idev->dev, "hw_reset failed ret = %d\n", ret);
		return ret;
	}

	if (idev->power_down) {
		dev_dbg(idev->dev, "powering down: end of reset\n");
		issei_rst_state_set(idev, ISSEI_RST_STATE_DISABLED);
		return -ENODEV;
	}
	return 0;
}

static int issei_process_read_msg(struct issei_device *idev)
{
	struct issei_dma_data data = {0};
	int ret;

	ret = issei_dma_read(idev, &data);
	if (ret)
		return ret;

	dev_dbg(idev->dev, "Processing response %u %u %u %u\n", data.fw_id, data.host_id,
		data.status, data.length);
	if (issei_is_ham_rsp(data.fw_id, data.host_id)) {
		ret = issei_ham_process_ham_rsp(idev, data.status, data.buf, data.length);
		kfree(data.buf);
	} else {
		dev_dbg(idev->dev, "Client data\n");
		ret = issei_cl_read_buf(idev, data.host_id, data.buf, data.length);
		if (ret)
			kfree(data.buf);
	}

	idev->ops->irq_write_generate(idev);
	return ret;
}

static int issei_process_write_msg(struct issei_device *idev)
{
	if (idev->rst_state != ISSEI_RST_STATE_DONE)
		return 0;

	return issei_cl_write_from_queue(idev);
}

static int issei_process_messages(struct issei_device *idev)
{
	issei_process_read_msg(idev);
	return issei_process_write_msg(idev);
}

static int issei_process_thread(void *_dev)
{
	long timeout, old_timeout = MAX_SCHEDULE_TIMEOUT;
	struct issei_device *idev = _dev;
	int ret;

	while (!kthread_should_stop()) {
		dev_dbg(idev->dev, "process_work in %d\n", idev->rst_state);
		if (!idev->ops->hw_is_ready(idev) && idev->rst_state > ISSEI_RST_STATE_HW_READY) {
			if (!idev->power_down)
				dev_dbg(idev->dev, "HW not ready, resetting\n");
			idev->rst_state = ISSEI_RST_STATE_INIT;
		}
		if (idev->power_down)
			idev->rst_state = ISSEI_RST_STATE_INIT;
		atomic_set(&idev->rst_irq, 0);
		dev_dbg(idev->dev, "reset_step in %d\n", idev->rst_state);
		timeout = MAX_SCHEDULE_TIMEOUT;
		ret = 0;
		switch (idev->rst_state) {
		case ISSEI_RST_STATE_DISABLED:
			if (idev->power_down) {
				dev_dbg(idev->dev, "Interrupt in power down?\n");
				break;
			}
			idev->rst_state = ISSEI_RST_STATE_INIT;
			fallthrough;

		case ISSEI_RST_STATE_INIT:
			idev->ops->irq_clear(idev);
			idev->ops->irq_sync(idev);

			if (!idev->power_down) {
				idev->reset_count++;
				if (idev->reset_count > ISSEI_MAX_CONSEC_RESET) {
					dev_err(idev->dev, "reset: reached maximal consecutive resets: disabling the device\n");
					issei_rst_state_set(idev, ISSEI_RST_STATE_DISABLED);
					break;
				}
			}

			ret = issei_reset(idev);
			if (idev->power_down) {
				dev_dbg(idev->dev, "Powering down\n");
				return 0;
			}
			if (!ret) {
				idev->rst_state = ISSEI_RST_STATE_HW_READY;
				timeout = msecs_to_jiffies(ISSEI_RST_HW_READY_TIMEOUT_MSEC);
			}
			break;

		case ISSEI_RST_STATE_HW_READY:
			if (idev->ops->hw_is_ready(idev)) {
				dev_dbg(idev->dev, "HW is ready\n");
				idev->ops->hw_reset_release(idev);
				idev->ops->host_set_ready(idev);
				ret = idev->ops->setup_message_send(idev);
				if (!ret) {
					idev->rst_state = ISSEI_RST_STATE_SETUP;
					timeout = msecs_to_jiffies(ISSEI_RST_STEP_TIMEOUT_MSEC);
				}
			} else {
				dev_dbg(idev->dev, "HW is not ready?\n");
				timeout = old_timeout;
			}
			break;

		case ISSEI_RST_STATE_SETUP:
			ret = idev->ops->setup_message_recv(idev);
			if (!ret) {
				timeout = msecs_to_jiffies(ISSEI_RST_STEP_TIMEOUT_MSEC);
				ret = issei_ham_send_start_req(idev);
				idev->rst_state = ISSEI_RST_STATE_START;
			} else if (ret == -ENODATA) {
				ret = 0;
				timeout = old_timeout;
			}
			break;

		case ISSEI_RST_STATE_START:
			ret = issei_process_read_msg(idev);
			if (!ret) {
				timeout = msecs_to_jiffies(ISSEI_RST_STEP_TIMEOUT_MSEC);
				idev->rst_state = ISSEI_RST_STATE_CLIENT_ENUM;
			} else if (ret == -ENODATA) {
				ret = 0;
				timeout = old_timeout;
			}
			break;

		case ISSEI_RST_STATE_CLIENT_ENUM:
			ret = issei_process_read_msg(idev);
			if (!ret) {
				idev->reset_count = 0;
				idev->rst_state = ISSEI_RST_STATE_DONE;
				dev_dbg(idev->dev, "Reset finished successfully\n");
			} else if (ret == -ENODATA) {
				ret = 0;
				timeout = old_timeout;
			}
			break;

		case ISSEI_RST_STATE_DONE:
			ret = issei_process_messages(idev);
			break;
		}

		if (ret) {
			dev_dbg(idev->dev, "Process failed ret = %d\n", ret);
			idev->rst_state = ISSEI_RST_STATE_INIT;
			continue;
		}
		old_timeout = wait_event_interruptible_timeout(idev->wait_rst_irq,
							       atomic_read(&idev->rst_irq),
							       timeout);
		dev_dbg(idev->dev, "Out of wait %d %d %ld\n", idev->rst_state,
			atomic_read(&idev->rst_irq), old_timeout);

		if (idev->rst_state == ISSEI_RST_STATE_DISABLED)
			continue;

		if (!atomic_read(&idev->rst_irq)) {
			dev_dbg(idev->dev, "Timed out at state %d, resetting\n", idev->rst_state);
			idev->rst_state = ISSEI_RST_STATE_INIT;
		}
	}

	return 0;
}

/**
 * issei_device_init - initialize issei device structure.
 * @idev: the device structure
 * @dev: parent device structure
 * @dma_length: structure with DMA sizes
 * @ops: hardware-related operations
 */
void issei_device_init(struct issei_device *idev, struct device *dev,
		       const struct issei_dma_length *dma_length,
		       const struct issei_hw_ops *ops)
{
	idev->dev = dev;
	idev->power_down = false;
	init_waitqueue_head(&idev->wait_rst_irq);
	atomic_set(&idev->rst_irq, 0);
	init_waitqueue_head(&idev->wait_rst_state);
	idev->rst_state = ISSEI_RST_STATE_INIT;

	mutex_init(&idev->host_client_lock);
	INIT_LIST_HEAD(&idev->host_client_list);
	idev->host_client_last_id = 0;
	idev->host_client_count = 0;

	mutex_init(&idev->fw_client_lock);
	INIT_LIST_HEAD(&idev->fw_client_list);

	idev->dma.length = *dma_length;

	INIT_LIST_HEAD(&idev->write_queue);

	idev->ops = ops;
}
EXPORT_SYMBOL_GPL(issei_device_init);

/**
 * issei_start - configure HW device and start processing thread.
 * @idev: the device structure
 *
 * Return: 0 on success, < 0 on failure
 */
int issei_start(struct issei_device *idev)
{
	int ret;

	idev->power_down = false;

	ret = issei_dmam_setup(idev);
	if (ret)
		return ret;

	idev->ops->irq_clear(idev);

	ret = idev->ops->hw_config(idev);
	if (ret)
		return ret;

	idev->reset_thread = kthread_run(issei_process_thread, idev,
					"kisseiprocess/%s", dev_name(idev->dev));
	if (IS_ERR(idev->reset_thread)) {
		dev_err(idev->dev, "unable to create process thread. ret = %d\n", ret);
		return PTR_ERR(idev->reset_thread);
	}

	atomic_set(&idev->rst_irq, 1);
	wake_up_interruptible(&idev->wait_rst_irq);
	return 0;
}
EXPORT_SYMBOL_GPL(issei_start);

/**
 * issei_stop - stop interrupts and processing thread.
 * @idev: the device structure
 */
void issei_stop(struct issei_device *idev)
{
	idev->power_down = true;

	idev->ops->irq_clear(idev);
	idev->ops->irq_sync(idev);

	atomic_set(&idev->rst_irq, 1);
	wake_up_interruptible(&idev->wait_rst_irq);

	wait_event_timeout(idev->wait_rst_state,
			   (idev->rst_state == ISSEI_RST_STATE_DISABLED),
			   ISSEI_STOP_TIMEOUT_MSEC);
	kthread_stop(idev->reset_thread);
}
EXPORT_SYMBOL_GPL(issei_stop);

/**
 * issei_is_busy - check if driver is busy with writes or reset flow.
 * @idev: the device structure
 *
 * Return: true if busy, false otherwise
 */
bool issei_is_busy(struct issei_device *idev)
{
	guard(mutex)(&idev->host_client_lock);

	return (idev->rst_state != ISSEI_RST_STATE_DONE) || !list_empty(&idev->write_queue);
}
EXPORT_SYMBOL_GPL(issei_is_busy);
