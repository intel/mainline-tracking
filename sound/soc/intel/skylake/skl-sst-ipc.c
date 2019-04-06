// SPDX-License-Identifier: GPL-2.0-only
/*
 * skl-sst-ipc.c - Intel skl IPC Support
 *
 * Copyright (C) 2014-15, Intel Corporation.
 */
#include <linux/device.h>
#include <linux/kfifo.h>
#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl.h"
#include "skl-sst-dsp.h"
#include "skl-sst-ipc.h"
#include "skl-topology.h"
#include "sound/hdaudio_ext.h"
#include "skl-topology.h"

/* Global Message - Generic */
#define IPC_GLB_TYPE_SHIFT		24
#define IPC_GLB_TYPE_MASK		(0xf << IPC_GLB_TYPE_SHIFT)
#define IPC_GLB_TYPE(x)			((x) << IPC_GLB_TYPE_SHIFT)

/* Global Message - Reply */
#define IPC_GLB_REPLY_STATUS_SHIFT	24
#define IPC_GLB_REPLY_STATUS_MASK	((0x1 << IPC_GLB_REPLY_STATUS_SHIFT) - 1)
#define IPC_GLB_REPLY_STATUS(x)		((x) << IPC_GLB_REPLY_STATUS_SHIFT)

#define IPC_GLB_REPLY_TYPE_SHIFT	29
#define IPC_GLB_REPLY_TYPE_MASK		0x1F
#define IPC_GLB_REPLY_TYPE(x)		(((x) >> IPC_GLB_REPLY_TYPE_SHIFT) \
					& IPC_GLB_RPLY_TYPE_MASK)

#define IPC_TIMEOUT_MSECS		3000

#define IPC_EMPTY_LIST_SIZE		8

#define IPC_MSG_TARGET_SHIFT		30
#define IPC_MSG_TARGET_MASK		0x1
#define IPC_MSG_TARGET(x)		(((x) & IPC_MSG_TARGET_MASK) \
					<< IPC_MSG_TARGET_SHIFT)

#define IPC_MSG_DIR_SHIFT		29
#define IPC_MSG_DIR_MASK		0x1
#define IPC_MSG_DIR(x)			(((x) & IPC_MSG_DIR_MASK) \
					<< IPC_MSG_DIR_SHIFT)
/* Global Notification Message */
#define IPC_GLB_NOTIFY_TYPE_SHIFT	16
#define IPC_GLB_NOTIFY_TYPE_MASK	0xFF
#define IPC_GLB_NOTIFY_TYPE(x)		(((x) >> IPC_GLB_NOTIFY_TYPE_SHIFT) \
					& IPC_GLB_NOTIFY_TYPE_MASK)

#define IPC_GLB_NOTIFY_MSG_TYPE_SHIFT	24
#define IPC_GLB_NOTIFY_MSG_TYPE_MASK	0x1F
#define IPC_GLB_NOTIFY_MSG_TYPE(x)	(((x) >> IPC_GLB_NOTIFY_MSG_TYPE_SHIFT)	\
						& IPC_GLB_NOTIFY_MSG_TYPE_MASK)

#define IPC_GLB_NOTIFY_RSP_SHIFT	29
#define IPC_GLB_NOTIFY_RSP_MASK		0x1
#define IPC_GLB_NOTIFY_RSP_TYPE(x)	(((x) >> IPC_GLB_NOTIFY_RSP_SHIFT) \
					& IPC_GLB_NOTIFY_RSP_MASK)

/* Pipeline operations */

/* Create pipeline message */
#define IPC_PPL_MEM_SIZE_SHIFT		0
#define IPC_PPL_MEM_SIZE_MASK		0x7FF
#define IPC_PPL_MEM_SIZE(x)		(((x) & IPC_PPL_MEM_SIZE_MASK) \
					<< IPC_PPL_MEM_SIZE_SHIFT)

#define IPC_PPL_TYPE_SHIFT		11
#define IPC_PPL_TYPE_MASK		0x1F
#define IPC_PPL_TYPE(x)			(((x) & IPC_PPL_TYPE_MASK) \
					<< IPC_PPL_TYPE_SHIFT)

#define IPC_INSTANCE_ID_SHIFT		16
#define IPC_INSTANCE_ID_MASK		0xFF
#define IPC_INSTANCE_ID(x)		(((x) & IPC_INSTANCE_ID_MASK) \
					<< IPC_INSTANCE_ID_SHIFT)

#define IPC_PPL_LP_MODE_SHIFT           0
#define IPC_PPL_LP_MODE_MASK            0x1
#define IPC_PPL_LP_MODE(x)              (((x) & IPC_PPL_LP_MODE_MASK) \
					<< IPC_PPL_LP_MODE_SHIFT)

/* Set pipeline state message */
#define IPC_PPL_STATE_SHIFT		0
#define IPC_PPL_STATE_MASK		0x1F
#define IPC_PPL_STATE(x)		(((x) & IPC_PPL_STATE_MASK) \
					<< IPC_PPL_STATE_SHIFT)

/* Module operations primary register */
#define IPC_MOD_ID_SHIFT		0
#define IPC_MOD_ID_MASK		0xFFFF
#define IPC_MOD_ID(x)		(((x) & IPC_MOD_ID_MASK) \
					<< IPC_MOD_ID_SHIFT)

#define IPC_MOD_INSTANCE_ID_SHIFT	16
#define IPC_MOD_INSTANCE_ID_MASK	0xFF
#define IPC_MOD_INSTANCE_ID(x)	(((x) & IPC_MOD_INSTANCE_ID_MASK) \
					<< IPC_MOD_INSTANCE_ID_SHIFT)

/* Init instance message extension register */
#define IPC_PARAM_BLOCK_SIZE_SHIFT	0
#define IPC_PARAM_BLOCK_SIZE_MASK	0xFFFF
#define IPC_PARAM_BLOCK_SIZE(x)		(((x) & IPC_PARAM_BLOCK_SIZE_MASK) \
					<< IPC_PARAM_BLOCK_SIZE_SHIFT)

#define IPC_PPL_INSTANCE_ID_SHIFT	16
#define IPC_PPL_INSTANCE_ID_MASK	0xFF
#define IPC_PPL_INSTANCE_ID(x)		(((x) & IPC_PPL_INSTANCE_ID_MASK) \
					<< IPC_PPL_INSTANCE_ID_SHIFT)

#define IPC_CORE_ID_SHIFT		24
#define IPC_CORE_ID_MASK		0x1F
#define IPC_CORE_ID(x)			(((x) & IPC_CORE_ID_MASK) \
					<< IPC_CORE_ID_SHIFT)

#define IPC_DOMAIN_SHIFT                28
#define IPC_DOMAIN_MASK                 0x1
#define IPC_DOMAIN(x)                   (((x) & IPC_DOMAIN_MASK) \
					<< IPC_DOMAIN_SHIFT)

/* Bind/Unbind message extension register */
#define IPC_DST_MOD_ID_SHIFT		0
#define IPC_DST_MOD_ID(x)		(((x) & IPC_MOD_ID_MASK) \
					<< IPC_DST_MOD_ID_SHIFT)

#define IPC_DST_MOD_INSTANCE_ID_SHIFT 16
#define IPC_DST_MOD_INSTANCE_ID(x)	(((x) & IPC_MOD_INSTANCE_ID_MASK) \
					<< IPC_DST_MOD_INSTANCE_ID_SHIFT)

#define IPC_DST_QUEUE_SHIFT		24
#define IPC_DST_QUEUE_MASK		0x7
#define IPC_DST_QUEUE(x)		(((x) & IPC_DST_QUEUE_MASK) \
					<< IPC_DST_QUEUE_SHIFT)

#define IPC_SRC_QUEUE_SHIFT		27
#define IPC_SRC_QUEUE_MASK		0x7
#define IPC_SRC_QUEUE(x)		(((x) & IPC_SRC_QUEUE_MASK) \
					<< IPC_SRC_QUEUE_SHIFT)
/* Load Module count */
#define IPC_LOAD_MODULE_SHIFT		0
#define IPC_LOAD_MODULE_MASK		0xFF
#define IPC_LOAD_MODULE_CNT(x)		(((x) & IPC_LOAD_MODULE_MASK) \
					<< IPC_LOAD_MODULE_SHIFT)

/* Save pipeline messgae extension register */
#define IPC_DMA_ID_SHIFT		0
#define IPC_DMA_ID_MASK			0x1F
#define IPC_DMA_ID(x)			(((x) & IPC_DMA_ID_MASK) \
					<< IPC_DMA_ID_SHIFT)
/* Large Config message extension register */
#define IPC_DATA_OFFSET_SZ_SHIFT	0
#define IPC_DATA_OFFSET_SZ_MASK		0xFFFFF
#define IPC_DATA_OFFSET_SZ(x)		(((x) & IPC_DATA_OFFSET_SZ_MASK) \
					<< IPC_DATA_OFFSET_SZ_SHIFT)
#define IPC_DATA_OFFSET_SZ_CLEAR	~(IPC_DATA_OFFSET_SZ_MASK \
					  << IPC_DATA_OFFSET_SZ_SHIFT)

#define IPC_LARGE_PARAM_ID_SHIFT	20
#define IPC_LARGE_PARAM_ID_MASK		0xFF
#define IPC_LARGE_PARAM_ID(x)		(((x) & IPC_LARGE_PARAM_ID_MASK) \
					<< IPC_LARGE_PARAM_ID_SHIFT)

#define IPC_FINAL_BLOCK_SHIFT		28
#define IPC_FINAL_BLOCK_MASK		0x1
#define IPC_FINAL_BLOCK(x)		(((x) & IPC_FINAL_BLOCK_MASK) \
					<< IPC_FINAL_BLOCK_SHIFT)

#define IPC_INITIAL_BLOCK_SHIFT		29
#define IPC_INITIAL_BLOCK_MASK		0x1
#define IPC_INITIAL_BLOCK(x)		(((x) & IPC_INITIAL_BLOCK_MASK) \
					<< IPC_INITIAL_BLOCK_SHIFT)
#define IPC_INITIAL_BLOCK_CLEAR		~(IPC_INITIAL_BLOCK_MASK \
					  << IPC_INITIAL_BLOCK_SHIFT)
/* Set D0ix IPC extension register */
#define IPC_D0IX_WAKE_SHIFT		0
#define IPC_D0IX_WAKE_MASK		0x1
#define IPC_D0IX_WAKE(x)		(((x) & IPC_D0IX_WAKE_MASK) \
					<< IPC_D0IX_WAKE_SHIFT)

#define IPC_D0IX_STREAMING_SHIFT	1
#define IPC_D0IX_STREAMING_MASK		0x1
#define IPC_D0IX_STREAMING(x)		(((x) & IPC_D0IX_STREAMING_MASK) \
					<< IPC_D0IX_STREAMING_SHIFT)

/* Offset to get the event data for module notification */
#define MOD_DATA_OFFSET		12
#define SET_LARGE_CFG_FW_CONFIG		7

#define SKL_FW_RSRCE_EVNT_DATA_SZ	6

struct skl_event_timestamp_notify {
	u32 module_instance_id;
	u32 node_id;
	struct skl_event_timestamp ts;
} __packed;

struct skl_event_notify {
	u32 resource_type;
	u32 resource_id;
	u32 event_type;
	u32 reserved;
	u32 event_data[SKL_FW_RSRCE_EVNT_DATA_SZ];
} __packed;

void skl_ipc_tx_data_copy(struct ipc_message *msg, char *tx_data,
		size_t tx_size)
{
	if (tx_size)
		memcpy(msg->tx.data, tx_data, tx_size);
}

static bool skl_ipc_is_dsp_busy(struct sst_dsp *dsp)
{
	u32 hipci;

	hipci = sst_dsp_shim_read_unlocked(dsp, SKL_ADSP_REG_HIPCI);
	return (hipci & SKL_ADSP_REG_HIPCI_BUSY);
}

static void skl_ipc_tx_msgs_direct(struct sst_generic_ipc *ipc)
{
        struct ipc_message *msg;
        unsigned long flags;

        spin_lock_irqsave(&ipc->dsp->spinlock, flags);

        if (list_empty(&ipc->tx_list) || ipc->pending) {
                spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
                return;
        }

        /* if the DSP is busy, we will TX messages after IRQ.
         * also postpone if we are in the middle of procesing completion irq*/
        if (ipc->ops.is_dsp_busy && ipc->ops.is_dsp_busy(ipc->dsp)) {
                dev_dbg(ipc->dev, "skl_ipc_tx_msgs_direct dsp busy\n");
                spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
                return;
        }

        msg = list_first_entry(&ipc->tx_list, struct ipc_message, list);
        list_move(&msg->list, &ipc->rx_list);

        dev_dbg(ipc->dev, "skl_ipc_tx_msgs_direct sending message, header - %#.16lx\n",
                                (unsigned long)msg->tx.header);
        print_hex_dump_debug("Params:", DUMP_PREFIX_OFFSET, 8, 4,
                             msg->tx.data, msg->tx.size, false);
        if (ipc->ops.tx_msg != NULL)
                ipc->ops.tx_msg(ipc, msg);

        spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
}

/* Lock to be held by caller */
static void skl_ipc_tx_msg(struct sst_generic_ipc *ipc, struct ipc_message *msg)
{
	struct skl_ipc_header *header = (struct skl_ipc_header *)(&msg->tx.header);

	if (msg->tx.size)
		sst_dsp_outbox_write(ipc->dsp, msg->tx.data, msg->tx.size);
	sst_dsp_shim_write_unlocked(ipc->dsp, SKL_ADSP_REG_HIPCIE,
						header->extension);
	sst_dsp_shim_write_unlocked(ipc->dsp, SKL_ADSP_REG_HIPCI,
		header->primary | SKL_ADSP_REG_HIPCI_BUSY);
}

int skl_ipc_check_D0i0(struct sst_dsp *dsp, bool state)
{
	int ret;

	/* check D0i3 support */
	if (!dsp->fw_ops.set_state_D0i0)
		return 0;

	/* Attempt D0i0 or D0i3 based on state */
	if (state)
		ret = dsp->fw_ops.set_state_D0i0(dsp);
	else
		ret = dsp->fw_ops.set_state_D0i3(dsp);

	return ret;
}

static struct ipc_message *skl_ipc_reply_get_msg(struct sst_generic_ipc *ipc,
				u64 ipc_header)
{
	struct ipc_message *msg =  NULL;
	struct skl_ipc_header *header = (struct skl_ipc_header *)(&ipc_header);

	if (list_empty(&ipc->rx_list)) {
		dev_err(ipc->dev, "ipc: rx list is empty but received 0x%x\n",
			header->primary);
		goto out;
	}

	msg = list_first_entry(&ipc->rx_list, struct ipc_message, list);

	list_del(&msg->list);
out:
	return msg;

}

static
int skl_process_timestamp_notification(struct skl_dev *skl)
{
	struct skl_module_cfg *mconfig;
	struct skl_event_timestamp_notify ts_notif;
	struct skl_pipeline *ppl;
	struct skl_pipe_module *m;
	u32 instance_id;
	int copier_id = skl_get_module_id(skl, &skl_copier_mod_uuid);
	int ret = -ENXIO;

	sst_dsp_inbox_read(skl->dsp, &ts_notif, sizeof(ts_notif));
	instance_id = ts_notif.module_instance_id & IPC_MOD_INSTANCE_ID_MASK;
	dev_dbg(skl->dev, "%s copier instance:%d\n", __func__, instance_id);

	list_for_each_entry(ppl, &skl->ppl_list, node)
		list_for_each_entry(m, &ppl->pipe->w_list, node) {
			mconfig = m->w->priv;
			if ((mconfig->id.module_id == copier_id) &&
			    (mconfig->id.pvt_id == instance_id)) {
				mconfig->ts = ts_notif.ts;
				complete(&mconfig->ts_completion);
				ret = 0;
				break;
			}
		}

	return ret;
}

static int skl_process_module_notification(struct skl_dev *skl)
{
	struct skl_notify_data *notify_data;
	struct skl_module_notify mod_notif;
	u32 notify_data_sz;
	char *module_data;

	dev_dbg(skl->dev, "***** Module Notification ******\n");
	/* read module notification structure from mailbox */
	sst_dsp_inbox_read(skl->dsp, &mod_notif,
				sizeof(struct skl_module_notify));

	notify_data_sz = sizeof(mod_notif) + mod_notif.event_data_size;
	notify_data = kzalloc((sizeof(*notify_data) + notify_data_sz),
							GFP_KERNEL);

	if (!notify_data)
		return -ENOMEM;

	/* read the complete notification message */
	sst_dsp_inbox_read(skl->dsp, notify_data->data, notify_data_sz);

	notify_data->length = notify_data_sz;
	notify_data->type = 0xFF;

	/* Module notification data to console */
	dev_dbg(skl->dev, "Module Id    = %#x\n",
					(mod_notif.unique_id >> 16));
	dev_dbg(skl->dev, "Instanse Id  = %#x\n",
					(mod_notif.unique_id & 0x0000FFFF));
	dev_dbg(skl->dev, "Data Size    = %d bytes\n",
					mod_notif.event_data_size);

	module_data = notify_data->data;

	print_hex_dump(KERN_DEBUG, "DATA: ", MOD_DATA_OFFSET, 8, 4,
				module_data, notify_data->length, false);

	skl->notify_ops.notify_cb(skl, IPC_GLB_MODULE_NOTIFICATION,
							notify_data);
	kfree(notify_data);

	return 0;
}

static void
skl_parse_resource_event(struct skl_dev *skl, struct skl_ipc_header header)
{
	struct skl_event_notify notify;
	struct sst_dsp *sst = skl->dsp;

	/* read the message contents from mailbox */
	sst_dsp_inbox_read(sst, &notify, sizeof(struct skl_event_notify));

	/* notify user about the event type */
	switch (notify.event_type) {

	case SKL_BUDGET_VIOLATION:
		dev_err(sst->dev, "MCPS Budget Violation: %x\n",
					header.primary);
		break;
	case SKL_MIXER_UNDERRUN:
		dev_err(sst->dev, "Mixer Underrun Detected: %x\n",
					header.primary);
		break;
	case SKL_STREAM_DATA_SEGMENT:
		dev_err(sst->dev, "Stream Data Segment: %x\n",
					header.primary);
		break;
	case SKL_PROCESS_DATA_ERR:
		dev_err(sst->dev, "Process Data Error: %x\n",
					header.primary);
		break;
	case SKL_STACK_OVERFLOW:
		dev_err(sst->dev, "Stack Overflow: %x\n",
					header.primary);
		break;
	case SKL_BUFFERING_MODE_CHANGED:
		dev_err(sst->dev, "Buffering Mode Changed: %x\n",
					header.primary);
		break;
	case SKL_GATEWAY_UNDERRUN:
		dev_err(sst->dev, "Gateway Underrun Detected: %x\n",
					header.primary);
		break;
	case SKL_GATEWAY_OVERRUN:
		dev_err(sst->dev, "Gateway Overrun Detected: %x\n",
					header.primary);
		break;
	case SKL_WCLK_SAMPLE_COUNT:
		dev_err(sst->dev,
			"FW Wclk and Sample count Notif Detected: %x\n",
					header.primary);
		break;
	case SKL_GATEWAY_HIGH_THRESHOLD:
		dev_err(sst->dev, "IPC gateway reached high threshold: %x\n",
					header.primary);
		break;
	case SKL_GATEWAY_LOW_THRESHOLD:
		dev_err(sst->dev, "IPC gateway reached low threshold: %x\n",
					header.primary);
		break;
	case SKL_I2S_BCE_DETECTED:
		dev_err(sst->dev, "Bit Count Error detected on I2S port: %x\n",
					header.primary);
		break;
	case SKL_I2S_CLK_STATE_CHANGED:
		dev_err(sst->dev, "Clock detected/loss on I2S port: %x\n",
					header.primary);
		break;
	case SKL_I2S_SINK_MODE_CHANGED:
		dev_err(sst->dev, "I2S Sink started/stopped dropping \
			data in non-blk mode: %x\n", header.primary);
		break;
	case SKL_I2S_SOURCE_MODE_CHANGED:
		dev_err(sst->dev, "I2S Source started/stopped generating 0's \
			in non-blk mode: %x\n", header.primary);
		break;
	case SKL_SRE_DRIFT_TOO_HIGH:
		dev_err(sst->dev,
			"Frequency drift exceeded limit in SRE: %x\n",
					header.primary);
		break;
	case SKL_INVALID_RESOURCE_EVENT_TYPE:
		dev_err(sst->dev, "Invalid type: %x\n", header.primary);
		break;
	default:
		dev_err(sst->dev, "ipc: Unhandled resource event=%x",
					header.primary);
		break;
	}

	print_hex_dump(KERN_INFO, "Params:",
			DUMP_PREFIX_OFFSET, 8, 4,
			&notify, sizeof(struct skl_event_notify), false);
}

int skl_ipc_process_notification(struct sst_generic_ipc *ipc,
		struct skl_ipc_header header)
{
	struct skl_dev *skl = container_of(ipc, struct skl_dev, ipc);
	int ret;

	if (IPC_GLB_NOTIFY_MSG_TYPE(header.primary)) {
		switch (IPC_GLB_NOTIFY_TYPE(header.primary)) {

		case IPC_GLB_NOTIFY_UNDERRUN:
			dev_err(ipc->dev, "FW Underrun %x\n", header.primary);
			break;

		case IPC_GLB_NOTIFY_RESOURCE_EVENT:
			skl_parse_resource_event(skl, header);
			break;

		case IPC_GLB_NOTIFY_FW_READY:
			skl->boot_complete = true;
			wake_up(&skl->boot_wait);
			break;

		case IPC_GLB_NOTIFY_PHRASE_DETECTED:
			dev_dbg(ipc->dev, "***** Phrase Detected **********\n");

			/*
			 * Per HW recomendation, After phrase detection,
			 * clear the CGCTL.MISCBDCGE.
			 *
			 * This will be set back on stream closure
			 */
			skl->enable_miscbdcge(ipc->dev, false);
			skl->miscbdcg_disabled = true;
			break;

		case IPC_GLB_NOTIFY_TIMESTAMP_CAPTURED:
			return skl_process_timestamp_notification(skl);

		case IPC_GLB_MODULE_NOTIFICATION:
			ret = skl_process_module_notification(skl);
			if (ret < 0) {
				dev_err(ipc->dev,
				"Module Notification read fail:%d\n", ret);
				return ret;
			}
			break;

		default:
			dev_err(ipc->dev, "ipc: Unhandled error msg=%x\n",
						header.primary);
			break;
		}
	}

	return 0;
}

struct skl_ipc_err_map {
	enum skl_ipc_glb_reply reply;
	const char *msg;
	int err;
};

static struct skl_ipc_err_map skl_err_map[] = {
	{IPC_GLB_REPLY_ERROR_INVALID_PARAM,
		"DSP invalid parameter", EINVAL},
	{IPC_GLB_REPLY_UNKNOWN_MSG_TYPE,
		"DSP unknown message ID", EINVAL},
	{IPC_GLB_REPLY_OUT_OF_MEMORY,
		"DSP out of memory", ENOMEM},
	{IPC_GLB_REPLY_BUSY,
		"DSP busy", EBUSY},
	{IPC_GLB_REPLY_PENDING,
		"DSP reply pending", EBUSY},
	{IPC_GLB_REPLY_FAILURE,
		"DSP unknown error", EFAULT},
	{IPC_GLB_REPLY_INVALID_REQUEST,
		"DSP unsupported operation", EINVAL},
	{IPC_GLB_REPLY_INVALID_RESOURCE_ID,
		"DSP resource not found", EINVAL},
	{IPC_GLB_REPLY_OUT_OF_MIPS,
		"DSP no MCPS to complete request", ENOMEM},
	{IPC_GLB_REPLY_INVALID_RESOURCE_STATE,
		"DSP resource in invalid state", EINVAL},
	{IPC_GLB_REPLY_UNAVAILABLE,
		"DSP requested service/data is unavailable", EINVAL},
};

static int skl_ipc_set_reply_error_code(struct sst_generic_ipc *ipc, u32 reply)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(skl_err_map); i++) {
		if (skl_err_map[i].reply == reply)
			break;
	}

	if (i == ARRAY_SIZE(skl_err_map)) {
		dev_err(ipc->dev, "ipc FW reply: %d, FW Error Code: %u\n",
				reply,
				ipc->dsp->fw_ops.get_fw_errcode(ipc->dsp));
		return -EINVAL;
	}

	if (skl_err_map[i].err < 0)
		dev_err(ipc->dev, "ipc FW reply: %s, FW Error Code: %u\n",
				skl_err_map[i].msg,
				ipc->dsp->fw_ops.get_fw_errcode(ipc->dsp));
	else
		dev_info(ipc->dev, "ipc FW reply: %s, FW Error Code: %u\n",
				skl_err_map[i].msg,
				ipc->dsp->fw_ops.get_fw_errcode(ipc->dsp));

	return -(skl_err_map[i].err);
}

void skl_ipc_process_reply(struct sst_generic_ipc *ipc,
		struct skl_ipc_header header)
{
	struct ipc_message *msg;
	u32 reply = header.primary & IPC_GLB_REPLY_STATUS_MASK;
	u64 *ipc_header = (u64 *)(&header);
	struct skl_dev *skl = container_of(ipc, struct skl_dev, ipc);
	unsigned long flags;

	spin_lock_irqsave(&ipc->dsp->spinlock, flags);
	msg = skl_ipc_reply_get_msg(ipc, *ipc_header);
	spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
	if (msg == NULL) {
		dev_dbg(ipc->dev, "ipc: rx list is empty\n");
		return;
	}

	msg->rx.header = *ipc_header;
	/* first process the header */
	if (reply == IPC_GLB_REPLY_SUCCESS) {
		dev_dbg(ipc->dev, "ipc FW reply %x: success\n", header.primary);
		/* copy the rx data from the mailbox */
		sst_dsp_inbox_read(ipc->dsp, msg->rx.data, msg->rx.size);
		switch (IPC_GLB_NOTIFY_MSG_TYPE(header.primary)) {
		case IPC_GLB_LOAD_MULTIPLE_MODS:
		case IPC_GLB_LOAD_LIBRARY:
			skl->mod_load_complete = true;
			skl->mod_load_status = true;
			wake_up(&skl->mod_load_wait);
			break;

		default:
			break;

		}
	} else {
		msg->errno = skl_ipc_set_reply_error_code(ipc, reply);
		switch (IPC_GLB_NOTIFY_MSG_TYPE(header.primary)) {
		case IPC_GLB_LOAD_MULTIPLE_MODS:
		case IPC_GLB_LOAD_LIBRARY:
			skl->mod_load_complete = true;
			skl->mod_load_status = false;
			wake_up(&skl->mod_load_wait);
			break;

		default:
			break;

		}
	}

	spin_lock_irqsave(&ipc->dsp->spinlock, flags);
	sst_ipc_tx_msg_reply_complete(ipc, msg);
	spin_unlock_irqrestore(&ipc->dsp->spinlock, flags);
}

irqreturn_t skl_dsp_irq_thread_handler(int irq, void *context)
{
	struct sst_dsp *dsp = context;
	struct skl_dev *skl = sst_dsp_get_thread_context(dsp);
	struct sst_generic_ipc *ipc = &skl->ipc;
	struct skl_ipc_header header = {0};
	u32 hipcie, hipct, hipcte;
	int ipc_irq = 0;

	if (dsp->intr_status & SKL_ADSPIS_CL_DMA)
		skl_cldma_process_intr(dsp);

	/* Here we handle IPC interrupts only */
	if (!(dsp->intr_status & SKL_ADSPIS_IPC))
		return IRQ_NONE;

	hipcie = sst_dsp_shim_read_unlocked(dsp, SKL_ADSP_REG_HIPCIE);
	hipct = sst_dsp_shim_read_unlocked(dsp, SKL_ADSP_REG_HIPCT);
	hipcte = sst_dsp_shim_read_unlocked(dsp, SKL_ADSP_REG_HIPCTE);

	/* reply message from DSP */
	if (hipcie & SKL_ADSP_REG_HIPCIE_DONE) {
		sst_dsp_shim_update_bits(dsp, SKL_ADSP_REG_HIPCCTL,
			SKL_ADSP_REG_HIPCCTL_DONE, 0);

		/* clear DONE bit - tell DSP we have completed the operation */
		sst_dsp_shim_update_bits_forced(dsp, SKL_ADSP_REG_HIPCIE,
			SKL_ADSP_REG_HIPCIE_DONE, SKL_ADSP_REG_HIPCIE_DONE);

		ipc_irq = 1;

		/* unmask Done interrupt */
		sst_dsp_shim_update_bits(dsp, SKL_ADSP_REG_HIPCCTL,
			SKL_ADSP_REG_HIPCCTL_DONE, SKL_ADSP_REG_HIPCCTL_DONE);
	}

	/* New message from DSP */
	if (hipct & SKL_ADSP_REG_HIPCT_BUSY) {
		header.primary = hipct;
		header.extension = hipcte;
		dev_dbg(dsp->dev, "IPC irq: Firmware respond primary:%x\n",
						header.primary);
		dev_dbg(dsp->dev, "IPC irq: Firmware respond extension:%x\n",
						header.extension);

		if (IPC_GLB_NOTIFY_RSP_TYPE(header.primary)) {
			/* Handle Immediate reply from DSP Core */
			skl_ipc_process_reply(ipc, header);
		} else {
			dev_dbg(dsp->dev, "IPC irq: Notification from firmware\n");
			skl_ipc_process_notification(ipc, header);
		}
		/* clear  busy interrupt */
		sst_dsp_shim_update_bits_forced(dsp, SKL_ADSP_REG_HIPCT,
			SKL_ADSP_REG_HIPCT_BUSY, SKL_ADSP_REG_HIPCT_BUSY);
		ipc_irq = 1;
	}

	if (ipc_irq == 0)
		return IRQ_NONE;

	skl_ipc_int_enable(dsp);

	/* continue to send any remaining messages... */
	schedule_work(&ipc->kwork);

	return IRQ_HANDLED;
}

void skl_ipc_int_enable(struct sst_dsp *ctx)
{
	sst_dsp_shim_update_bits(ctx, SKL_ADSP_REG_ADSPIC,
			SKL_ADSPIC_IPC, SKL_ADSPIC_IPC);
}

void skl_ipc_int_disable(struct sst_dsp *ctx)
{
	sst_dsp_shim_update_bits_unlocked(ctx, SKL_ADSP_REG_ADSPIC,
			SKL_ADSPIC_IPC, 0);
}

void skl_ipc_op_int_enable(struct sst_dsp *ctx)
{
	/* enable IPC DONE interrupt */
	sst_dsp_shim_update_bits(ctx, SKL_ADSP_REG_HIPCCTL,
		SKL_ADSP_REG_HIPCCTL_DONE, SKL_ADSP_REG_HIPCCTL_DONE);

	/* Enable IPC BUSY interrupt */
	sst_dsp_shim_update_bits(ctx, SKL_ADSP_REG_HIPCCTL,
		SKL_ADSP_REG_HIPCCTL_BUSY, SKL_ADSP_REG_HIPCCTL_BUSY);
}

void skl_ipc_op_int_disable(struct sst_dsp *ctx)
{
	/* disable IPC DONE interrupt */
	sst_dsp_shim_update_bits(ctx, SKL_ADSP_REG_HIPCCTL,
					SKL_ADSP_REG_HIPCCTL_DONE, 0);

	/* Disable IPC BUSY interrupt */
	sst_dsp_shim_update_bits(ctx, SKL_ADSP_REG_HIPCCTL,
					SKL_ADSP_REG_HIPCCTL_BUSY, 0);

}

bool skl_ipc_int_status(struct sst_dsp *ctx)
{
	return sst_dsp_shim_read_unlocked(ctx,
			SKL_ADSP_REG_ADSPIS) & SKL_ADSPIS_IPC;
}

int skl_ipc_init(struct device *dev, struct skl_dev *skl)
{
	struct sst_generic_ipc *ipc;
	int err;

	ipc = &skl->ipc;
	ipc->dsp = skl->dsp;
	ipc->dev = dev;

	ipc->tx_data_max_size = SKL_MAILBOX_SIZE;
	ipc->rx_data_max_size = SKL_MAILBOX_SIZE;

	err = sst_ipc_init(ipc);
	if (err)
		return err;

	ipc->ops.tx_msg = skl_ipc_tx_msg;
	ipc->ops.tx_data_copy = skl_ipc_tx_data_copy;
	ipc->ops.direct_tx_msg = skl_ipc_tx_msgs_direct;
	ipc->ops.is_dsp_busy = skl_ipc_is_dsp_busy;

	return 0;
}

int skl_ipc_create_pipeline(struct sst_generic_ipc *ipc,
		u16 ppl_mem_size, u8 ppl_type, u8 instance_id, u8 lp_mode)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_CREATE_PPL);
	header.primary |= IPC_INSTANCE_ID(instance_id);
	header.primary |= IPC_PPL_TYPE(ppl_type);
	header.primary |= IPC_PPL_MEM_SIZE(ppl_mem_size);

	header.extension = IPC_PPL_LP_MODE(lp_mode);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: create pipeline fail, err: %d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_create_pipeline);

int skl_ipc_delete_pipeline(struct sst_generic_ipc *ipc, u8 instance_id)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_DELETE_PPL);
	header.primary |= IPC_INSTANCE_ID(instance_id);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: delete pipeline failed, err %d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(skl_ipc_delete_pipeline);

int skl_ipc_set_pipeline_state(struct sst_generic_ipc *ipc,
		u8 instance_id, enum skl_ipc_pipeline_state state)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_SET_PPL_STATE);
	header.primary |= IPC_INSTANCE_ID(instance_id);
	header.primary |= IPC_PPL_STATE(state);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: set pipeline state failed, err: %d\n", ret);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_set_pipeline_state);

int
skl_ipc_save_pipeline(struct sst_generic_ipc *ipc, u8 instance_id, int dma_id)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_SAVE_PPL);
	header.primary |= IPC_INSTANCE_ID(instance_id);

	header.extension = IPC_DMA_ID(dma_id);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: save pipeline failed, err: %d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_save_pipeline);

int skl_ipc_restore_pipeline(struct sst_generic_ipc *ipc, u8 instance_id)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_RESTORE_PPL);
	header.primary |= IPC_INSTANCE_ID(instance_id);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: restore  pipeline failed, err: %d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_restore_pipeline);

int skl_ipc_set_dx(struct sst_generic_ipc *ipc, u8 instance_id,
		u16 module_id, struct skl_ipc_dxstate_info *dx)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request;
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MOD_SET_DX);
	header.primary |= IPC_MOD_INSTANCE_ID(instance_id);
	header.primary |= IPC_MOD_ID(module_id);

	request.header = *(u64 *)(&header);
	request.data = dx;
	request.size = sizeof(*dx);

	dev_dbg(ipc->dev, "In %s primary =%x ext=%x\n", __func__,
			 header.primary, header.extension);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: set dx failed, err %d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_set_dx);

int skl_ipc_init_instance(struct sst_generic_ipc *ipc,
		struct skl_ipc_init_instance_msg *msg, void *param_data)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request;
	int ret;
	u32 *buffer = (u32 *)param_data;
	 /* param_block_size must be in dwords */
	u16 param_block_size = msg->param_data_size / sizeof(u32);

	print_hex_dump_debug("Param data:", DUMP_PREFIX_NONE,
		16, 4, buffer, param_block_size, false);

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MOD_INIT_INSTANCE);
	header.primary |= IPC_MOD_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MOD_ID(msg->module_id);

	header.extension = IPC_CORE_ID(msg->core_id);
	header.extension |= IPC_PPL_INSTANCE_ID(msg->ppl_instance_id);
	header.extension |= IPC_PARAM_BLOCK_SIZE(param_block_size);
	header.extension |= IPC_DOMAIN(msg->domain);

	request.header = *(u64 *)(&header);
	request.data = param_data;
	request.size = msg->param_data_size;

	dev_dbg(ipc->dev, "In %s primary =%x ext=%x\n", __func__,
			 header.primary, header.extension);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);

	if (ret < 0) {
		dev_err(ipc->dev, "ipc: init instance failed\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_init_instance);

int skl_ipc_bind_unbind(struct sst_generic_ipc *ipc,
		struct skl_ipc_bind_unbind_msg *msg)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	u8 bind_unbind = msg->bind ? IPC_MOD_BIND : IPC_MOD_UNBIND;
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(bind_unbind);
	header.primary |= IPC_MOD_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MOD_ID(msg->module_id);

	header.extension = IPC_DST_MOD_ID(msg->dst_module_id);
	header.extension |= IPC_DST_MOD_INSTANCE_ID(msg->dst_instance_id);
	header.extension |= IPC_DST_QUEUE(msg->dst_queue);
	header.extension |= IPC_SRC_QUEUE(msg->src_queue);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s hdr=%x ext=%x\n", __func__, header.primary,
			 header.extension);
	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: bind/unbind failed\n");
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_bind_unbind);

/*
 * In order to load a module we need to send IPC to initiate that. DMA will
 * performed to load the module memory. The FW supports multiple module load
 * at single shot, so we can send IPC with N modules represented by
 * module_cnt
 */
int skl_ipc_load_modules(struct sst_generic_ipc *ipc,
				u8 module_cnt, void *data)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request;
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_LOAD_MULTIPLE_MODS);
	header.primary |= IPC_LOAD_MODULE_CNT(module_cnt);

	request.header = *(u64 *)(&header);
	request.data = data;
	request.size = sizeof(u16) * module_cnt;

	ret = sst_ipc_tx_message_nowait(ipc, request);
	if (ret < 0)
		dev_err(ipc->dev, "ipc: load modules failed :%d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_load_modules);

int skl_ipc_unload_modules(struct sst_generic_ipc *ipc, u8 module_cnt,
							void *data)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request;
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_UNLOAD_MULTIPLE_MODS);
	header.primary |= IPC_LOAD_MODULE_CNT(module_cnt);

	request.header = *(u64 *)(&header);
	request.data = data;
	request.size = sizeof(u16) * module_cnt;

	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0)
		dev_err(ipc->dev, "ipc: unload modules failed :%d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_unload_modules);

int skl_ipc_set_large_config(struct sst_generic_ipc *ipc,
		struct skl_ipc_large_config_msg *msg, u32 *param)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request;
	int ret = 0;
	size_t sz_remaining, tx_size, data_offset;

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MOD_LARGE_CONFIG_SET);
	header.primary |= IPC_MOD_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MOD_ID(msg->module_id);

	header.extension = IPC_DATA_OFFSET_SZ(msg->param_data_size);
	header.extension |= IPC_LARGE_PARAM_ID(msg->large_param_id);
	header.extension |= IPC_FINAL_BLOCK(0);
	header.extension |= IPC_INITIAL_BLOCK(1);

	sz_remaining = msg->param_data_size;
	data_offset = 0;
	while (sz_remaining != 0) {
		tx_size = sz_remaining > SKL_MAILBOX_SIZE
				? SKL_MAILBOX_SIZE : sz_remaining;
		if (tx_size == sz_remaining)
			header.extension |= IPC_FINAL_BLOCK(1);

		dev_dbg(ipc->dev, "In %s primary=%#x ext=%#x\n", __func__,
			header.primary, header.extension);
		dev_dbg(ipc->dev, "transmitting offset: %#x, size: %#x\n",
			(unsigned)data_offset, (unsigned)tx_size);

		request.header = *(u64 *)(&header);
		request.data = ((char *)param) + data_offset;
		request.size = tx_size;
		ret = sst_ipc_tx_message_wait(ipc, request, NULL);
		if (ret < 0) {
			dev_err(ipc->dev,
				"ipc: set large config fail, err: %d\n", ret);
			return ret;
		}
		sz_remaining -= tx_size;
		data_offset = msg->param_data_size - sz_remaining;

		/* clear the fields */
		header.extension &= IPC_INITIAL_BLOCK_CLEAR;
		header.extension &= IPC_DATA_OFFSET_SZ_CLEAR;
		/* fill the fields */
		header.extension |= IPC_INITIAL_BLOCK(0);
		header.extension |= IPC_DATA_OFFSET_SZ(data_offset);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_set_large_config);

int skl_ipc_get_large_config(struct sst_generic_ipc *ipc,
		struct skl_ipc_large_config_msg *msg,
		u32 **payload, size_t *bytes)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request, reply = {0};
	unsigned int *buf;
	int ret;

	reply.data = kzalloc(SKL_MAILBOX_SIZE, GFP_KERNEL);
	if (!reply.data)
		return -ENOMEM;

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MOD_LARGE_CONFIG_GET);
	header.primary |= IPC_MOD_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MOD_ID(msg->module_id);

	header.extension = IPC_DATA_OFFSET_SZ(msg->param_data_size);
	header.extension |= IPC_LARGE_PARAM_ID(msg->large_param_id);
	header.extension |= IPC_FINAL_BLOCK(1);
	header.extension |= IPC_INITIAL_BLOCK(1);

	request.header = *(u64 *)&header;
	request.data = *payload;
	request.size = *bytes;
	reply.size = SKL_MAILBOX_SIZE;

	ret = sst_ipc_tx_message_wait(ipc, request, &reply);
	if (ret < 0)
		dev_err(ipc->dev, "ipc: get large config fail, err: %d\n", ret);

	reply.size = (reply.header >> 32) & IPC_DATA_OFFSET_SZ_MASK;
	buf = krealloc(reply.data, reply.size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	*payload = buf;
	*bytes = reply.size;

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_get_large_config);

void skl_ipc_set_fw_cfg(struct sst_generic_ipc *ipc, u8 instance_id,
			u16 module_id, u32 *data)
{
	struct skl_ipc_large_config_msg msg = {0};
	u32 size_offset = 1;
	int ret;

	msg.module_id = module_id;
	msg.instance_id = instance_id;
	msg.large_param_id = SET_LARGE_CFG_FW_CONFIG;
	/* size of total message = size of payload + size of headers*/
	msg.param_data_size = data[size_offset] + (2 * sizeof(u32));

	ret = skl_ipc_set_large_config(ipc, &msg, data);
	if (ret < 0)
		dev_err(ipc->dev, "ipc: set fw config failed, err %d\n", ret);
}
EXPORT_SYMBOL_GPL(skl_ipc_set_fw_cfg);

int skl_sst_ipc_load_library(struct sst_generic_ipc *ipc,
				u8 dma_id, u8 table_id, bool wait)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_LOAD_LIBRARY);
	header.primary |= IPC_MOD_INSTANCE_ID(table_id);
	header.primary |= IPC_MOD_ID(dma_id);
	request.header = *(u64 *)(&header);

	if (wait)
		ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	else
		ret = sst_ipc_tx_message_nowait(ipc, request);

	if (ret < 0)
		dev_err(ipc->dev, "ipc: load lib failed\n");

	return ret;
}
EXPORT_SYMBOL_GPL(skl_sst_ipc_load_library);

int skl_ipc_set_d0ix(struct sst_generic_ipc *ipc, struct skl_ipc_d0ix_msg *msg)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MOD_SET_D0IX);
	header.primary |= IPC_MOD_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MOD_ID(msg->module_id);

	header.extension = IPC_D0IX_WAKE(msg->wake);
	header.extension |= IPC_D0IX_STREAMING(msg->streaming);
	request.header = *(u64 *)(&header);

	dev_dbg(ipc->dev, "In %s primary=%x ext=%x\n", __func__,
			header.primary,	header.extension);

	/*
	 * Use the nopm IPC here as we dont want it checking for D0iX
	 */
	ret = sst_ipc_tx_message_nopm(ipc, request, NULL);
	if (ret < 0)
		dev_err(ipc->dev, "ipc: set d0ix failed, err %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_set_d0ix);

int skl_ipc_delete_instance(struct sst_generic_ipc *ipc,
		unsigned int module_id, unsigned int instance_id)
{
	struct skl_ipc_header header = {0};
	struct sst_ipc_message request = {0};
	int ret;

	header.primary = IPC_MSG_TARGET(IPC_MOD_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MOD_DELETE_INSTANCE);
	header.primary |= IPC_MOD_INSTANCE_ID(instance_id);
	header.primary |= IPC_MOD_ID(module_id);
	request.header = *(u64 *)&header;

	ret = sst_ipc_tx_message_wait(ipc, request, NULL);
	if (ret < 0)
		dev_err(ipc->dev, "ipc: delete instance failed, ret %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_delete_instance);

int skl_ipc_fw_cfg_get(struct sst_generic_ipc *ipc, struct skl_fw_cfg *cfg)
{
	struct skl_ipc_large_config_msg msg = {0};
	struct skl_tlv *tlv;
	size_t bytes = 0, offset = 0;
	u8 *payload = NULL;
	int ret;

	msg.module_id = 0;
	msg.instance_id = 0;
	msg.large_param_id = SKL_BASEFW_FIRMWARE_CONFIG;

	ret = skl_ipc_get_large_config(ipc, &msg, (u32 **)&payload, &bytes);
	if (ret)
		goto exit;

	while (offset < bytes) {
		tlv = (struct skl_tlv *)(payload + offset);

		switch (tlv->type) {
		case SKL_FW_CFG_FW_VERSION:
			memcpy(&cfg->fw_version, tlv->value,
				sizeof(cfg->fw_version));
			break;

		case SKL_FW_CFG_MEMORY_RECLAIMED:
			cfg->memory_reclaimed = *tlv->value;
			break;

		case SKL_FW_CFG_SLOW_CLOCK_FREQ_HZ:
			cfg->slow_clock_freq_hz = *tlv->value;
			break;

		case SKL_FW_CFG_FAST_CLOCK_FREQ_HZ:
			cfg->fast_clock_freq_hz = *tlv->value;
			break;

		case SKL_FW_CFG_ALH_SUPPORT_LEVEL:
			cfg->alh_support = *tlv->value;
			break;

		case SKL_FW_CFG_IPC_DL_MAILBOX_BYTES:
			cfg->ipc_dl_mailbox_bytes = *tlv->value;
			break;

		case SKL_FW_CFG_IPC_UL_MAILBOX_BYTES:
			cfg->ipc_ul_mailbox_bytes = *tlv->value;
			break;

		case SKL_FW_CFG_TRACE_LOG_BYTES:
			cfg->trace_log_bytes = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_PPL_COUNT:
			cfg->max_ppl_count = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_ASTATE_COUNT:
			cfg->max_astate_count = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_MODULE_PIN_COUNT:
			cfg->max_module_pin_count = *tlv->value;
			break;

		case SKL_FW_CFG_MODULES_COUNT:
			cfg->modules_count = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_MOD_INST_COUNT:
			cfg->max_mod_inst_count = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_LL_TASKS_PER_PRI_COUNT:
			cfg->max_ll_tasks_per_pri_count = *tlv->value;
			break;

		case SKL_FW_CFG_LL_PRI_COUNT:
			cfg->ll_pri_count = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_DP_TASKS_COUNT:
			cfg->max_dp_tasks_count = *tlv->value;
			break;

		case SKL_FW_CFG_MAX_LIBS_COUNT:
			cfg->max_libs_count = *tlv->value;
			break;

		case SKL_FW_CFG_XTAL_FREQ_HZ:
			cfg->xtal_freq_hz = *tlv->value;
			break;

		case SKL_FW_CFG_UAOL_SUPPORT:
			cfg->uaol_support = *tlv->value;
			break;

		case SKL_FW_CFG_POWER_GATING_POLICY:
			cfg->power_gating_policy = *tlv->value;
			break;

		case SKL_FW_CFG_DMA_BUFFER_CONFIG:
		case SKL_FW_CFG_SCHEDULER_CONFIG:
		case SKL_FW_CFG_CLOCKS_CONFIG:
			break;

		default:
			dev_info(ipc->dev, "Unrecognized fw param: %d\n",
				tlv->type);
			break;
		}

		offset += sizeof(*tlv) + tlv->length;
	}

exit:
	kfree(payload);
	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_fw_cfg_get);

int skl_ipc_hw_cfg_get(struct sst_generic_ipc *ipc, struct skl_hw_cfg *cfg)
{
	struct skl_ipc_large_config_msg msg = {0};
	struct skl_tlv *tlv;
	size_t size, bytes = 0, offset = 0;
	u8 *payload = NULL;
	int ret;

	msg.module_id = 0;
	msg.instance_id = 0;
	msg.large_param_id = SKL_BASEFW_HARDWARE_CONFIG;

	ret = skl_ipc_get_large_config(ipc, &msg, (u32 **)&payload, &bytes);
	if (ret)
		goto exit;

	while (offset < bytes) {
		tlv = (struct skl_tlv *)(payload + offset);

		switch (tlv->type) {
		case SKL_HW_CFG_CAVS_VER:
			cfg->cavs_version = *tlv->value;
			break;

		case SKL_HW_CFG_DSP_CORES:
			cfg->dsp_cores = *tlv->value;
			break;

		case SKL_HW_CFG_MEM_PAGE_BYTES:
			cfg->mem_page_bytes = *tlv->value;
			break;

		case SKL_HW_CFG_TOTAL_PHYS_MEM_PAGES:
			cfg->total_phys_mem_pages = *tlv->value;
			break;

		case SKL_HW_CFG_I2S_CAPS:
			cfg->i2s_caps.version = tlv->value[0];
			size = tlv->value[1];
			cfg->i2s_caps.ctrl_count = size;
			if (!size)
				break;

			size *= sizeof(*cfg->i2s_caps.ctrl_base_addr);
			cfg->i2s_caps.ctrl_base_addr =
				kmemdup(&tlv->value[2], size, GFP_KERNEL);
			if (!cfg->i2s_caps.ctrl_base_addr) {
				ret = -ENOMEM;
				goto exit;
			}
			break;

		case SKL_HW_CFG_GATEWAY_COUNT:
			cfg->gateway_count = *tlv->value;
			break;

		case SKL_HW_CFG_HP_EBB_COUNT:
			cfg->hp_ebb_count = *tlv->value;
			break;

		case SKL_HW_CFG_LP_EBB_COUNT:
			cfg->lp_ebb_count = *tlv->value;
			break;

		case SKL_HW_CFG_EBB_SIZE_BYTES:
			cfg->ebb_size_bytes = *tlv->value;
			break;

		case SKL_HW_CFG_GPDMA_CAPS:
		case SKL_HW_CFG_UAOL_CAPS:
			break;

		default:
			dev_info(ipc->dev, "Unrecognized hw param: %d\n",
				tlv->type);
			break;
		}

		offset += sizeof(*tlv) + tlv->length;
	}

exit:
	kfree(payload);
	return ret;
}
EXPORT_SYMBOL_GPL(skl_ipc_hw_cfg_get);

unsigned int
skl_kfifo_fromio_locked(struct kfifo *fifo, const void __iomem *src,
		unsigned int len, spinlock_t *lock)
{
	struct __kfifo *__fifo = &fifo->kfifo;
	unsigned long flags;
	unsigned int l, off;

	spin_lock_irqsave(lock, flags);
	len = min(len, kfifo_avail(fifo));
	off = __fifo->in & __fifo->mask;
	l = min(len, kfifo_size(fifo) - off);

	memcpy_fromio(__fifo->data + off, src, l);
	memcpy_fromio(__fifo->data, src + l, len - l);
	smp_mb();
	__fifo->in += len;
	spin_unlock_irqrestore(lock, flags);

	return len;
}
