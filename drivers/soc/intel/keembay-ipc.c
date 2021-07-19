// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keem Bay VPU IPC driver - IPC logic.
 *
 * Copyright (C) 2018-2021 Intel Corporation
 *
 * On the Intel Keem Bay SoC, communication between the Application
 * Processor(AP) and the VPU is enabled by the Keem Bay Inter-Processor
 * Communication (IPC) mechanism.
 *
 * Keem Bay IPC uses the following terminology:
 *
 * - Node:    A processing entity that can use the IPC to communicate
 *            (currently, we just have two nodes, the AP and the VPU).
 *
 * - Link:    Two nodes that can communicate over IPC form an IPC link
 *            (currently, we just have one link, the one formed by the AP
 *            and the VPU).
 *
 * - Channel: An IPC link can provide multiple IPC channels. IPC channels
 *            allow communication multiplexing, i.e., the same IPC link can
 *            be used by different applications for different
 *            communications. Each channel is identified by a channel ID,
 *            which must be unique within a single IPC link. Channels are
 *            divided in two categories, High-Speed (HS) channels and
 *            General-Purpose (GP) channels. HS channels have higher
 *            priority over GP channels.
 *
 * The Keem Bay IPC mechanism is built on top of the Keem Bay IPC mailbox,
 * which allows the AP and the VPU to exchange 32-bit messages. Specifically,
 * the IPC uses shared memory (shared between the AP and the VPU) to allocate
 * IPC packets and then exchanges them using the Keem Bay IPC mailbox (the
 * 32-bit physical address of the packet is passed as a message to the Keem Bay
 * IPC mailbox).
 *
 * IPC packets have a fixed structure containing the (VPU) physical address of
 * the payload (which must be located in shared memory too) as well as other
 * information (payload size, IPC channel ID, etc.).
 *
 * Each IPC node (i.e., both the AP and the VPU) has its own reserved memory
 * region (in shared memory) from which it instantiates its own pool of IPC
 * packets.  When instantiated, IPC packets are marked as free. When the node
 * needs to send an IPC message, it gets the first free packet it finds (from
 * its own pool), marks it as allocated (used), and transfer its physical
 * address to the destination node using the Keem Bay IPC mailbox. The
 * destination node uses the received physical address to access the IPC
 * packet, process the packet, and, once done with it, marks it as free (so
 * that the sender can reuse it).
 */

#include <linux/circ_buf.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h> /* Needed for MBOX_TX_QUEUE_LEN. */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include "keembay-ipc.h"

/* The alignment to be used for IPC Packets and IPC Data. */
#define KMB_IPC_ALIGNMENT	64

/* Maximum number of channels per link. */
#define KMB_IPC_MAX_CHANNELS	1024

/* The number of high-speed channels per link. */
#define KMB_IPC_NUM_HS_CHANNELS	10

/* Device tree "memory-region" used by the CPU to allocated IPC packets. */
#define IPC_MEM_CPU			"ipc-mem-cpu"

/* Device tree "memory-region" used by the VPU to allocated IPC packets. */
#define IPC_MEM_VPU			"ipc-mem-vpu"

/* The possible IPC node IDs. */
enum {
	KMB_IPC_NODE_CPU = 0,
	KMB_IPC_NODE_VPU,
};

/* The possible states of an IPC packet. */
enum {
	/*
	 * KMB_IPC_PKT_FREE must be set to 0 to ensure that packets can be
	 * initialized with memset(&buf, 0, sizeof(buf)).
	 */
	KMB_IPC_PKT_FREE = 0,
	KMB_IPC_PKT_ALLOCATED,
};

/**
 * struct kmb_ipc_pkt - The IPC packet structure.
 * @data_addr:	The address where the IPC payload is located; NOTE: this is a
 *		VPU address (not a CPU one).
 * @data_size:	The size of the payload.
 * @channel:	The channel used.
 * @src_node:	The Node ID of the sender.
 * @dst_node:	The Node ID of the intended receiver.
 * @status:	Either free or allocated.
 */
struct kmb_ipc_pkt {
	u32	data_addr;
	u32	data_size;
	u16	channel;
	u8	src_node;
	u8	dst_node;
	u8	status;
} __packed __aligned(KMB_IPC_ALIGNMENT);

/**
 * struct ipc_pkt_mem - IPC Packet Memory Region.
 * @dev:	Child device managing the memory region.
 * @vaddr:	The virtual address of the memory region.
 * @dma_handle:	The VPU address of the memory region.
 * @size:	The size of the memory region.
 */
struct ipc_pkt_mem {
	struct device	dev;
	void		*vaddr;
	dma_addr_t	dma_handle;
	size_t		size;
};

/**
 * struct ipc_pkt_pool - IPC packet pool.
 * @packets:	Pointer to the array of packets.
 * @buf_cnt:	Pool size (i.e., number of packets).
 * @idx:	Current index.
 */
struct ipc_pkt_pool {
	struct kmb_ipc_pkt	*packets;
	size_t			buf_cnt;
	size_t			idx;
};

/**
 * struct ipc_chan - IPC channel.
 * @rx_data_list:	The list of incoming messages.
 * @rx_lock:		The lock for modifying the rx_data_list.
 * @rx_wait_queue:	The wait queue for RX Data (recv() waits on it).
 * @closing:		Closing flag, set when the channel is being closed.
 */
struct ipc_chan {
	struct list_head	rx_data_list;
	spinlock_t		rx_lock; /* Protects 'rx_data_list'. */
	wait_queue_head_t	rx_wait_queue;
	bool			closing;
};

/**
 * struct tx_data - Element of a TX queue.
 * @list:	The list head used to concatenate TX data elements.
 * @vpu_addr:	The VPU address of the data to be transferred.
 * @size:	The size of the data to be transferred.
 * @chan_id:	The channel to be used for the transfer.
 * @retv:	The result of the transfer.
 * @tx_done:	The completion struct used by the sender to wait for the
 *		transfer to complete.
 * @entry:	The IPC packet VPU address to be sent to the VPU (this field is
 *		set by tx_data_send() and used by ipc_mbox_tx_done() to get a
 *		reference to the associated tx_data struct).
 */
struct tx_data {
	struct	list_head list;
	u32	vpu_addr;
	u32	size;
	u16	chan_id;
	int	retv;
	struct	completion tx_done;
	u32	entry;
};

/**
 * struct tx_queue - The TX queue structure.
 * @tx_data_list: The list of pending TX data on this queue.
 * @lock:	  The lock protecting the TX data list.
 */
struct tx_queue {
	struct list_head	tx_data_list;
	spinlock_t		lock;	/* Protects tx_data_list. */
};

/**
 * struct ipc_link - An IPC link.
 * @mbox_cl:	   The mailbox client associated with this link.
 * @mbox_chan:	   The mailbox channel associated with this link.
 * @mbox_tx_queue: The completion used to avoid overflowing the mailbox
 *		   framework queue (MBOX_TX_QUEUE_LEN), which will result in
 *		   IPC packets being dropped.
 * @channels:	   The channels associated with this link (the pointers to the
 *		   channels are RCU-protected).
 * @chan_lock:	   The lock for modifying the channels array.
 * @srcu_sp:	   The Sleepable RCU structs associated with the link's
 *		   channels.
 * @tx_queues:	   The TX queues for this link (1 queue for each high-speed
 *		   channels + 1 shared among all the general-purpose channels).
 * @tx_qidx:	   The index of the next tx_queue to be check for outgoing data.
 * @tx_queued:	   The TX completion used to signal when new TX data is pending.
 * @tx_thread:	   The TX thread.
 * @tx_stopping:   Flag signaling that the IPC Link is being closed.
 */
struct ipc_link {
	struct mbox_client	mbox_cl;
	struct mbox_chan	*mbox_chan;
	struct completion       mbox_tx_queue;
	struct ipc_chan __rcu	*channels[KMB_IPC_MAX_CHANNELS];
	spinlock_t		chan_lock; /* Protects 'channels'. */
	struct srcu_struct	srcu_sp[KMB_IPC_MAX_CHANNELS];
	struct tx_queue		tx_queues[KMB_IPC_NUM_HS_CHANNELS + 1];
	int			tx_qidx;
	struct completion	tx_queued;
	struct task_struct	*tx_thread;
	bool			tx_stopping;
};

/**
 * struct keembay_ipc - IPC private data.
 *
 * @dev:		The device associated with this IPC instance.
 * @cpu_ipc_mem:	Local IPC Packet memory region.
 * @vpu_ipc_mem:	Remove IPC Packet memory region.
 * @ipc_pkt_pool:	The pool of IPC packets.
 * @vpu_link:		The CPU-VPU link.
 */
struct keembay_ipc {
	struct device		*dev;
	struct ipc_pkt_mem	cpu_ipc_mem;
	struct ipc_pkt_mem	vpu_ipc_mem;
	struct ipc_pkt_pool	ipc_pkt_pool;
	struct ipc_link		vpu_link;
};

/**
 * struct rx_data - RX Data Descriptor.
 * @list:		List head for creating a list of rx_data elements.
 * @data_vpu_addr:	The VPU address of the received data.
 * @data_size:		The size of the received data.
 *
 * Instances of this struct are meant to be inserted in the RX Data queue
 * (list) associated with each channel.
 */
struct rx_data {
	struct list_head	list;
	u32			data_vpu_addr;
	u32			data_size;
};

/* Forward declaration of TX thread function. */
static int tx_thread_fn(void *ptr);

/* Forward declaration of mailbox client callbacks. */
static void ipc_mbox_rx_callback(struct mbox_client *cl, void *msg);
static void ipc_mbox_tx_done(struct mbox_client *cl, void *mssg, int r);

/*
 * ipc_rsvd_mem_init() - Init the specified IPC reserved memory.
 * @dev:	The IPC instance for which the memory will be initialized.
 * @mem:	Where to stored information about the initialized memory.
 * @mem_name:	The name of this IPC memory.
 *
 * Create a child device for 'dev' and use it to initialize the reserved
 * memory region specified in the device tree at index 'mem_idx'.
 * Also allocate DMA memory from the initialized memory region.
 *
 * Return:	0 on success, negative error code otherwise.
 */
static int ipc_rsvd_mem_init(struct device *dev, struct ipc_pkt_mem *mem,
			     const char *mem_name)
{
	struct device *mem_dev = &mem->dev;
	struct resource mem_res;
	struct device_node *np;
	dma_addr_t dma_handle;
	size_t mem_size;
	int mem_idx;
	void *vaddr;
	int rc;

	device_initialize(mem_dev);
	dev_set_name(mem_dev, "%s:%s", dev_name(dev), mem_name);
	mem_dev->parent = dev;
	mem_dev->dma_mask = dev->dma_mask;
	mem_dev->coherent_dma_mask = dev->coherent_dma_mask;
	mem_dev->release = of_reserved_mem_device_release;

	/* Set up DMA configuration using information from parent's DT node. */
	rc = of_dma_configure(mem_dev, dev->of_node, true);
	if (rc)
		goto err_add;

	rc = device_add(mem_dev);
	if (rc)
		goto err_add;

	/* Get memory region ID from "memory-region-names". */
	mem_idx = of_property_match_string(dev->of_node, "memory-region-names",
					   mem_name);
	/* Initialized the device reserved memory region. */
	rc = of_reserved_mem_device_init_by_idx(mem_dev, dev->of_node, mem_idx);
	if (rc) {
		dev_err(dev, "Couldn't get reserved memory with idx = %d, %d\n",
			mem_idx, rc);
		goto err_post_add;
	}

	/* Find out the size of the memory region. */
	np = of_parse_phandle(dev->of_node, "memory-region", mem_idx);
	if (!np) {
		dev_err(dev, "Couldn't find memory-region %d\n", mem_idx);
		rc = -EINVAL;
		goto err_post_add;
	}
	rc = of_address_to_resource(np, 0, &mem_res);
	if (rc) {
		dev_err(dev, "Couldn't map address to resource %d\n", mem_idx);
		goto err_post_add;
	}
	mem_size = resource_size(&mem_res);

	/* Allocate memory from the reserved memory regions */
	vaddr = dma_alloc_coherent(mem_dev, mem_size, &dma_handle, GFP_KERNEL);
	if (!vaddr) {
		rc = -ENOMEM;
		goto err_post_add;
	}

	mem->vaddr = vaddr;
	mem->dma_handle = dma_handle;
	mem->size = mem_size;

	return 0;

err_post_add:
	device_del(mem_dev);
err_add:
	put_device(mem_dev);
	return rc;
}

/*
 * ipc_rsvd_mem_release() - Release the specified IPC reserved memory.
 * @mem:	IPC reserve memory to release.
 */
static void ipc_rsvd_mem_release(struct ipc_pkt_mem *mem)
{
	/* Free allocated memory. */
	dma_free_coherent(&mem->dev, mem->size, mem->vaddr, mem->dma_handle);
	mem->vaddr = NULL;
	mem->dma_handle = DMA_MAPPING_ERROR;
	mem->size = 0;
	/* Remove device. */
	device_unregister(&mem->dev);
}

/**
 * channel_close() - Close a channel and return whether it was open or not.
 * @link:	The link the channel belongs to.
 * @chan_id:	The channel ID of the channel to close.
 *
 * Return:	0 if the channel was already closed, 1 otherwise.
 */
static int channel_close(struct ipc_link *link, u16 chan_id)
{
	struct ipc_chan *chan;
	struct rx_data *pos, *nxt;

	/* Get channel from channel array. */
	spin_lock(&link->chan_lock);
	chan = rcu_dereference_protected(link->channels[chan_id],
					 lockdep_is_held(&link->chan_lock));

	/* If channel is already NULL, we are done. */
	if (!chan) {
		spin_unlock(&link->chan_lock);
		return 0;
	}

	/* Otherwise remove it from the 'channels' array. */
	RCU_INIT_POINTER(link->channels[chan_id], NULL);
	spin_unlock(&link->chan_lock);

	/* Set closing flag and wake up user threads waiting on recv(). */
	chan->closing = true;
	wake_up_all(&chan->rx_wait_queue);

	/* Wait for channel users to drop the reference to the old channel. */
	synchronize_srcu(&link->srcu_sp[chan_id]);

	/* Free channel memory (rx_data queue and channel struct). */
	/*
	 * No need to get chan->rx_lock as we know that nobody is using the
	 * channel at this point.
	 */
	list_for_each_entry_safe(pos, nxt, &chan->rx_data_list, list) {
		list_del(&pos->list);
		kfree(pos);
	}
	kfree(chan);

	return 1;
}

/**
 * ipc_pkt_tx_alloc() - Allocate an IPC packet to be used for TX.
 * @pool:  The IPC packet pool from which the IPC packet will be allocated.
 *
 * Return: The pointer to the allocated packet, or NULL if allocation fails.
 */
static struct kmb_ipc_pkt *ipc_pkt_tx_alloc(struct ipc_pkt_pool *pool)
{
	struct kmb_ipc_pkt *buf;
	int i;

	/*
	 * Look for a free packet starting from the last index (pointing to the
	 * next packet after the last allocated one) and potentially going
	 * through all the packets in the pool.
	 */
	for (i = 0; i < pool->buf_cnt; i++) {
		/*
		 * Get reference to current packet and increment index (for
		 * next iteration or function call).
		 */
		buf = &pool->packets[pool->idx++];
		if (pool->idx == pool->buf_cnt)
			pool->idx = 0;

		/* Use current packet if free. */
		if (buf->status == KMB_IPC_PKT_FREE) {
			buf->status = KMB_IPC_PKT_ALLOCATED;
			return buf;
		}
	}

	/* We went through all the packets and found none free. */
	return NULL;
}

/**
 * init_ipc_pkt_pool() - Init the CPU IPC Packet Pool.
 * @kmb_ipc:	The IPC instance the pool belongs to.
 *
 * Set up the IPC Packet Pool to be used for allocating TX packets.
 *
 * The pool uses the CPU IPC Packet memory previously allocated.
 *
 * Return:	0 on success, negative error code otherwise.
 */
static int init_ipc_pkt_pool(struct keembay_ipc *kmb_ipc)
{
	struct ipc_pkt_mem *mem = &kmb_ipc->cpu_ipc_mem;

	kmb_ipc->ipc_pkt_pool.buf_cnt = mem->size / sizeof(struct kmb_ipc_pkt);

	/* Fail if we end up having a pool of 0 packets. */
	if (kmb_ipc->ipc_pkt_pool.buf_cnt == 0)
		return -ENOMEM;
	/*
	 * Set reserved memory to 0 to initialize the IPC Packet array
	 * (ipc_pkt_pool.packets); this works because the value of
	 * KMB_IPC_PKT_FREE is 0.
	 */
	memset(mem->vaddr, 0, mem->size);
	kmb_ipc->ipc_pkt_pool.packets = mem->vaddr;
	kmb_ipc->ipc_pkt_pool.idx = 0;

	return 0;
}

/*
 * ipc_link_init() - Initialize CPU/VPU IPC link.
 * @kmb_ipc: The IPC instance the link belongs to.
 *
 * This function is expected to be called during probing.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int ipc_link_init(struct keembay_ipc *kmb_ipc)
{
	struct ipc_link *link = &kmb_ipc->vpu_link;
	struct tx_queue *queue;
	int i;

	/* Init mailbox client. */
	link->mbox_cl.dev = kmb_ipc->dev;
	link->mbox_cl.tx_block = false;
	link->mbox_cl.tx_tout = 0;
	link->mbox_cl.knows_txdone = false;
	link->mbox_cl.rx_callback = ipc_mbox_rx_callback;
	link->mbox_cl.tx_prepare = NULL;
	link->mbox_cl.tx_done = ipc_mbox_tx_done;

	/* Init completion keeping track of free slots in mbox tx queue. */
	init_completion(&link->mbox_tx_queue);
	for (i = 0; i < MBOX_TX_QUEUE_LEN; i++)
		complete(&link->mbox_tx_queue);

	/* Request mailbox channel */
	link->mbox_chan = mbox_request_channel(&link->mbox_cl, 0);
	if (IS_ERR(link->mbox_chan))
		return PTR_ERR(link->mbox_chan);

	spin_lock_init(&link->chan_lock);
	for (i = 0; i < ARRAY_SIZE(link->srcu_sp); i++)
		init_srcu_struct(&link->srcu_sp[i]);
	memset(link->channels, 0, sizeof(link->channels));

	/* Init TX queues. */
	for (i = 0; i < ARRAY_SIZE(link->tx_queues); i++) {
		queue = &link->tx_queues[i];
		INIT_LIST_HEAD(&queue->tx_data_list);
		spin_lock_init(&queue->lock);
	}
	link->tx_qidx = 0;
	link->tx_stopping = false;
	init_completion(&link->tx_queued);

	/* Start TX thread. */
	link->tx_thread = kthread_run(tx_thread_fn, kmb_ipc,
				      "kmb_ipc_tx_thread");
	if (IS_ERR(link->tx_thread)) {
		mbox_free_channel(link->mbox_chan);
		return PTR_ERR(link->tx_thread);
	}

	return 0;
}

/**
 * ipc_link_deinit() - De-initialize CPU/VPU IPC link.
 * @kmb_ipc:	The IPC instance the link belongs to.
 */
static void ipc_link_deinit(struct keembay_ipc *kmb_ipc)
{
	struct ipc_link *link = &kmb_ipc->vpu_link;
	struct tx_queue *queue;
	struct tx_data *pos, *nxt;
	int i;

	/* Close all channels. */
	for (i = 0; i < ARRAY_SIZE(link->channels); i++)
		channel_close(link, i);

	/* Stop TX Thread. */
	link->tx_stopping = true;
	complete(&link->tx_queued);
	kthread_stop(link->tx_thread);

	/* Flush all TX queue. */
	for (i = 0; i < ARRAY_SIZE(link->tx_queues); i++) {
		queue = &link->tx_queues[i];
		list_for_each_entry_safe(pos, nxt, &queue->tx_data_list, list) {
			list_del(&pos->list);
			pos->retv = -EPIPE;
			complete(&pos->tx_done);
		}
	}

	mbox_free_channel(link->mbox_chan);
}

/* Init IPC */
struct keembay_ipc *keembay_ipc_init(struct device *dev)
{
	struct keembay_ipc *kmb_ipc;
	int rc;

	kmb_ipc = kzalloc(sizeof(*kmb_ipc), GFP_KERNEL);
	if (!kmb_ipc)
		return ERR_PTR(-ENOMEM);

	kmb_ipc->dev = dev;

	/* Init the memory used for CPU packets. */
	rc = ipc_rsvd_mem_init(dev, &kmb_ipc->cpu_ipc_mem, IPC_MEM_CPU);
	if (rc) {
		dev_err(dev, "Failed to set up CPU reserved memory.\n");
		goto err_post_ipc_dev_alloc;
	}

	/* Init the memory used for VPU packets. */
	rc = ipc_rsvd_mem_init(dev, &kmb_ipc->vpu_ipc_mem, IPC_MEM_VPU);
	if (rc) {
		dev_err(dev, "Failed to set up VPU reserved memory.\n");
		ipc_rsvd_mem_release(&kmb_ipc->cpu_ipc_mem);
		goto err_post_ipc_dev_alloc;
	}

	/* Init the pool of IPC packets to be used to TX. */
	rc = init_ipc_pkt_pool(kmb_ipc);
	if (rc)
		goto err_post_rsvd_mem;

	/* Init the only link we have (CPU -> VPU). */
	rc = ipc_link_init(kmb_ipc);
	if (rc)
		goto err_post_rsvd_mem;

	return kmb_ipc;

err_post_rsvd_mem:
	ipc_rsvd_mem_release(&kmb_ipc->cpu_ipc_mem);
	ipc_rsvd_mem_release(&kmb_ipc->vpu_ipc_mem);

err_post_ipc_dev_alloc:
	kfree(kmb_ipc);

	return ERR_PTR(rc);
}

/* De-init IPC instance and free it. */
void keembay_ipc_deinit(struct keembay_ipc *kmb_ipc)
{
	ipc_link_deinit(kmb_ipc);

	ipc_rsvd_mem_release(&kmb_ipc->cpu_ipc_mem);
	ipc_rsvd_mem_release(&kmb_ipc->vpu_ipc_mem);

	kfree(kmb_ipc);
}

/* Get VPU address and size of the IPC memory reserved to the VPU. */
int keembay_ipc_get_vpu_mem_info(struct keembay_ipc *kmb_ipc,
				 dma_addr_t *vpu_addr, size_t *size)
{
	if (!kmb_ipc)
		return -EINVAL;

	*vpu_addr = kmb_ipc->vpu_ipc_mem.dma_handle;
	*size = kmb_ipc->vpu_ipc_mem.size;

	return 0;
}

/*
 * ipc_vpu_to_virt() - Convert a VPU address to a CPU virtual address.
 *
 * @ipc_mem:  The IPC memory region where the VPU address is expected to be
 *	      mapped to.
 * @vpu_addr: The VPU address to be converted to a virtual one.
 *
 * The VPU can map the DDR differently than the CPU. This functions converts
 * VPU addresses to CPU virtual addresses.
 *
 * Return: The corresponding CPU virtual address, or NULL if the VPU address
 *	   is not in the expected memory range.
 */
static void *ipc_vpu_to_virt(const struct ipc_pkt_mem *ipc_mem,
			     u32 vpu_addr)
{
	if (unlikely(vpu_addr < ipc_mem->dma_handle ||
		     vpu_addr >= (ipc_mem->dma_handle + ipc_mem->size)))
		return NULL;

	/* Cast to (u8 *) needed since void pointer arithmetic is undefined. */
	return (u8 *)ipc_mem->vaddr + (vpu_addr - ipc_mem->dma_handle);
}

/*
 * ipc_virt_to_vpu() - Convert an CPU virtual address to a VPU address.
 * @ipc_mem:  [in]  The IPC memory region where the VPU address is expected to
 *		    be mapped to.
 * @vaddr:    [in]  The CPU virtual address to be converted to a VPU one.
 * @vpu_addr: [out] Where to store the computed VPU address.
 *
 * The VPU can map the DDR differently than the CPU. This functions converts
 * CPU virtual addresses to VPU virtual addresses.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int ipc_virt_to_vpu(struct ipc_pkt_mem *ipc_mem, void *vaddr,
			   u32 *vpu_addr)
{
	if (unlikely((ipc_mem->dma_handle + ipc_mem->size) > 0xFFFFFFFF))
		return -EINVAL;

	/* Cast to (u8 *) needed since void pointer arithmetic is undefined. */
	if (unlikely(vaddr < ipc_mem->vaddr ||
		     (u8 *)vaddr >= ((u8 *)ipc_mem->vaddr + ipc_mem->size)))
		return -EINVAL;

	*vpu_addr = ipc_mem->dma_handle + (vaddr - ipc_mem->vaddr);

	return 0;
}

/**
 * ipc_mbox_rx_callback() - Process a FIFO entry coming from IPC mailbox.
 * @cl:		The mailbox client.
 * @msg:	The FIFO entry to process.
 *
 * This function performs the following tasks:
 * - Check the source node id.
 * - Process the IPC packet (locate it, validate it, read data info, release
 *   packet).
 * - Add an RX Data descriptor (data ptr and data size) to the channel RX queue.
 */
static void ipc_mbox_rx_callback(struct mbox_client *cl, void *msg)
{
	struct device *dev = cl->dev;
	struct ipc_link *link = container_of(cl, struct ipc_link, mbox_cl);
	struct keembay_ipc *kmb_ipc = container_of(link,
						       struct keembay_ipc,
						       vpu_link);
	struct kmb_ipc_pkt *ipc_pkt;
	struct rx_data *rx_data;
	struct ipc_chan *chan;
	u32 entry;
	int idx;

	entry = *((u32 *)msg);

	/* Get IPC packet. */
	ipc_pkt = ipc_vpu_to_virt(&kmb_ipc->vpu_ipc_mem, entry);
	if (unlikely(!ipc_pkt)) {
		dev_warn(dev, "RX: Message out of expected memory range: %x\n",
			 entry);

		/* Return immediately (cannot mark the IPC packet as free). */
		return;
	}
	if (unlikely(ipc_pkt->src_node != KMB_IPC_NODE_VPU)) {
		dev_warn(dev, "RX: Message from unexpected source: %d\n",
			 ipc_pkt->src_node);
		goto exit;
	}

	/* Check destination node. */
	if (unlikely(ipc_pkt->dst_node != KMB_IPC_NODE_CPU)) {
		dev_warn(dev, "RX: Message is not for this node\n");
		goto exit;
	}

	/* Preliminary channel check. */
	if (unlikely(ipc_pkt->channel >= KMB_IPC_MAX_CHANNELS)) {
		dev_warn(dev, "RX: Message for invalid channel\n");
		goto exit;
	}

	/* Access internal channel struct (this is protected by an SRCU). */
	idx = srcu_read_lock(&link->srcu_sp[ipc_pkt->channel]);
	chan = srcu_dereference(link->channels[ipc_pkt->channel],
				&link->srcu_sp[ipc_pkt->channel]);
	if (unlikely(!chan)) {
		srcu_read_unlock(&link->srcu_sp[ipc_pkt->channel], idx);
		dev_warn(dev, "RX: Message for closed channel.\n");
		goto exit;
	}
	rx_data = kmalloc(sizeof(*rx_data), GFP_ATOMIC);
	if (unlikely(!rx_data)) {
		/* If kmalloc fails, we are forced to discard the message. */
		srcu_read_unlock(&link->srcu_sp[ipc_pkt->channel], idx);
		dev_err(dev, "RX: Message dropped: Cannot allocate RX Data.\n");
		goto exit;
	}
	/* Read data info. */
	rx_data->data_vpu_addr = ipc_pkt->data_addr;
	rx_data->data_size = ipc_pkt->data_size;
	/*
	 * Put data info in rx channel queue.
	 *
	 * Note: rx_lock is shared with user context only (and this function is
	 * run in tasklet context), so spin_lock() is enough.
	 */
	spin_lock(&chan->rx_lock);
	list_add_tail(&rx_data->list, &chan->rx_data_list);
	spin_unlock(&chan->rx_lock);

	/* Wake up thread waiting on recv(). */
	wake_up_interruptible(&chan->rx_wait_queue);

	/* Exit SRCU region protecting chan struct. */
	srcu_read_unlock(&link->srcu_sp[ipc_pkt->channel], idx);

exit:
	barrier(); /* Ensure IPC packet is fully processed before release. */
	ipc_pkt->status = KMB_IPC_PKT_FREE;
}

static void ipc_mbox_tx_done(struct mbox_client *cl, void *mssg, int r)
{
	struct tx_data *tx_data = container_of(mssg, struct tx_data, entry);
	struct ipc_link *link = container_of(cl, struct ipc_link, mbox_cl);

	/* Signal that there is one more free slot in mbox tx queue. */
	complete(&link->mbox_tx_queue);

	/* Save TX result and notify that IPC TX is completed. */
	tx_data->retv = r;
	complete(&tx_data->tx_done);
}

/**
 * tx_data_send() - Send a TX data element.
 * @kmb_ipc:	The IPC instance to use.
 * @tx_data:	The TX data element to send.
 */
static void tx_data_send(struct keembay_ipc *kmb_ipc,
			 struct tx_data *tx_data)
{
	struct device *dev = kmb_ipc->dev;
	struct ipc_link *link = &kmb_ipc->vpu_link;
	struct kmb_ipc_pkt *ipc_pkt;
	int rc;

	/* Allocate and set IPC packet. */
	ipc_pkt = ipc_pkt_tx_alloc(&kmb_ipc->ipc_pkt_pool);
	if (unlikely(!ipc_pkt)) {
		rc = -ENOMEM;
		goto error;
	}

	/* Prepare IPC packet. */
	ipc_pkt->channel = tx_data->chan_id;
	ipc_pkt->src_node = KMB_IPC_NODE_CPU;
	ipc_pkt->dst_node = KMB_IPC_NODE_VPU;
	ipc_pkt->data_addr = tx_data->vpu_addr;
	ipc_pkt->data_size = tx_data->size;

	/* Ensure changes to IPC Packet are performed before entry is sent. */
	wmb();

	/* Initialize entry to ipc_pkt VPU address. */
	rc = ipc_virt_to_vpu(&kmb_ipc->cpu_ipc_mem, ipc_pkt, &tx_data->entry);

	/*
	 * Check validity of IPC packet VPU address (this error should never
	 * occur if IPC packet region is defined properly in Device Tree).
	 */
	if (unlikely(rc)) {
		dev_err(dev, "Cannot convert IPC buf vaddr to vpu_addr: %p\n",
			ipc_pkt);
		rc = -ENXIO;
		goto error;
	}
	if (unlikely(!IS_ALIGNED(tx_data->entry, KMB_IPC_ALIGNMENT))) {
		dev_err(dev, "Allocated IPC buf is not 64-byte aligned: %p\n",
			ipc_pkt);
		rc = -EFAULT;
		goto error;
	}

	/*
	 * Ensure that the mbox TX queue is not full before passing the packet
	 * to mbox controller with mbox_send_message(). This will ensure that
	 * the packet won't be dropped by the mbox framework with the error
	 * "Try increasing MBOX_TX_QUEUE_LEN".
	 */
	rc = wait_for_completion_interruptible(&link->mbox_tx_queue);
	if (unlikely(rc))
		goto error;

	mbox_send_message(link->mbox_chan, &tx_data->entry);

	return;

error:
	/* If an error occurred and a packet was allocated, free it. */
	if (ipc_pkt)
		ipc_pkt->status = KMB_IPC_PKT_FREE;

	tx_data->retv = rc;
	complete(&tx_data->tx_done);
}

/**
 * tx_data_dequeue() - Dequeue the next TX data waiting for transfer.
 * @link:  The link from which we dequeue the TX Data.
 *
 * The de-queue policy is round robin between each high speed channel queue and
 * the one queue for all low speed channels.
 *
 * Return: The next TX data waiting to be transferred, or NULL if no TX is
 *	   pending.
 */
static struct tx_data *tx_data_dequeue(struct ipc_link *link)
{
	struct tx_data *tx_data;
	struct tx_queue *queue;
	int i;

	/*
	 * TX queues are logically organized in a circular array.
	 * We go through such an array until we find a non-empty queue.
	 * We start from where we left since last function invocation.
	 * If all queues are empty we return NULL.
	 */
	for (i = 0; i < ARRAY_SIZE(link->tx_queues); i++) {
		queue = &link->tx_queues[link->tx_qidx];
		link->tx_qidx++;
		if (link->tx_qidx == ARRAY_SIZE(link->tx_queues))
			link->tx_qidx = 0;

		spin_lock(&queue->lock);
		tx_data = list_first_entry_or_null(&queue->tx_data_list,
						   struct tx_data, list);
		/* If no data in the queue, process the next queue. */
		if (!tx_data) {
			spin_unlock(&queue->lock);
			continue;
		}
		/* Otherwise remove rx_data from queue and return. */
		list_del(&tx_data->list);
		spin_unlock(&queue->lock);

		return tx_data;
	}

	return NULL;
}

/**
 * tx_data_enqueue() - Enqueue TX data for transfer into the specified link.
 * @link:	The link the data is enqueued to.
 * @tx_data:	The TX data to enqueue.
 */
static void tx_data_enqueue(struct ipc_link *link, struct tx_data *tx_data)
{
	struct tx_queue *queue;
	int qid;

	/*
	 * Find the right queue where to put TX data:
	 * - Each high-speed channel has a dedicated queue, whose index is the
	 *   same as the channel id (e.g., Channel 1 uses tx_queues[1]).
	 * - All the general-purpose channels use the same TX queue, which is
	 *   the last element in the tx_queues array.
	 *
	 * Note: tx_queues[] has KMB_IPC_NUM_HS_CHANNELS+1 elements)
	 */
	qid = tx_data->chan_id < ARRAY_SIZE(link->tx_queues) ?
	      tx_data->chan_id : (ARRAY_SIZE(link->tx_queues) - 1);

	queue = &link->tx_queues[qid];

	/*
	 * Lock shared between user contexts (callers of ipc_send()) and tx
	 * thread; so spin_lock() is enough.
	 */
	spin_lock(&queue->lock);
	list_add_tail(&tx_data->list, &queue->tx_data_list);
	spin_unlock(&queue->lock);
}

/**
 * tx_data_remove() - Remove TX data element from specified link.
 * @link:	The link the data is currently enqueued to.
 * @tx_data:	The TX data element to be removed.
 *
 * This function is called by the main send function, when the send is
 * interrupted or has timed out.
 */
static void tx_data_remove(struct ipc_link *link, struct tx_data *tx_data)
{
	struct tx_queue *queue;
	int qid;

	/*
	 * Find the TX queue where TX data is currently located:
	 * - Each high-speed channel has a dedicated queue, whose index is the
	 *   same as the channel id (e.g., Channel 1 uses tx_queues[1]).
	 * - All the general-purpose channels use the same TX queue, which is
	 *   the last element in the tx_queues array.
	 *
	 * Note: tx_queues[] has KMB_IPC_NUM_HS_CHANNELS+1 elements)
	 */
	qid = tx_data->chan_id < ARRAY_SIZE(link->tx_queues) ?
	      tx_data->chan_id : (ARRAY_SIZE(link->tx_queues) - 1);

	queue = &link->tx_queues[qid];

	/*
	 * Lock shared between user contexts (callers of ipc_send()) and tx
	 * thread; so spin_lock() is enough.
	 */
	spin_lock(&queue->lock);
	list_del(&tx_data->list);
	spin_unlock(&queue->lock);
}

/**
 * tx_thread_fn() - The function run by the TX thread.
 * @ptr: A pointer to the keembay_ipc struct associated with the thread.
 *
 * This thread continuously dequeues and send TX data elements. The TX
 * semaphore is used to pause the loop when all the pending TX data elements
 * have been transmitted (the send function 'ups' the semaphore every time a
 * new TX data element is enqueued).
 */
static int tx_thread_fn(void *ptr)
{
	struct keembay_ipc *kmb_ipc = ptr;
	struct ipc_link *link = &kmb_ipc->vpu_link;
	struct tx_data *tx_data;
	int rc;

	while (1) {
		rc = wait_for_completion_interruptible(&link->tx_queued);
		if (rc || link->tx_stopping)
			break;
		tx_data = tx_data_dequeue(link);
		/*
		 * We can get a null tx_data if tx_data_remove() has been
		 * called. Just ignore it and continue.
		 */
		if (!tx_data)
			continue;
		tx_data_send(kmb_ipc, tx_data);
	}

	/* Wait until kthread_stop() is called. */
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);

	return rc;
}

/* Internal send. */
static int __ipc_send(struct keembay_ipc *kmb_ipc, u16 chan_id, u32 vpu_addr,
		      size_t size)
{
	struct ipc_link *link = &kmb_ipc->vpu_link;
	struct tx_data *tx_data;
	int rc;

	/* Allocate and init TX data. */
	tx_data = kmalloc(sizeof(*tx_data), GFP_KERNEL);
	if (!tx_data)
		return -ENOMEM;
	tx_data->chan_id = chan_id;
	tx_data->vpu_addr = vpu_addr;
	tx_data->size = size;
	tx_data->retv = 1;
	INIT_LIST_HEAD(&tx_data->list);
	init_completion(&tx_data->tx_done);

	/* Add tx_data to tx queues. */
	tx_data_enqueue(link, tx_data);

	/* Signal that we have a new pending TX. */
	complete(&link->tx_queued);

	/* Wait until data is transmitted. */
	rc = wait_for_completion_interruptible(&tx_data->tx_done);
	if (unlikely(rc)) {
		tx_data_remove(link, tx_data);
		goto exit;
	}
	rc = tx_data->retv;

exit:
	kfree(tx_data);
	return rc;
}

/*
 * Perform basic validity check on common API arguments.
 *
 * Verify the Keem Bay IPC instance is not NULL and that and the channel ID is
 * within the allowed range.
 */
static int validate_api_args(struct keembay_ipc *kmb_ipc, u16 chan_id)
{
	if (!kmb_ipc)
		return -EINVAL;

	if (chan_id >= KMB_IPC_MAX_CHANNELS) {
		dev_warn(kmb_ipc->dev, "Invalid Channel ID\n");
		return -EINVAL;
	}
	return 0;
}

/**
 * keembay_ipc_open_channel() - Open an IPC channel.
 * @kmb_ipc:	The IPC instance to use.
 * @chan_id:	The ID of the channel to be opened.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int keembay_ipc_open_channel(struct keembay_ipc *kmb_ipc, u16 chan_id)
{
	struct ipc_chan *chan, *cur_chan;
	struct ipc_link *link;
	int rc;

	rc = validate_api_args(kmb_ipc, chan_id);
	if (rc)
		return rc;

	link = &kmb_ipc->vpu_link;

	/* Create channel before getting lock. */
	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	INIT_LIST_HEAD(&chan->rx_data_list);
	spin_lock_init(&chan->rx_lock);
	init_waitqueue_head(&chan->rx_wait_queue);

	/* Add channel to the channel array (if not already present). */
	spin_lock(&link->chan_lock);
	cur_chan = rcu_dereference_protected(link->channels[chan_id],
					     lockdep_is_held(&link->chan_lock));
	if (cur_chan) {
		spin_unlock(&link->chan_lock);
		kfree(chan);
		return -EEXIST;
	}
	rcu_assign_pointer(link->channels[chan_id], chan);
	spin_unlock(&link->chan_lock);

	return 0;
}

/**
 * keembay_ipc_close_channel() - Close an IPC channel.
 * @kmb_ipc:	The IPC instance to use.
 * @chan_id:	The ID of the channel to be closed.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int keembay_ipc_close_channel(struct keembay_ipc *kmb_ipc, u16 chan_id)
{
	struct ipc_link *link;
	int rc;

	rc = validate_api_args(kmb_ipc, chan_id);
	if (rc)
		return rc;

	link = &kmb_ipc->vpu_link;

	rc = channel_close(link, chan_id);
	if (!rc)
		dev_info(kmb_ipc->dev, "Channel was already closed\n");

	return 0;
}

/**
 * keembay_ipc_send() - Send data to VPU via IPC.
 * @kmb_ipc:	The IPC instance to use.
 * @chan_id:	The IPC channel to be used to send the message.
 * @vpu_addr:	The VPU address of the data to be transferred.
 * @size:	The size of the data to be transferred.
 *
 * Return:	0 on success, negative error code otherwise.
 */
int keembay_ipc_send(struct keembay_ipc *kmb_ipc, u16 chan_id, u32 vpu_addr,
		     size_t size)
{
	struct ipc_link *link;
	struct ipc_chan *chan;
	int idx, rc;

	rc = validate_api_args(kmb_ipc, chan_id);
	if (rc)
		return rc;

	link = &kmb_ipc->vpu_link;
	/*
	 * Start Sleepable RCU critical section (this prevents close() from
	 * destroying the channels struct while we are sending data)
	 */
	idx = srcu_read_lock(&link->srcu_sp[chan_id]);

	/* Get channel. */
	chan = srcu_dereference(link->channels[chan_id],
				&link->srcu_sp[chan_id]);
	if (unlikely(!chan)) {
		/* The channel is closed. */
		rc = -ENOENT;
		goto exit;
	}

	rc = __ipc_send(kmb_ipc, chan_id, vpu_addr, size);

exit:
	/* End sleepable RCU critical section. */
	srcu_read_unlock(&link->srcu_sp[chan_id], idx);
	return rc;
}

/**
 * keembay_ipc_recv() - Read data received from VPU via IPC
 * @kmb_ipc:	The IPC instance to use.
 * @chan_id:	The IPC channel to read from.
 * @vpu_addr:	[out] The VPU address of the received data.
 * @size:	[out] Where to store the size of the received data.
 * @timeout:	How long (in ms) the function will block waiting for an IPC
 *		message; if UINT32_MAX it will block indefinitely; if 0 it
 *		will not block.
 *
 * Return:	0 on success, negative error code otherwise
 */
int keembay_ipc_recv(struct keembay_ipc *kmb_ipc, u16 chan_id, u32 *vpu_addr,
		     size_t *size, u32 timeout)
{
	struct rx_data *rx_entry;
	struct ipc_link *link;
	struct ipc_chan *chan;
	int idx, rc;

	rc = validate_api_args(kmb_ipc, chan_id);
	if (rc)
		return rc;

	if (!vpu_addr || !size)
		return -EINVAL;

	link = &kmb_ipc->vpu_link;

	/*
	 * Start Sleepable RCU critical section (this prevents close() from
	 * destroying the channels struct while we are using it)
	 */
	idx = srcu_read_lock(&link->srcu_sp[chan_id]);

	/* Get channel. */
	chan = srcu_dereference(link->channels[chan_id],
				&link->srcu_sp[chan_id]);
	if (unlikely(!chan)) {
		rc = -ENOENT;
		goto err;
	}
	/*
	 * Get the lock protecting rx_data_list; the lock will be released by
	 * wait_event_*_lock_irq() before going to sleep and automatically
	 * reacquired after wake up.
	 *
	 * NOTE: lock_irq() is needed because rx_lock is also used by the RX
	 * tasklet; also lock_bh() is not used because there is no
	 * wait_event_interruptible_lock_bh().
	 */
	spin_lock_irq(&chan->rx_lock);
	/*
	 * Wait for RX data.
	 *
	 * Note: wait_event_interruptible_lock_irq_timeout() has different
	 * return values than wait_event_interruptible_lock_irq().
	 *
	 * The following if/then branch ensures that return values are
	 * consistent for the both cases, that is:
	 * - rc == 0 only if the wait was successfully (i.e., we were notified
	 *   of a message or of a channel closure)
	 * - rc < 0 if an error occurred (we got interrupted or the timeout
	 *   expired).
	 */
	if (timeout == U32_MAX) {
		rc = wait_event_interruptible_lock_irq(chan->rx_wait_queue,
						       !list_empty(&chan->rx_data_list) ||
						       chan->closing,
						       chan->rx_lock);
	} else {
		rc = wait_event_interruptible_lock_irq_timeout(chan->rx_wait_queue,
							       !list_empty(&chan->rx_data_list) ||
							       chan->closing,
							       chan->rx_lock,
							       msecs_to_jiffies(timeout));
		if (!rc)
			rc = -ETIME;
		if (rc > 0)
			rc = 0;
	}

	/* Check if the channel was closed while waiting. */
	if (chan->closing)
		rc = -EPIPE;
	if (rc) {
		spin_unlock_irq(&chan->rx_lock);
		goto err;
	}

	/* Extract RX entry. */
	rx_entry = list_first_entry(&chan->rx_data_list, struct rx_data, list);
	list_del(&rx_entry->list);
	spin_unlock_irq(&chan->rx_lock);

	/* Set output parameters. */
	*vpu_addr =  rx_entry->data_vpu_addr;
	*size = rx_entry->data_size;

	/* Free RX entry. */
	kfree(rx_entry);

err:
	/* End sleepable RCU critical section. */
	srcu_read_unlock(&link->srcu_sp[chan_id], idx);
	return rc;
}
