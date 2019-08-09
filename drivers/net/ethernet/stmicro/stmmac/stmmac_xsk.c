// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2019 Intel Corporation. */

#include <linux/bpf_trace.h>
#include <net/xdp_sock.h>
#include <net/xdp.h>

#include "stmmac.h"
#include "stmmac_xsk.h"

struct xdp_umem *stmmac_xsk_umem(struct stmmac_priv *priv, u16 qid)
{
	bool xdp_on = stmmac_enabled_xdp(priv);

	if (!xdp_on)
		return NULL;

	if (!queue_is_xdp(qid))
		return NULL;

	if (!test_bit(qid, priv->has_xdp_zc_umem))
		return NULL;

	return xdp_get_umem_from_qid(priv->dev, qid);
}

static int stmmac_xsk_umem_dma_map(struct stmmac_priv *priv,
				   struct xdp_umem *umem)
{
	struct device *dev = priv->device;
	unsigned int i, j;
	dma_addr_t dma;

	for (i = 0; i < umem->npgs; i++) {
		dma = dma_map_page_attrs(dev, umem->pgs[i], 0, PAGE_SIZE,
					 DMA_BIDIRECTIONAL,
					 STMMAC_RX_DMA_ATTR);
		if (dma_mapping_error(dev, dma))
			goto out_unmap;

		umem->pages[i].dma = dma;
	}

	return 0;

out_unmap:
	for (j = 0; j < i; j++) {
		dma_unmap_page_attrs(dev, umem->pages[i].dma, PAGE_SIZE,
				     DMA_BIDIRECTIONAL, STMMAC_RX_DMA_ATTR);
		umem->pages[i].dma = 0;
	}

	return -EPERM;
}

static void stmmac_xsk_umem_dma_unmap(struct stmmac_priv *priv,
				      struct xdp_umem *umem)
{
	struct device *dev = priv->device;
	unsigned int i;

	for (i = 0; i < umem->npgs; i++) {
		dma_unmap_page_attrs(dev, umem->pages[i].dma, PAGE_SIZE,
				     DMA_BIDIRECTIONAL, STMMAC_RX_DMA_ATTR);

		umem->pages[i].dma = 0;
	}
}

static int stmmac_xsk_umem_enable(struct stmmac_priv *priv,
				  struct xdp_umem *umem,
				  u16 qid)
{
	struct xdp_umem_fq_reuse *reuseq;
	bool if_running;
	int err;

	if (qid >= priv->plat->rx_queues_to_use ||
	    qid >= priv->plat->tx_queues_to_use)
		return -EINVAL;

	reuseq = xsk_reuseq_prepare(priv->dma_rx_size);
	if (!reuseq)
		return -ENOMEM;

	xsk_reuseq_free(xsk_reuseq_swap(umem, reuseq));

	err = stmmac_xsk_umem_dma_map(priv, umem);
	if (err)
		return err;

	set_bit(qid, priv->has_xdp_zc_umem);

	if_running = netif_running(priv->dev) && READ_ONCE(priv->xdp_prog);

	if (if_running) {
		stmmac_txrx_ring_disable(priv, qid);
		stmmac_txrx_ring_enable(priv, qid);
		/* Kick start the NAPI context so that receiving will start */
		err = stmmac_xsk_async_xmit(priv->dev, qid);
		if (err)
			return err;
	}

	return 0;
}

static int stmmac_xsk_umem_disable(struct stmmac_priv *priv, u16 qid)
{
	struct xdp_umem *umem;
	bool if_running;

	umem = xdp_get_umem_from_qid(priv->dev, qid);
	if (!umem)
		return -EINVAL;

	if_running = netif_running(priv->dev) && READ_ONCE(priv->xdp_prog);

	if (if_running)
		stmmac_txrx_ring_disable(priv, qid);

	clear_bit(qid, priv->has_xdp_zc_umem);

	/* UMEM is shared for both Tx & Rx, we unmap once */
	stmmac_xsk_umem_dma_unmap(priv, umem);

	if (if_running)
		stmmac_txrx_ring_enable(priv, qid);

	return 0;
}

int stmmac_xsk_umem_setup(struct stmmac_priv *priv, struct xdp_umem *umem,
			  u16 qid)
{
	return umem ? stmmac_xsk_umem_enable(priv, umem, qid) :
		      stmmac_xsk_umem_disable(priv, qid);
}

int stmmac_run_xdp_zc(struct stmmac_priv *priv, struct stmmac_rx_queue *rx_q,
		      struct xdp_buff *xdp)
{
	int err, result = STMMAC_XDP_PASS;
	struct bpf_prog *xdp_prog;
	struct xdp_frame *xdpf;
	u32 act;

	rcu_read_lock();
	xdp_prog = READ_ONCE(rx_q->xdp_prog);
	act = bpf_prog_run_xdp(xdp_prog, xdp);
	xdp->handle += xdp->data - xdp->data_hard_start;
	switch (act) {
	case XDP_PASS:
		break;
	case XDP_REDIRECT:
		err = xdp_do_redirect(priv->dev, xdp, xdp_prog);
		result = (err >= 0) ? STMMAC_XDP_REDIRECT : STMMAC_XDP_DROP;
		break;
	default:
		bpf_warn_invalid_xdp_action(act);
		/* Fallthrough */
	case XDP_ABORTED:
		trace_xdp_exception(priv->dev, xdp_prog, act);
		/* Fallthrough -- handle aborts by dropping packet */
	case XDP_DROP:
		result = STMMAC_XDP_DROP;
		break;
	}
	rcu_read_unlock();
	return result;
}

struct stmmac_rx_buffer *stmmac_get_rx_buffer_zc(struct stmmac_rx_queue *rx_q,
						 unsigned int size)
{
	struct stmmac_rx_buffer *buf;

	buf = &rx_q->buf_pool[rx_q->cur_rx];

	/* We are reusing so sync this buffer for CPU use */
	dma_sync_single_range_for_cpu(rx_q->priv_data->device,
				      buf->dma_addr, 0,
				      size,
				      DMA_BIDIRECTIONAL);

	return buf;
}

void stmmac_reuse_rx_buffer_zc(struct stmmac_rx_queue *rx_q,
			       struct stmmac_rx_buffer *obuf)
{
	unsigned long mask = (unsigned long)rx_q->xsk_umem->chunk_mask;
	u64 hr = rx_q->xsk_umem->headroom + XDP_PACKET_HEADROOM;
	u16 nta = rx_q->next_to_alloc;
	struct stmmac_rx_buffer *nbuf;

	nbuf = &rx_q->buf_pool[nta];
	/* Update, and store next to alloc */
	nta++;
	rx_q->next_to_alloc = (nta < rx_q->priv_data->dma_rx_size) ? nta : 0;

	/* Transfer page from old buffer to new buffer */
	nbuf->dma_addr = obuf->dma_addr & mask;
	nbuf->dma_addr += hr;

	nbuf->addr = (void *)((unsigned long)obuf->addr & mask);
	nbuf->addr += hr;

	nbuf->handle = obuf->handle & mask;
	nbuf->handle += rx_q->xsk_umem->headroom;

	obuf->addr = NULL;
}

void stmmac_zca_free(struct zero_copy_allocator *alloc, unsigned long handle)
{
	struct stmmac_rx_buffer *buf;
	struct stmmac_rx_queue *rx_q;
	struct stmmac_priv *priv;
	unsigned int nta;
	u64 hr, mask;

	rx_q = container_of(alloc, struct stmmac_rx_queue, zca);
	priv = rx_q->priv_data;

	hr = rx_q->xsk_umem->headroom + XDP_PACKET_HEADROOM;
	mask = rx_q->xsk_umem->chunk_mask;

	nta = rx_q->next_to_alloc;
	buf = &rx_q->buf_pool[nta];
	nta++;
	rx_q->next_to_alloc = (nta < priv->dma_rx_size) ? nta : 0;

	handle &= mask;

	buf->dma_addr = xdp_umem_get_dma(rx_q->xsk_umem, handle);
	buf->dma_addr += hr;

	buf->addr = xdp_umem_get_data(rx_q->xsk_umem, handle);
	buf->addr += hr;

	buf->handle = (u64)handle + rx_q->xsk_umem->headroom;
}

static bool stmmac_alloc_buffer_fast_zc(struct stmmac_rx_queue *rx_q,
					struct stmmac_rx_buffer *buf)
{
	struct xdp_umem *umem = rx_q->xsk_umem;
	void *addr = buf->addr;
	u64 handle, hr;

	if (addr)
		return true;

	if (!xsk_umem_peek_addr(umem, &handle))
		return false;

	/* TODO: Add XDP statistics for alloc failures */

	hr = umem->headroom + XDP_PACKET_HEADROOM;

	buf->dma_addr = xdp_umem_get_dma(umem, handle);
	buf->dma_addr += hr;

	buf->addr = xdp_umem_get_data(umem, handle);
	buf->addr += hr;

	buf->handle = handle + umem->headroom;

	xsk_umem_discard_addr(umem);
	return true;
}

static bool stmmac_alloc_buffer_slow_zc(struct stmmac_rx_queue *rx_q,
					struct stmmac_rx_buffer *buf)
{
	struct xdp_umem *umem = rx_q->xsk_umem;
	u64 handle, hr;

	if (!xsk_umem_peek_addr_rq(umem, &handle))
		return false;

	/* TODO: Add XDP statistics for alloc failures */

	handle &= rx_q->xsk_umem->chunk_mask;

	hr = umem->headroom + XDP_PACKET_HEADROOM;

	buf->dma_addr = xdp_umem_get_dma(umem, handle);
	buf->dma_addr += hr;

	buf->addr = xdp_umem_get_data(umem, handle);
	buf->addr += hr;

	buf->handle = handle + umem->headroom;

	xsk_umem_discard_addr_rq(umem);
	return true;
}

static __always_inline bool
__stmmac_alloc_rx_buffers_zc(struct stmmac_rx_queue *rx_q, u16 cleaned_count,
			     bool alloc(struct stmmac_rx_queue *rx_q,
					struct stmmac_rx_buffer *buf))
{
	bool ok = true;
	u16 i = rx_q->dirty_rx;
	u16 entry = rx_q->dirty_rx;
	struct stmmac_priv *priv = rx_q->priv_data;

	/* Nothing to do */
	if (!cleaned_count)
		return true;

	i -= rx_q->priv_data->dma_rx_size;

	do {
		struct dma_desc *p;
		struct stmmac_rx_buffer *buf = &rx_q->buf_pool[entry];

		if (!alloc(rx_q, buf)) {
			ok = false;
			break;
		}

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		/* Sync the buffer for use by the device */
		dma_sync_single_range_for_device(priv->device,
						 buf->dma_addr,
						 0, //buf->page_offset,
						 rx_q->xsk_buf_len,
						 DMA_BIDIRECTIONAL);

		/* Refresh the desc even if buffer_addrs didn't change
		 * because each write-back erases this info.
		 */
		stmmac_set_desc_addr(priv, p, buf->dma_addr);
		stmmac_refill_desc3(priv, rx_q, p);

		dma_wmb();
		stmmac_set_rx_owner(priv, p, priv->use_riwt);
		dma_wmb();

		i++;
		entry = STMMAC_GET_ENTRY(entry,  priv->dma_rx_size);
		if (unlikely(!i))
			i -= rx_q->priv_data->dma_rx_size;

		cleaned_count--;
	} while (cleaned_count);

	i += rx_q->priv_data->dma_rx_size;

	if (rx_q->dirty_rx != i) {
		/* Update next to alloc since we have filled the ring */
		rx_q->next_to_alloc = i;
		rx_q->dirty_rx = i;

		/* Init Rx DMA - DMA_CHAN_RX_END_ADDR */
		rx_q->rx_tail_addr = rx_q->dma_rx_phy +
				     (priv->dma_rx_size *
				      sizeof(struct dma_desc));
		stmmac_set_rx_tail_ptr(priv, priv->ioaddr,
				       rx_q->rx_tail_addr, rx_q->queue_index);
	}

	return ok;
}

void stmmac_alloc_rx_buffers_slow_zc(struct stmmac_rx_queue *rx_q,
				     u16 cleaned_count)
{
	__stmmac_alloc_rx_buffers_zc(rx_q, cleaned_count,
				     stmmac_alloc_buffer_slow_zc);
}

bool stmmac_alloc_rx_buffers_fast_zc(struct stmmac_rx_queue *rx_q,
				     u16 cleaned_count)
{
	return __stmmac_alloc_rx_buffers_zc(rx_q, cleaned_count,
					    stmmac_alloc_buffer_fast_zc);
}

static struct sk_buff *stmmac_construct_skb_zc(struct stmmac_rx_queue *rx_q,
					       struct stmmac_rx_buffer *buf,
					       struct xdp_buff *xdp)
{
	struct sk_buff *skb;
	unsigned int metasize = xdp->data - xdp->data_meta;
	unsigned int datasize = xdp->data_end - xdp->data;
	struct stmmac_priv *priv = rx_q->priv_data;
	u32 qid = rx_q->queue_index;
	struct stmmac_channel *ch = &priv->channel[qid];

	/* allocate a skb to store the frags */
	skb = __napi_alloc_skb(&ch->rx_napi,
			       xdp->data_end - xdp->data_hard_start,
			       GFP_ATOMIC | __GFP_NOWARN);
	if (unlikely(!skb))
		return NULL;

	skb_reserve(skb, xdp->data - xdp->data_hard_start);
	memcpy(__skb_put(skb, datasize), xdp->data, datasize);
	if (metasize)
		skb_metadata_set(skb, metasize);

	stmmac_reuse_rx_buffer_zc(rx_q, buf);
	return skb;
}

/* RX Q next_to_clean increment, prefetch and rollover function */
static struct dma_desc *stmmac_inc_ntc(struct stmmac_rx_queue *rx_q)
{
	struct dma_desc *np;
	struct stmmac_priv *priv = rx_q->priv_data;
	u32 ntc = rx_q->cur_rx + 1;

	ntc = (ntc < priv->dma_rx_size) ? ntc : 0;
	rx_q->cur_rx = ntc;

	if (priv->extend_desc)
		np = (struct dma_desc *)(rx_q->dma_erx + ntc);
	else
		np = rx_q->dma_rx + ntc;

	prefetch(np);

	return np;
}

int stmmac_rx_zc(struct stmmac_priv *priv, const int budget, u32 qid)
{
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	struct stmmac_rx_queue *rx_q = &priv->rx_queue[qid];
	struct stmmac_channel *ch = &priv->channel[qid];
	unsigned int xdp_res, xdp_xmit = 0;
	int coe = priv->hw->rx_csum;
	bool failure = false;
	struct sk_buff *skb;
	struct xdp_buff xdp;
	unsigned int entry;
	u16 cleaned_count;
	int count = 0;

	xdp.rxq = &rx_q->xdp_rxq;
	cleaned_count = stmmac_rx_dirty(rx_q);
	entry = rx_q->cur_rx - 1;	/* Offset the while loop's entry++ */

	while (likely(total_rx_packets < budget) && count < priv->dma_rx_size) {
		struct dma_desc *rx_desc, *rx_desc_n;
		struct stmmac_rx_buffer *buf;
		int size, status;

		entry++;

		if (cleaned_count >= STMMAC_RX_BUFFER_WRITE) {
			failure = failure ||
				  !stmmac_alloc_rx_buffers_fast_zc(rx_q,
								 cleaned_count);
			cleaned_count = 0;
		}

		if (priv->extend_desc)
			rx_desc = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			rx_desc = rx_q->dma_rx + entry;

		/* Read the status of the incoming frame */
		status = stmmac_rx_status(priv, &priv->dev->stats,
					  &priv->xstats, rx_desc);

		/* check if managed by the DMA otherwise go ahead */
		if (unlikely(status & dma_own))
			break;

		if (priv->extend_desc)
			stmmac_rx_extended_status(priv, &priv->dev->stats,
						  &priv->xstats,
						  rx_q->dma_erx + entry);

		count++;

		size = stmmac_get_rx_frame_len(priv, rx_desc, coe);

		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * descriptor has been written back.
		 */
		dma_rmb();

		buf = stmmac_get_rx_buffer_zc(rx_q, size);

		/* Check if valid dma addr */
		if (!unlikely(buf->dma_addr)) {
			stmmac_reuse_rx_buffer_zc(rx_q, buf);
			cleaned_count++;
			continue;
		}

		if (unlikely(size <= 0)) {
			stmmac_reuse_rx_buffer_zc(rx_q, buf);
			cleaned_count++;
			stmmac_inc_ntc(rx_q);
			continue;
		}

		xdp.data = buf->addr;
		xdp.data_meta = xdp.data;
		xdp.data_hard_start = xdp.data - XDP_PACKET_HEADROOM;
		xdp.data_end = xdp.data + size;
		xdp.handle = buf->handle;

		xdp_res = stmmac_run_xdp_zc(priv, rx_q, &xdp);

		if (xdp_res) {
			if (xdp_res & (STMMAC_XDP_TX | STMMAC_XDP_REDIRECT)) {
				xdp_xmit |= xdp_res;
				buf->addr = NULL;
			} else {
				stmmac_reuse_rx_buffer_zc(rx_q, buf);
			}
			total_rx_packets++;
			total_rx_bytes += size;

			cleaned_count++;
			stmmac_inc_ntc(rx_q);

			continue; /* packets processed, go to the next one */
		}

		/* XDP_PASS path */
		skb = stmmac_construct_skb_zc(rx_q, buf, &xdp);
		if (!skb)
			break;

		/* TODO: Add XDP statistics */

		cleaned_count++;
		rx_desc_n = stmmac_inc_ntc(rx_q);

		if (eth_skb_pad(skb))
			continue;

		total_rx_bytes += skb->len;
		total_rx_packets++;

		if (netif_msg_pktdata(priv)) {
			netdev_dbg(priv->dev, "frame received (%dbytes)",
				   size);
			print_pkt(skb->data, size);
		}

		stmmac_get_rx_hwtstamp(priv, rx_desc, rx_desc_n, skb);

		if (-EINVAL == stmmac_rx_hw_vlan(priv, priv->dev,
						 priv->hw, rx_desc, skb))
			stmmac_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);

		if (unlikely(!coe))
			skb_checksum_none_assert(skb);
		else
			skb->ip_summed = CHECKSUM_UNNECESSARY;

		napi_gro_receive(&ch->rx_napi, skb);
	}

	if (xdp_xmit & STMMAC_XDP_REDIRECT)
		xdp_do_flush_map();

	if (xdp_xmit & STMMAC_XDP_TX) {
		struct stmmac_tx_queue *tx_q = &priv->tx_queue[qid];

		stmmac_enable_dma_transmission(priv, priv->ioaddr);

		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.
		 */
		wmb();

		if (priv->extend_desc)
			tx_q->tx_tail_addr = tx_q->dma_tx_phy + (tx_q->cur_tx *
					sizeof(struct dma_extended_desc));
		else if (priv->enhanced_tx_desc)
			tx_q->tx_tail_addr = tx_q->dma_tx_phy + (tx_q->cur_tx *
					sizeof(struct dma_enhanced_tx_desc));
		else
			tx_q->tx_tail_addr = tx_q->dma_tx_phy + (tx_q->cur_tx *
					sizeof(struct dma_desc));
		stmmac_set_tx_tail_ptr(priv, priv->ioaddr, tx_q->tx_tail_addr,
				       qid);
	}

	priv->xstats.rx_pkt_n += total_rx_packets;

	switch (qid) {
	case 0x0:
		priv->xstats.q0_rx_pkt_n += total_rx_packets;
		break;
	case 0x1:
		priv->xstats.q1_rx_pkt_n += total_rx_packets;
		break;
	case 0x2:
		priv->xstats.q2_rx_pkt_n += total_rx_packets;
		break;
	case 0x3:
		priv->xstats.q3_rx_pkt_n += total_rx_packets;
		break;
	case 0x4:
		priv->xstats.q4_rx_pkt_n += total_rx_packets;
		break;
	case 0x5:
		priv->xstats.q5_rx_pkt_n += total_rx_packets;
		break;
	case 0x6:
		priv->xstats.q6_rx_pkt_n += total_rx_packets;
		break;
	case 0x7:
		priv->xstats.q7_rx_pkt_n += total_rx_packets;
		break;
	default:
		break;
	}

	return failure ? budget : (int)total_rx_packets;
}

void stmmac_xsk_free_rx_ring(struct stmmac_rx_queue *rx_q)
{
	struct stmmac_priv *priv = rx_q->priv_data;
	unsigned int entry = rx_q->cur_rx;

	struct stmmac_rx_buffer *buf = &rx_q->buf_pool[entry];

	while (entry != rx_q->next_to_alloc) {
		xsk_umem_fq_reuse(rx_q->xsk_umem, buf->handle);
		entry++;
		buf++;
		if (entry == priv->dma_rx_size) {
			entry = 0;
			buf = rx_q->buf_pool;
		}
	}
}
