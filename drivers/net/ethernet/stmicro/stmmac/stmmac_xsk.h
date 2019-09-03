/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2018 Intel Corporation. */

#ifndef __STMMAC_XSK_H__
#define __STMMAC_XSK_H__

#define STMMAC_XDP_PASS		0
#define STMMAC_XDP_DROP		BIT(0)
#define STMMAC_XDP_TX		BIT(1)
#define STMMAC_XDP_REDIRECT	BIT(2)
#define STMMAC_DEFAULT_TX_WORK	256

#define queue_is_xdp(qid) (priv->is_xdp[(qid)])
#define enable_queue_xdp(qid) { priv->is_xdp[(qid)] = true; }
#define disable_queue_xdp(qid) { priv->is_xdp[(qid)] = false; }

static inline bool stmmac_enabled_xdp(struct stmmac_priv *priv)
{
	return !!priv->xdp_prog;
}

int stmmac_xdp_xmit_queue(struct stmmac_priv *priv, u32 queue,
			  struct xdp_frame *xdpf);
struct xdp_umem *stmmac_xsk_umem(struct stmmac_priv *priv, u16 qid);
int stmmac_xsk_umem_setup(struct stmmac_priv *priv, struct xdp_umem *umem,
			  u16 qid);
void stmmac_zca_free(struct zero_copy_allocator *alloc, unsigned long handle);
int stmmac_run_xdp_zc(struct stmmac_priv *priv, struct stmmac_rx_queue *rx_q,
		      struct xdp_buff *xdp);
void stmmac_alloc_rx_buffers_slow_zc(struct stmmac_rx_queue *rx_q,
				     u16 cleaned_count);
bool stmmac_alloc_rx_buffers_fast_zc(struct stmmac_rx_queue *rx_q,
				     u16 cleaned_count);
struct stmmac_rx_buffer *stmmac_get_rx_buffer_zc(struct stmmac_rx_queue *rx_q,
						 unsigned int size);
void stmmac_reuse_rx_buffer_zc(struct stmmac_rx_queue *rx_q,
			       struct stmmac_rx_buffer *obi);
int stmmac_rx_zc(struct stmmac_priv *priv, const int budget, u32 qid);
void stmmac_xsk_free_rx_ring(struct stmmac_rx_queue *rx_q);
void stmmac_xsk_free_tx_ring(struct stmmac_tx_queue *tx_q);
int stmmac_xsk_async_xmit(struct net_device *dev, u32 qid);
bool stmmac_xdp_xmit_zc(struct stmmac_tx_queue *tx_q, unsigned int budget);

#endif /* __STMMAC_XSK_H__ */
