/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2018 Intel Corporation. */

#ifndef __STMMAC_XSK_H__
#define __STMMAC_XSK_H__

#define STMMAC_XDP_PASS		0
#define STMMAC_XDP_DROP		BIT(0)
#define STMMAC_XDP_TX		BIT(1)
#define STMMAC_XDP_REDIRECT	BIT(2)

#define queue_is_xdp(qid) (priv->is_xdp[(qid)])
#define enable_queue_xdp(qid) { priv->is_xdp[(qid)] = true; }
#define disable_queue_xdp(qid) { priv->is_xdp[(qid)] = false; }

static inline bool stmmac_enabled_xdp(struct stmmac_priv *priv)
{
	return !!priv->xdp_prog;
}

#endif /* __STMMAC_XSK_H__ */
