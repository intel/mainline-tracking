/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2018 Intel Corporation. */

#ifndef __STMMAC_XSK_H__
#define __STMMAC_XSK_H__

#define queue_is_xdp(qid) (priv->is_xdp[(qid)])
#define enable_queue_xdp(qid) { priv->is_xdp[(qid)] = true; }
#define disable_queue_xdp(qid) { priv->is_xdp[(qid)] = false; }

#endif /* __STMMAC_XSK_H__ */
