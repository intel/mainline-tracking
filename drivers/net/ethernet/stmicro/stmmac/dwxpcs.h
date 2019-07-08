/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2019, Intel Corporation.
 * DWC Ethernet Physical Coding Sublayer
 */
#ifndef __DW_XPCS_H__
#define __DW_XPCS_H__

/* XPCS MII MMD Device Addresses */
#define XPCS_MDIO_MII_MMD	MDIO_MMD_VEND2

/* MII MMD registers offsets */
#define MDIO_MII_MMD_DIGITAL_CTRL_1	0x8000	/* Digital Control 1 */
#define MDIO_MII_MMD_AN_CTRL		0x8001	/* AN Control */
#define MDIO_MII_MMD_AN_STAT		0x8002	/* AN Status */

/* MII MMD SR AN Advertisement & Link Partner Ability are slightly
 * different from MII_ADVERTISEMENT & MII_LPA in below fields:
 */
#define MDIO_MII_MMD_HD			BIT(6)	/* Half duplex */
#define MDIO_MII_MMD_FD			BIT(5)	/* Full duplex */
#define MDIO_MII_MMD_PSE_SHIFT		7	/* Pause Ability shift */
#define MDIO_MII_MMD_PSE		GENMASK(8, 7)	/* Pause Ability */
#define MDIO_MII_MMD_PSE_NO		0x0
#define MDIO_MII_MMD_PSE_ASYM		0x1
#define MDIO_MII_MMD_PSE_SYM		0x2
#define MDIO_MII_MMD_PSE_BOTH		0x3

/* Automatic Speed Mode Change for MAC side SGMII AN */
#define MDIO_MII_MMD_DIGI_CTRL_1_MAC_AUTO_SW	BIT(9)

/* MII MMD AN Control defines */
#define MDIO_MII_MMD_AN_CTRL_TX_CONFIG_SHIFT	3 /* TX Config shift */
#define AN_CTRL_TX_CONF_PHY_SIDE_SGMII		0x1 /* PHY side SGMII mode */
#define AN_CTRL_TX_CONF_MAC_SIDE_SGMII		0x0 /* MAC side SGMII mode */
#define MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT	1  /* PCS Mode shift */
#define MDIO_MII_MMD_AN_CTRL_PCS_MD	GENMASK(2, 1) /* PCS Mode */
#define AN_CTRL_PCS_MD_C37_1000BASEX	0x0	/* C37 AN for 1000BASE-X */
#define AN_CTRL_PCS_MD_C37_SGMII	0x2	/* C37 AN for SGMII */
#define MDIO_MII_MMD_AN_CTRL_AN_INTR_EN	BIT(0)	/* AN Complete Intr Enable */

/* MII MMD AN Status defines for SGMII AN Status */
#define AN_STAT_SGMII_AN_CMPLT		BIT(0)	/* AN Complete Intr */
#define AN_STAT_SGMII_AN_FD		BIT(1)	/* Full Duplex */
#define AN_STAT_SGMII_AN_SPEED_SHIFT	2	/* AN Speed shift */
#define AN_STAT_SGMII_AN_SPEED		GENMASK(3, 2)	/* AN Speed */
#define AN_STAT_SGMII_AN_10MBPS		0x0	/* 10 Mbps */
#define AN_STAT_SGMII_AN_100MBPS	0x1	/* 100 Mbps */
#define AN_STAT_SGMII_AN_1000MBPS	0x2	/* 1000 Mbps */
#define AN_STAT_SGMII_AN_LNKSTS		BIT(4)	/* Link Status */

#endif /* __DW_XPCS_H__ */
