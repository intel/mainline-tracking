// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2019, Intel Corporation.
 * DWC Ethernet Physical Coding Sublayer
 */
#include <linux/bitops.h>
#include <linux/mdio.h>
#include "dwxpcs.h"
#include "stmmac.h"

/* DW xPCS mdiobus_read and mdiobus_write helper functions */
#define xpcs_read(dev, reg) \
	mdiobus_read(priv->mii, xpcs_phy_addr, \
		     MII_ADDR_C45 | (reg) | \
		     ((dev) << MII_DEVADDR_C45_SHIFT))
#define xpcs_write(dev, reg, val) \
	mdiobus_write(priv->mii, xpcs_phy_addr, \
		      MII_ADDR_C45 | (reg) | \
		      ((dev) << MII_DEVADDR_C45_SHIFT), val)

static void dw_xpcs_init(struct net_device *ndev, int pcs_mode)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr;
	int phydata;

	xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	if (pcs_mode == AN_CTRL_PCS_MD_C37_SGMII) {
		/* For AN for SGMII mode, the settings are :-
		 * 1) VR_MII_AN_CTRL Bit(2:1)[PCS_MODE] = 10b (SGMII AN)
		 * 2) VR_MII_AN_CTRL Bit(3) [TX_CONFIG] = 0b (MAC side SGMII)
		 *    DW xPCS used with DW EQoS MAC is always MAC
		 *    side SGMII.
		 * 3) VR_MII_AN_CTRL Bit(0) [AN_INTR_EN] = 1b (AN Interrupt
		 *    enabled)
		 * 4) VR_MII_DIG_CTRL1 Bit(9) [MAC_AUTO_SW] = 1b (Automatic
		 *    speed mode change after SGMII AN complete)
		 * Note: Since it is MAC side SGMII, there is no need to set
		 *	 SR_MII_AN_ADV. MAC side SGMII receives AN Tx Config
		 *	 from PHY about the link state change after C28 AN
		 *	 is completed between PHY and Link Partner.
		 */
		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL);
		phydata &= ~MDIO_MII_MMD_AN_CTRL_PCS_MD;
		phydata |= MDIO_MII_MMD_AN_CTRL_AN_INTR_EN |
			   (AN_CTRL_PCS_MD_C37_SGMII <<
			    MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT &
			    MDIO_MII_MMD_AN_CTRL_PCS_MD) |
			   (AN_CTRL_TX_CONF_MAC_SIDE_SGMII <<
			    MDIO_MII_MMD_AN_CTRL_TX_CONFIG_SHIFT);
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL, phydata);

		phydata = xpcs_read(XPCS_MDIO_MII_MMD,
				    MDIO_MII_MMD_DIGITAL_CTRL_1);
		phydata |= MDIO_MII_MMD_DIGI_CTRL_1_MAC_AUTO_SW;
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_DIGITAL_CTRL_1,
			   phydata);
	} else {
		/* For AN for 1000BASE-X mode, the settings are :-
		 * 1) VR_MII_AN_CTRL Bit(2:1)[PCS_MODE] = 00b (1000BASE-X C37)
		 * 2) VR_MII_AN_CTRL Bit(0) [AN_INTR_EN] = 1b (AN Interrupt
		 *    enabled)
		 * 3) SR_MII_AN_ADV Bit(6)[FD] = 1b (Full Duplex)
		 *    Note: Half Duplex is rarely used, so don't advertise.
		 * 4) SR_MII_AN_ADV Bit(8:7)[PSE] = 11b (Sym & Asym Pause)
		 */
		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL);
		phydata &= ~MDIO_MII_MMD_AN_CTRL_PCS_MD;
		phydata |= MDIO_MII_MMD_AN_CTRL_AN_INTR_EN |
			   (AN_CTRL_PCS_MD_C37_1000BASEX <<
			    MDIO_MII_MMD_AN_CTRL_PCS_MD_SHIFT &
			    MDIO_MII_MMD_AN_CTRL_PCS_MD);
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_CTRL, phydata);

		phydata = xpcs_read(XPCS_MDIO_MII_MMD, MII_ADVERTISE);
		phydata |= MDIO_MII_MMD_FD |
			   (MDIO_MII_MMD_PSE_BOTH << MDIO_MII_MMD_PSE_SHIFT);
		xpcs_write(XPCS_MDIO_MII_MMD, MII_ADVERTISE, phydata);
	}
}

static void dw_xpcs_ctrl_ane(struct net_device *ndev, bool ane,
			     bool loopback)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr;
	int phydata;

	xpcs_phy_addr = priv->plat->xpcs_phy_addr;
	phydata = xpcs_read(XPCS_MDIO_MII_MMD, MII_BMCR);

	if (ane)
		phydata |= (BMCR_ANENABLE | BMCR_ANRESTART);

	if (loopback)
		phydata |= BMCR_LOOPBACK;

	xpcs_write(XPCS_MDIO_MII_MMD, MII_BMCR, phydata);
}

static void dw_xpcs_get_adv_lp(struct net_device *ndev,
			       struct rgmii_adv *adv_lp,
			       int pcs_mode)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int xpcs_phy_addr;
	int value;

	xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	/* AN Advertisement Ability */
	value = xpcs_read(XPCS_MDIO_MII_MMD, MII_ADVERTISE);

	if (value & MDIO_MII_MMD_FD)
		adv_lp->duplex = DUPLEX_FULL;
	if (value & MDIO_MII_MMD_HD)
		adv_lp->duplex = DUPLEX_HALF;
	adv_lp->pause = (u32)((value & MDIO_MII_MMD_PSE) >>
			      MDIO_MII_MMD_PSE_SHIFT);

	/* Link Partner Ability - 1000BASE-X only*/
	if (pcs_mode == AN_CTRL_PCS_MD_C37_1000BASEX) {
		value = xpcs_read(XPCS_MDIO_MII_MMD, MII_LPA);
		if (value & MDIO_MII_MMD_FD)
			adv_lp->lp_duplex = DUPLEX_FULL;
		if (value & MDIO_MII_MMD_HD)
			adv_lp->lp_duplex = DUPLEX_HALF;
		adv_lp->lp_pause = (u32)((value & MDIO_MII_MMD_PSE) >>
					 MDIO_MII_MMD_PSE_SHIFT);
	}
}

static void dw_xpcs_get_linkstatus(struct net_device *ndev,
				   u16 an_stat,
				   struct stmmac_extra_stats *x)
{
	/* Check the SGMII AN link status */
	if (an_stat & AN_STAT_SGMII_AN_LNKSTS) {
		int speed_value;

		x->pcs_link = 1;

		speed_value = ((an_stat & AN_STAT_SGMII_AN_SPEED) >>
				AN_STAT_SGMII_AN_SPEED_SHIFT);
		if (speed_value == AN_STAT_SGMII_AN_1000MBPS)
			x->pcs_speed = SPEED_1000;
		else if (speed_value == AN_STAT_SGMII_AN_100MBPS)
			x->pcs_speed = SPEED_100;
		else
			x->pcs_speed = SPEED_10;

		if (an_stat & AN_STAT_SGMII_AN_FD)
			x->pcs_duplex = 1;
		else
			x->pcs_duplex = 0;

		netdev_info(ndev, "Link is Up - %d/%s\n", (int)x->pcs_speed,
			    x->pcs_duplex ? "Full" : "Half");
	} else {
		x->pcs_link = 0;
		netdev_info(ndev, "Link is Down\n");
	}
}

static int dw_xpcs_irq_status(struct net_device *ndev,
			      struct stmmac_extra_stats *x,
			      int pcs_mode)
{
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret = IRQ_NONE;
	int xpcs_phy_addr;
	int an_stat;

	xpcs_phy_addr = priv->plat->xpcs_phy_addr;

	/* AN status */
	an_stat = xpcs_read(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_STAT);

	if (an_stat & AN_STAT_SGMII_AN_CMPLT) {
		x->irq_pcs_ane_n++;

		if (pcs_mode == AN_CTRL_PCS_MD_C37_SGMII) {
			dw_xpcs_get_linkstatus(ndev, an_stat, x);
		} else {
			/* For 1000BASE-X AN, DW xPCS does not have register
			 * to read the link state of 1000BASE-X C37 AN and
			 * since 1000BASE-X is always 1000Mbps and FD, we
			 * just set the default link here.
			 */
			x->pcs_link = 1;
			x->pcs_duplex = 1;
			x->pcs_speed = SPEED_1000;
		}

		/* Clear C37 AN complete status by writing zero */
		xpcs_write(XPCS_MDIO_MII_MMD, MDIO_MII_MMD_AN_STAT, 0);
		ret = IRQ_HANDLED;
	}

	return ret;
}

const struct stmmac_xpcs_ops xpcs_ops = {
	.xpcs_init = dw_xpcs_init,
	.xpcs_ctrl_ane = dw_xpcs_ctrl_ane,
	.xpcs_get_adv_lp = dw_xpcs_get_adv_lp,
	.xpcs_irq_status = dw_xpcs_irq_status,
};
