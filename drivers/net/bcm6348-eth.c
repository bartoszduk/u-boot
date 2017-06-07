/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * Derived from linux/drivers/net/ethernet/broadcom/bcm63xx_enet.c:
 *	Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dma.h>
#include <miiphy.h>
#include <net.h>
#include <phy.h>
#include <reset.h>
#include <asm/io.h>


#define ETH_TX_WATERMARK		32
#define ETH_MAX_MTU_SIZE		1518

#define ETH_TIMEOUT_US			1000

/* ETH Receiver Configuration register */
#define ETH_RXCFG_REG			0x00
#define ETH_RXCFG_ENFLOW_SHIFT		5
#define ETH_RXCFG_ENFLOW_MASK		(1 << ETH_RXCFG_ENFLOW_SHIFT)

/* ETH Receive Maximum Length register */
#define ETH_RXMAXLEN_REG		0x04
#define ETH_RXMAXLEN_SHIFT		0
#define ETH_RXMAXLEN_MASK		(0x7ff << ETH_RXMAXLEN_SHIFT)

/* ETH Transmit Maximum Length register */
#define ETH_TXMAXLEN_REG		0x08
#define ETH_TXMAXLEN_SHIFT		0
#define ETH_TXMAXLEN_MASK		(0x7ff << ETH_TXMAXLEN_SHIFT)

/* MII Status/Control register */
#define MII_SC_REG			0x10
#define MII_SC_MDCFREQDIV_SHIFT		0
#define MII_SC_MDCFREQDIV_MASK		(0x7f << MII_SC_MDCFREQDIV_SHIFT)

/* MII Data register */
#define MII_DAT_REG			0x14
#define MII_DAT_DATA_SHIFT		0
#define MII_DAT_DATA_MASK		(0xffff << MII_DAT_DATA_SHIFT)
#define MII_DAT_TA_SHIFT		16
#define MII_DAT_TA_MASK			(0x3 << MII_DAT_TA_SHIFT)
#define MII_DAT_REG_SHIFT		18
#define MII_DAT_REG_MASK		(0x1f << MII_DAT_REG_SHIFT)
#define MII_DAT_PHY_SHIFT		23
#define MII_DAT_PHY_MASK		(0x1f << MII_DAT_PHY_SHIFT)
#define MII_DAT_OP_SHIFT		28
#define MII_DAT_OP_WRITE		(0x5 << MII_DAT_OP_SHIFT)
#define MII_DAT_OP_READ			(0x6 << MII_DAT_OP_SHIFT)

/* ETH Interrupts Mask register */
#define ETH_IRMASK_REG			0x18

/* ETH Interrupts register */
#define ETH_IR_REG			0x1c
#define ETH_IR_MII_SHIFT		0
#define ETH_IR_MII_MASK			(1 << ETH_IR_MII_SHIFT)

/* ETH Control register */
#define ETH_CTL_REG			0x2c
#define ETH_CTL_ENABLE_SHIFT		0
#define ETH_CTL_ENABLE_MASK		(1 << ETH_CTL_ENABLE_SHIFT)
#define ETH_CTL_DISABLE_SHIFT		1
#define ETH_CTL_DISABLE_MASK		(1 << ETH_CTL_DISABLE_SHIFT)
#define ETH_CTL_RESET_SHIFT		2
#define ETH_CTL_RESET_MASK		(1 << ETH_CTL_RESET_SHIFT)
#define ETH_CTL_EPHY_SHIFT		3
#define ETH_CTL_EPHY_MASK		(1 << ETH_CTL_EPHY_SHIFT)

/* ETH Transmit Control register */
#define ETH_TXCTL_REG			0x30
#define ETH_TXCTL_FD_SHIFT		0
#define ETH_TXCTL_FD_MASK		(1 << ETH_TXCTL_FD_SHIFT)

/* ETH Transmit Watermask register */
#define ETH_TXWMARK_REG			0x34
#define ETH_TXWMARK_WM_SHIFT		0
#define ETH_TXWMARK_WM_MASK		(0x3f << ETH_TXWMARK_WM_SHIFT)

/* MIB Control register */
#define MIB_CTL_REG			0x38
#define MIB_CTL_RDCLEAR_SHIFT		0
#define MIB_CTL_RDCLEAR_MASK		(1 << MIB_CTL_RDCLEAR_SHIFT)

/* ETH Perfect Match registers */
#define ETH_PM_CNT			4
#define ETH_PML_REG(x)			(0x58 + (x) * 0x8)
#define ETH_PMH_REG(x)			(0x5c + (x) * 0x8)
#define ETH_PMH_VALID_SHIFT		16
#define ETH_PMH_VALID_MASK		(1 << ETH_PMH_VALID_SHIFT)

/* MIB Counters registers */
#define MIB_REG_CNT			55
#define MIB_REG(x)			(0x200 + (x) * 4)

/* ETH data */
struct bcm6348_eth_priv {
	void __iomem *base;
	/* DMA */
	struct dma rx_dma;
	struct dma tx_dma;
	/* PHY */
	int phyid;
	phy_interface_t phyif;
};

DECLARE_GLOBAL_DATA_PTR;

static void bcm6348_eth_mac_disable(struct bcm6348_eth_priv *priv)
{
	uint32_t timeout = ETH_TIMEOUT_US;

	/* disable emac */
	setbits_be32(priv->base + ETH_CTL_REG, ETH_CTL_DISABLE_MASK);

	/* wait until emac is disabled */
	do {
		uint32_t val;

		val = readl_be(priv->base + ETH_CTL_REG);
		if (!(val & ETH_CTL_DISABLE_MASK))
			break;

		udelay(1);
	} while(timeout--);
}

static void bcm6348_eth_mac_enable(struct bcm6348_eth_priv *priv)
{
	setbits_be32(priv->base + ETH_CTL_REG, ETH_CTL_ENABLE_MASK);
}

static void bcm6348_eth_mac_reset(struct bcm6348_eth_priv *priv)
{
	uint32_t timeout = ETH_TIMEOUT_US;

	/* reset emac */
	writel_be(ETH_CTL_RESET_MASK, priv->base + ETH_CTL_REG);
	wmb();

	/* wait until emac is reset */
	do {
		uint32_t val = readl_be(priv->base + ETH_CTL_REG);

		if (!(val & ETH_CTL_RESET_MASK))
			break;

		udelay(1);
	} while(timeout--);
}

static int bcm6348_eth_free_pkt(struct udevice *dev, uchar *packet,
				int length)
{
#if DEBUG
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);

	printf("%s: priv=%p packet=%p length=%d\n", __func__, priv, packet, length);
#endif

	return 0;
}

static int bcm6348_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);

#if DEBUG
	printf("%s: priv=%p packetp=%p\n", __func__, priv, packetp);
#endif

	return dma_receive(&priv->rx_dma, (void *)packetp);
}

static int bcm6348_eth_send(struct udevice *dev, void *packet, int length)
{
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);

#if DEBUG
	printf("%s: priv=%p packet=%p length=%d\n", __func__, priv, packet, length);
#endif

	return dma_send(&priv->tx_dma, (void **)packet, length);
}

static int bcm6348_eth_adjust_link(struct udevice *dev, struct phy_device *phydev)
{
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);

	/* check link */
#if 0
	if (!phydev->link) {
		printf("%s: no link (%s)\n", __func__, phydev->dev->name);
		return -EINVAL;
	}
#endif

	/* mac duplex parameters */
	if (phydev->duplex)
		setbits_be32(priv->base + ETH_TXCTL_REG, ETH_TXCTL_FD_MASK);
	else
		clrbits_be32(priv->base + ETH_TXCTL_REG, ETH_TXCTL_FD_MASK);

	/* rx flow control (pause frame handling) */
	if (phydev->pause)
		setbits_be32(priv->base + ETH_RXCFG_REG, ETH_RXCFG_ENFLOW_MASK);
	else
		clrbits_be32(priv->base + ETH_RXCFG_REG, ETH_RXCFG_ENFLOW_MASK);

	printf("%s: phy is %s with %d/%s\n", dev->name, phydev->dev->name, phydev->speed, (phydev->duplex) ? "full" : "half");

	return 0;
}

static int bcm6348_eth_start(struct udevice *dev)
{
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);
	struct mii_dev *mii;
	struct phy_device *phydev = NULL;
	int i;

	printf("%s: priv=%p\n", __func__, priv);

	/* zero mib counters */
	for (i = 0; i < MIB_REG_CNT; i++)
		writel_be(0, MIB_REG(i));

	/* get mii bus */
	mii = miiphy_get_dev_by_name(dev->name);

	/* find & connect phy */
	phydev = phy_connect(mii, priv->phyid, dev, priv->phyif);
	if (!phydev) {
		printf("%s: could not attach to phy\n", __func__);
		return 0;
	}

	/* wait for phy to complete reset */
	mdelay(1000);

	/* configure supported modes */
	phydev->supported = (SUPPORTED_10baseT_Half |
			     SUPPORTED_10baseT_Full |
			     SUPPORTED_100baseT_Half |
			     SUPPORTED_100baseT_Full |
			     SUPPORTED_Autoneg |
			     SUPPORTED_Pause |
			     SUPPORTED_MII);
	phydev->advertising = phydev->supported;

	/* start and update link status of phy */
	phy_startup(phydev);

#if 1
	/* adjust mac with phy link status */
	int ret = bcm6348_eth_adjust_link(dev, phydev);
	if (ret)
		return ret;
#endif

	/* enable rx flow control */
	setbits_be32(priv->base + ETH_RXCFG_REG, ETH_RXCFG_ENFLOW_MASK);

	/* set max rx/tx length */
	writel_be((ETH_MAX_MTU_SIZE << ETH_RXMAXLEN_SHIFT) & ETH_RXMAXLEN_MASK, priv->base + ETH_RXMAXLEN_REG);
	writel_be((ETH_MAX_MTU_SIZE << ETH_TXMAXLEN_SHIFT) & ETH_TXMAXLEN_MASK, priv->base + ETH_TXMAXLEN_REG);

	/* set correct transmit fifo watermark */
	writel_be((ETH_TX_WATERMARK << ETH_TXWMARK_WM_SHIFT) & ETH_TXWMARK_WM_MASK, priv->base + ETH_TXWMARK_REG);

	/* enable emac */
	bcm6348_eth_mac_enable(priv);

	/* clear interrupts */
	writel_be(0, priv->base + ETH_IRMASK_REG);

	/* enable dma rx channel */
	dma_enable(&priv->rx_dma);

	return 0;
}

static void bcm6348_eth_stop(struct udevice *dev)
{
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);

	printf("%s: priv=%p\n", __func__, priv);

	/* disable emac */
	bcm6348_eth_mac_disable(priv);
}

static int bcm6348_eth_write_hwaddr(struct udevice *dev)
{
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_platdata(dev);
	bool running = false;

	/* check if emac is running */
	if (readl_be(priv->base + ETH_CTL_REG) & ETH_CTL_ENABLE_MASK)
		running = true;

	/* disable emac */
	if (running)
		bcm6348_eth_mac_disable(priv);

	/* set mac address */
	writel_be(pdata->enetaddr[2] | pdata->enetaddr[3] << 8 |
		  pdata->enetaddr[4] << 16 | pdata->enetaddr[5] << 24,
		  priv->base + ETH_PML_REG(0));
	writel_be(pdata->enetaddr[1] | pdata->enetaddr[0] << 8 |
		  ETH_PMH_VALID_MASK, priv->base + ETH_PMH_REG(0));

	/* enable emac */
	if (running)
		bcm6348_eth_mac_enable(priv);

	return 0;
}

static const struct eth_ops bcm6348_eth_ops = {
	.free_pkt = bcm6348_eth_free_pkt,
	.recv = bcm6348_eth_recv,
	.send = bcm6348_eth_send,
	.start = bcm6348_eth_start,
	.stop = bcm6348_eth_stop,
	.write_hwaddr = bcm6348_eth_write_hwaddr,
};

static const struct udevice_id bcm6348_eth_ids[] = {
	{ .compatible = "brcm,bcm6348-enet", },
	{ /* sentinel */ }
};

static int bcm6348_mdio_op(void __iomem *base, uint32_t data)
{
	uint32_t timeout = ETH_TIMEOUT_US;

	/* make sure mii interrupt status is cleared */
	writel_be(ETH_IR_MII_MASK, base + ETH_IR_REG);

	/* issue mii op */
	writel_be(data, base + MII_DAT_REG);
	wmb();

	/* wait for mii interrupt */
	do {
		uint32_t val = readl_be(base + ETH_IR_REG);
		rmb();

		if (val & ETH_IR_MII_MASK)
			break;

		udelay(1);
	} while(timeout--);

	writel_be(ETH_IR_MII_MASK, base + ETH_IR_REG);

	return !timeout;
}

static int bcm6348_mdio_read(struct mii_dev *bus, int addr, int devaddr,
			     int reg)
{
	void __iomem *base = bus->priv;
	uint32_t val;

	val = MII_DAT_OP_READ;
	val |= (reg << MII_DAT_REG_SHIFT) & MII_DAT_REG_MASK;
	val |= (0x2 << MII_DAT_TA_SHIFT) & MII_DAT_TA_MASK;
	val |= (addr << MII_DAT_PHY_SHIFT) & MII_DAT_PHY_MASK;

	if (bcm6348_mdio_op(base, val)) {
		printf("%s: bcm6348_mdio_op timeout\n", __func__);
		return -EINVAL;
	}

	val = readl_be(base + MII_DAT_REG) & MII_DAT_DATA_MASK;
	rmb();
	val >>= MII_DAT_DATA_SHIFT;

	printf("%s: addr=%d reg=%x val=%x\n", __func__, addr, reg, val);

	return val;
}

static int bcm6348_mdio_write(struct mii_dev *bus, int addr, int dev_addr,
			      int reg, u16 value)
{
	void __iomem *base = bus->priv;
	uint32_t val;

	val = MII_DAT_OP_WRITE;
	val |= (reg << MII_DAT_REG_SHIFT) & MII_DAT_REG_MASK;
	val |= (0x2 << MII_DAT_TA_SHIFT) & MII_DAT_TA_MASK;
	val |= (addr << MII_DAT_PHY_SHIFT) & MII_DAT_PHY_MASK;
	val |= (value << MII_DAT_DATA_SHIFT) & MII_DAT_DATA_MASK;

	printf("%s: addr=%d reg=%x val=%x write=%x\n", __func__, addr, reg, val, value);

	return bcm6348_mdio_op(base, val);
}

static int bcm6348_mdio_init(const char *name, void __iomem *base,
			     int phyid)
{
	struct mii_dev *bus;

	printf("%s: name=%s base=%p phyid=%d\n", __func__, name, base, phyid);

	bus = mdio_alloc();
	if (!bus) {
		error("%s: failed to allocate MDIO bus\n", __func__);
		return -ENOMEM;
	}

	bus->read = bcm6348_mdio_read;
	bus->write = bcm6348_mdio_write;
	bus->priv = base;
	snprintf(bus->name, sizeof(bus->name), "%s", name);

	/* only probe bus where we think the PHY is, because
	 * the mdio read operation return 0 instead of 0xffff
	 * if a slave is not present on hw */
	bus->phy_mask = ~(1 << phyid);

	printf("%s: phy_mask=%x\n", __func__, bus->phy_mask);

	return mdio_register(bus);
}

static int bcm6348_eth_probe(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct bcm6348_eth_priv *priv = dev_get_priv(dev);
	void *blob = (void *)gd->fdt_blob;
	int node = dev_of_offset(dev);
	const char *phy_mode;
	fdt_addr_t addr;
	int phy_node, ret, i;

	/* get base address */
	addr = dev_get_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	/* get phy mode */
	pdata->phy_interface = -1;
	phy_mode = fdt_getprop(blob, node, "phy-mode", NULL);
	if (phy_mode)
		pdata->phy_interface = phy_get_interface_by_name(phy_mode);
	if (pdata->phy_interface == -1) {
		error("%s: invalid phy interface '%s'\n", __func__, phy_mode);
		return -EINVAL;
	}
	priv->phyif = pdata->phy_interface;

	/* get dma channels */
	ret = dma_get_by_name(dev, "tx", &priv->tx_dma);
	if (ret)
		return -EINVAL;

	ret = dma_get_by_name(dev, "rx", &priv->rx_dma);
	if (ret)
		return -EINVAL;

	/* try to enable clocks */
	for (i = 0; ; i++) {
		struct clk clk;
		int ret;

		ret = clk_get_by_index(dev, i, &clk);
		if (ret < 0)
			break;
		if (clk_enable(&clk))
			printf("failed to enable clock %d\n", i);
		clk_free(&clk);
	}

	/* try to perform resets */
	for (i = 0; ; i++) {
		struct reset_ctl reset;
		int ret;

		ret = reset_get_by_index(dev, i, &reset);
		if (ret < 0)
			break;
		if (reset_deassert(&reset))
			printf("failed to deassert reset %d\n", i);
		reset_free(&reset);
	}

	/* get phy */
	phy_node = fdtdec_lookup_phandle(blob, node, "phy");
	if (phy_node > 0)
		priv->phyid = fdtdec_get_int(blob, phy_node, "reg", 0);
	else
		return -ENODEV;

	/* get base addr */
	priv->base = ioremap(addr, 0);
	pdata->iobase = (phys_addr_t) priv->base;

	/* disable emac */
	bcm6348_eth_mac_disable(priv);

	/* reset emac */
	bcm6348_eth_mac_reset(priv);

	/* select correct mii interface */
	if (fdtdec_get_bool(blob, phy_node, "brcm,internal-phy"))
		clrbits_be32(priv->base + ETH_CTL_REG, ETH_CTL_EPHY_MASK);
	else
		setbits_be32(priv->base + ETH_CTL_REG, ETH_CTL_EPHY_MASK);

	/* turn on mdc clock */
	writel_be(0x1f << MII_SC_MDCFREQDIV_SHIFT, priv->base + MII_SC_REG);

	/* set mib counters to not clear when read */
	clrbits_be32(priv->base + MIB_CTL_REG, MIB_CTL_RDCLEAR_MASK);

	/* initialize perfect match registers */
	for (i = 0; i < ETH_PM_CNT; i++) {
		writel_be(0, priv->base + ETH_PML_REG(i));
		writel_be(0, priv->base + ETH_PMH_REG(i));
	}

	/* init mii bus */
	ret = bcm6348_mdio_init(dev->name, priv->base, priv->phyid);
	if (ret)
		return ret;

	return 0;
}

U_BOOT_DRIVER(bcm6348_eth) = {
	.name = "bcm6348_eth",
	.id = UCLASS_ETH,
	.of_match = bcm6348_eth_ids,
	.ops = &bcm6348_eth_ops,
	.platdata_auto_alloc_size = sizeof(struct eth_pdata),
	.priv_auto_alloc_size = sizeof(struct bcm6348_eth_priv),
	.probe = bcm6348_eth_probe,
};
