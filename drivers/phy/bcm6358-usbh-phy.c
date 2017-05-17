/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * Derived from linux/arch/mips/bcm63xx/usb-common.c:
 *	Copyright 2008 Maxime Bizon <mbizon@freebox.fr>
 *	Copyright 2013 Florian Fainelli <florian@openwrt.org>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <generic-phy.h>
#include <power-domain.h>
#include <reset.h>
#include <asm/io.h>
#include <dm/device.h>

#define USBH_INVALID_REG	0xff

/* USBH PLL Control register */
#define USBH_PLL_IDDQ_PWRDN	BIT(9)
#define USBH_PLL_PWRDN_DELAY	BIT(10)

/* USBH Setup register */
#define USBH_SETUP_IOC	BIT(4)
#define USBH_SETUP_IPP	BIT(5)

/* USBH Swap Control register */
#define USBH_SWAP_OHCI_DATA	BIT(0)
#define USBH_SWAP_OHCI_ENDIAN	BIT(1)
#define USBH_SWAP_EHCI_DATA	BIT(3)
#define USBH_SWAP_EHCI_ENDIAN	BIT(4)

struct bcm6358_usbh_hw {
	uint8_t reg_pll_ctl;
	uint8_t reg_setup_ctl;
	uint8_t reg_swap_ctl;
	uint8_t reg_tp_ctl;
	uint32_t pll_clr;
	uint32_t pll_set;
	uint32_t setup_clr;
	uint32_t setup_set;
	uint32_t tpc_val;
};

struct bcm6358_usbh_priv {
	const struct bcm6358_usbh_hw *hw;
	void __iomem *regs;
};

static bool bcm6358_usbh_has_reg(uint8_t reg) {
	if (reg != USBH_INVALID_REG)
		return true;
	else
		return false;
}

static int bcm6358_usbh_exit(struct phy *phy)
{
	struct bcm6358_usbh_priv *priv = dev_get_priv(phy->dev);
	const struct bcm6358_usbh_hw *hw = priv->hw;

	/* set setup config */
	if (bcm6358_usbh_has_reg(hw->reg_setup_ctl))
		clrsetbits_be32(priv->regs + hw->reg_setup_ctl, hw->setup_set,
				hw->setup_clr);

	return 0;
}

static int bcm6358_usbh_init(struct phy *phy)
{
	struct bcm6358_usbh_priv *priv = dev_get_priv(phy->dev);
	const struct bcm6358_usbh_hw *hw = priv->hw;

	/* configure to work in native cpu endian */
	clrsetbits_be32(priv->regs + hw->reg_swap_ctl,
			USBH_SWAP_EHCI_ENDIAN | USBH_SWAP_OHCI_ENDIAN,
			USBH_SWAP_EHCI_DATA | USBH_SWAP_OHCI_DATA);

	/* set setup config */
	if (bcm6358_usbh_has_reg(hw->reg_setup_ctl))
		clrsetbits_be32(priv->regs + hw->reg_setup_ctl, hw->setup_clr,
				hw->setup_set);

	/* set test port control */
	if (hw->tpc_val)
		writel_be(hw->tpc_val, priv->regs + hw->reg_tp_ctl);

	return 0;
}

static int bcm6358_usbh_power_off(struct phy *phy)
{
	struct bcm6358_usbh_priv *priv = dev_get_priv(phy->dev);
	const struct bcm6358_usbh_hw *hw = priv->hw;

	/* disable pll control */
	if (bcm6358_usbh_has_reg(hw->reg_pll_ctl))
		clrsetbits_be32(priv->regs + hw->reg_pll_ctl, hw->pll_set,
				hw->pll_clr);

	return 0;
}

static int bcm6358_usbh_power_on(struct phy *phy)
{
	struct bcm6358_usbh_priv *priv = dev_get_priv(phy->dev);
	const struct bcm6358_usbh_hw *hw = priv->hw;

	/* enable pll control */
	if (bcm6358_usbh_has_reg(hw->reg_pll_ctl))
		clrsetbits_be32(priv->regs + hw->reg_pll_ctl, hw->pll_clr,
				hw->pll_set);

	return 0;
}

static struct phy_ops bcm6358_usbh_ops = {
	.exit = bcm6358_usbh_exit,
	.init = bcm6358_usbh_init,
	.power_off = bcm6358_usbh_power_off,
	.power_on = bcm6358_usbh_power_on,
};

static const struct bcm6358_usbh_hw bcm6328_hw = {
	.reg_pll_ctl = 0x18,
	.reg_setup_ctl = 0x28,
	.reg_swap_ctl = 0x1c,
	.reg_tp_ctl = 0x14,
	.pll_clr = USBH_PLL_IDDQ_PWRDN | USBH_PLL_PWRDN_DELAY,
	.pll_set = 0,
	.setup_set = USBH_SETUP_IOC,
	.setup_clr = 0,
	.tpc_val = 0,
};

static const struct bcm6358_usbh_hw bcm6358_hw = {
	.reg_pll_ctl = USBH_INVALID_REG,
	.reg_setup_ctl = USBH_INVALID_REG,
	.reg_swap_ctl = 0x00,
	.reg_tp_ctl = 0x24,
	.pll_clr = 0,
	.pll_set = 0,
	.setup_set = 0,
	.setup_clr = 0,
	.tpc_val = 0x1c0020,
};

static const struct bcm6358_usbh_hw bcm63268_hw = {
	.reg_pll_ctl = 0x18,
	.reg_setup_ctl = 0x28,
	.reg_swap_ctl = 0x1c,
	.reg_tp_ctl = 0x14,
	.pll_clr = USBH_PLL_IDDQ_PWRDN | USBH_PLL_PWRDN_DELAY,
	.pll_set = 0,
	.setup_set = USBH_SETUP_IOC,
	.setup_clr = USBH_SETUP_IPP,
	.tpc_val = 0,
};

static const struct udevice_id bcm6358_usbh_ids[] = {
	{
		.compatible = "brcm,bcm6328-usbh",
		.data = (ulong)&bcm6328_hw,
	}, {
		.compatible = "brcm,bcm6358-usbh",
		.data = (ulong)&bcm6358_hw,
	}, {
		.compatible = "brcm,bcm63268-usbh",
		.data = (ulong)&bcm63268_hw,
	}, { /* sentinel */ }
};

static int bcm6358_usbh_probe(struct udevice *dev)
{
	struct bcm6358_usbh_priv *priv = dev_get_priv(dev);
	const struct bcm6358_usbh_hw *hw =
		(const struct bcm6358_usbh_hw *)dev_get_driver_data(dev);
#if defined(CONFIG_POWER_DOMAIN)
	struct power_domain pwr_dom;
#endif
	struct reset_ctl rst_ctl;
	struct clk clk;
	fdt_addr_t addr;
	fdt_size_t size;
	int ret;

	addr = dev_get_addr_size_index(dev, 0, &size);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->regs = ioremap(addr, size);
	priv->hw = hw;

	/* perform reset */
	ret = reset_get_by_index(dev, 0, &rst_ctl);
	if (ret < 0)
		return ret;

	/* enable usbh clock */
	ret = clk_get_by_name(dev, "usbh", &clk);
	if (!ret) {
		clk_enable(&clk);
		clk_free(&clk);
	}

#if defined(CONFIG_POWER_DOMAIN)
	/* enable power domain */
	ret = power_domain_get(dev, &pwr_dom);
	if (!ret) {
		power_domain_on(&pwr_dom);
		power_domain_free(&pwr_dom);
 	}
#endif

	/* reset done */
	reset_deassert(&rst_ctl);
	reset_free(&rst_ctl);

	/* enable usb_ref clock */
	ret = clk_get_by_name(dev, "usb_ref", &clk);
	if (!ret) {
		clk_enable(&clk);
		clk_free(&clk);
	}

	mdelay(100);

	return 0;
}

U_BOOT_DRIVER(bcm6358_usbh) = {
	.name = "bcm6358-usbh",
	.id = UCLASS_PHY,
	.of_match = bcm6358_usbh_ids,
	.ops = &bcm6358_usbh_ops,
	.priv_auto_alloc_size = sizeof(struct bcm6358_usbh_priv),
	.probe = bcm6358_usbh_probe,
};
