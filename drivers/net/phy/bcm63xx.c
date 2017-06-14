/*
 * Copyright (C) 2017 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * Derived from linux/drivers/net/phy/bcm63xx.c:
 *	Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <phy.h>

/* MII Interrupt register */
#define MII_IR_REG	0x1a

static int bcm63xx_startup(struct phy_device *phydev)
{
	int ret;

	/* Read the Status (2x to make sure link is right) */
	ret = genphy_update_link(phydev);
	if (ret)
		return ret;

	return genphy_parse_link(phydev);
}

static int bcm63xx_config(struct phy_device *phydev)
{
	genphy_config_aneg(phydev);

	phy_reset(phydev);

	return 0;
}

static struct phy_driver bcm63xx_driver1 = {
	.name = "Broadcom BCM63XX (1)",
	.uid = 0x00406000,
	.mask = 0xfffffc00,
	.features = (PHY_BASIC_FEATURES | SUPPORTED_Pause),
	.config = &bcm63xx_config,
	.startup = &bcm63xx_startup,
	.shutdown = &genphy_shutdown,
};

static struct phy_driver bcm63xx_driver2 = {
	.name = "Broadcom BCM63XX (2)",
	.uid = 0x002bdc00,
	.mask = 0xfffffc00,
	.features = (PHY_BASIC_FEATURES | SUPPORTED_Pause),
	.config = &bcm63xx_config,
	.startup = &bcm63xx_startup,
	.shutdown = &genphy_shutdown,
};

int phy_bcm63xx_init(void)
{
	phy_register(&bcm63xx_driver1);
	phy_register(&bcm63xx_driver2);

	return 0;
}
