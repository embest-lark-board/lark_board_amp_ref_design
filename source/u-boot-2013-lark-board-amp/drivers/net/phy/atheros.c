/*
 * Atheros PHY drivers
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Copyright 2011 Freescale Semiconductor, Inc.
 * author Andy Fleming
 *
 */
#include <phy.h>
#include <config.h>
#include <common.h>

static int ar8021_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x3D47);

	return 0;
}

static struct phy_driver AR8021_driver =  {
	.name = "AR8021",
	.uid = 0x4dd040,
	.mask = 0xfffff0,
	.features = PHY_GBIT_FEATURES,
	.config = ar8021_config,
	.startup = genphy_startup,
	.shutdown = genphy_shutdown,
};

#ifdef CONFIG_PHY_AR8035

static int ar8035_config(struct phy_device *phydev)
{
	int regval;

	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x0007);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);
	regval = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, (regval|0x0018));

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	regval = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, (regval|0x0100));

	genphy_config_aneg(phydev);

	phy_reset(phydev);

	return 0;
}

static int ar8035_startup(struct phy_device *phydev)
{
	unsigned phy_status;
	
	genphy_update_link(phydev);
	
	phy_status = phy_read(phydev, MDIO_DEVAD_NONE, 0x11);
	printf("phy_status = 0x%x\n", phy_status);

	if (phy_status & (0x1 << 13))
		phydev->duplex = DUPLEX_FULL;
	else
		phydev->duplex = DUPLEX_HALF;

	if ((phy_status & (0x3 << 14)) == (0x2 << 14))
		phydev->speed = SPEED_1000;
	else if ((phy_status & (0x3 << 14)) == (0x1 << 14))
		phydev->speed = SPEED_100;
	else if ((phy_status & (0x3 << 14)) == (0x0 << 14))
		phydev->speed = SPEED_10;
	
	return 0;
}


static struct phy_driver AR8035_driver =  {
	.name = "AR8035",
	.uid = 0x4dd072,
	.mask = 0xffffff,
	.features = PHY_BASIC_FEATURES,//PHY_GBIT_FEATURES,
	.config = ar8035_config,//genphy_config,
	.startup = ar8035_startup,//genphy_startup,
	.shutdown = genphy_shutdown,
};

#endif

int phy_atheros_init(void)
{
	phy_register(&AR8021_driver);

#ifdef CONFIG_PHY_AR8035
	printf("AR8035 phy registering...\n");
	phy_register(&AR8035_driver);
#endif

	return 0;
}


