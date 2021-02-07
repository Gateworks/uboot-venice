// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#include <common.h>
#include <dm.h>
#include <net.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#include <dm/lists.h>

struct eth_phy_device_priv {
	struct mii_dev *mdio_bus;
	struct udevice *eth_dev;
};

int eth_phy_binds_nodes(struct udevice *eth_dev)
{
	ofnode mdio_node, phy_node;
	const char *node_name;
	int ret;

	mdio_node = dev_read_subnode(eth_dev, "mdio");
	if (!ofnode_valid(mdio_node)) {
		debug("%s: %s mdio subnode not found!", __func__,
		      eth_dev->name);
		return -ENXIO;
	}

	ofnode_for_each_subnode(phy_node, mdio_node) {
		node_name = ofnode_get_name(phy_node);

		debug("* Found child node: '%s'\n", node_name);

		ret = device_bind_driver_to_node(eth_dev,
						 "eth_phy_generic_drv",
						 node_name, phy_node, NULL);
		if (ret) {
			debug("  - Eth phy binding error: %d\n", ret);
			continue;
		}

		debug("  - bound phy device: '%s'\n", node_name);
	}

	return 0;
}

int eth_phy_set_mdio_bus(struct udevice *phy_dev, struct udevice *eth_dev, struct mii_dev *mdio_bus)
{
	struct eth_phy_device_priv *uc_priv = (struct eth_phy_device_priv *)(phy_dev->uclass_priv);

	uc_priv->eth_dev = eth_dev;
	uc_priv->mdio_bus = mdio_bus;

	return 0;
}

struct mii_dev *eth_phy_get_mdio_bus(struct udevice *eth_dev)
{
	struct udevice *phy_dev;
	struct eth_phy_device_priv *uc_priv;

	/* Iterate over UCLASS_ETH_PHY devices looking for a eth_dev match */
	uclass_foreach_dev_probe(UCLASS_ETH_PHY, phy_dev) {
		uc_priv = (struct eth_phy_device_priv *)(phy_dev->uclass_priv);
		if (uc_priv->eth_dev == eth_dev) {
			return uc_priv->mdio_bus;
		}
	}

	return NULL;
}

int eth_phy_get_addr(struct udevice *dev)
{
	struct ofnode_phandle_args phandle_args;
	int reg;

	if (dev_read_phandle_with_args(dev, "phy-handle", NULL, 0, 0,
				       &phandle_args)) {
		debug("Failed to find phy-handle");
		return -ENODEV;
	}

	reg = ofnode_read_u32_default(phandle_args.node, "reg", 0);

	return reg;
}

UCLASS_DRIVER(eth_phy_generic) = {
	.id		= UCLASS_ETH_PHY,
	.name		= "eth_phy_generic",
	.per_device_auto_alloc_size = sizeof(struct eth_phy_device_priv),
};

U_BOOT_DRIVER(eth_phy_generic_drv) = {
	.name		= "eth_phy_generic_drv",
	.id		= UCLASS_ETH_PHY,
};
