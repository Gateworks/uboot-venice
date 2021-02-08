// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2020
 * Tim Harvey, Gateworks Corporation
 */
#include <dm.h>
#include <dm/device_compat.h>
#include <eth_phy.h>
#include <linux/delay.h>
#include <miiphy.h>
#include <i2c.h>

#include <asm-generic/gpio.h>

#define REG_CHIP_ID0__1			0x0000

/* Port Control */
#define REG_PORT_XMII_CTRL_1		0x0301
#define PORT_MII_NOT_1GBIT		BIT(6)
#define PORT_MII_SEL_EDGE		BIT(5)
#define PORT_RGMII_ID_IG_ENABLE		BIT(4)
#define PORT_RGMII_ID_EG_ENABLE		BIT(3)
#define PORT_MII_MAC_MODE		BIT(2)
#define PORT_MII_SEL_M			0x3
#define PORT_RGMII_SEL			0x0
#define PORT_RMII_SEL			0x1
#define PORT_GMII_SEL			0x2
#define PORT_MII_SEL			0x3

#define REG_PORT_MSTP_STATE		0x0b04
#define PORT_TX_ENABLE			BIT(2)
#define PORT_RX_ENABLE			BIT(1)
#define PORT_LEARN_DISABLE		BIT(0)

/* MMD */
#define REG_PORT_PHY_MMD_SETUP		0x011A
#define PORT_MMD_OP_MODE_M		0x3
#define PORT_MMD_OP_MODE_S		14
#define PORT_MMD_OP_INDEX		0
#define PORT_MMD_OP_DATA_NO_INCR	1
#define PORT_MMD_OP_DATA_INCR_RW	2
#define PORT_MMD_OP_DATA_INCR_W		3
#define PORT_MMD_DEVICE_ID_M		0x1F
#define MMD_SETUP(mode, dev)		(((u16)(mode) << PORT_MMD_OP_MODE_S) | (dev))
#define REG_PORT_PHY_MMD_INDEX_DATA	0x011C

/* Operation control */
#define REG_SW_OPERATION		0x0300
#define SW_RESET			BIT(1)
#define SW_START			BIT(0)

#define PORT_CTRL_ADDR(port, addr) ((addr) | (((port) + 1) << 12))

struct ksz_phy_priv {
	struct udevice *dev;
	struct mii_dev *bus;
	phy_interface_t interface;
	int phy_port_cnt;
	int phy_port_cpu;
	int phy_ports;
};

static inline int ksz_read8(struct udevice *dev, u32 reg, u8 *val)
{
	int ret = dm_i2c_read(dev, reg, val, 1);

	dev_dbg(dev, "%s 0x%04x<<0x%02x\n", __func__, reg, *val);

	return ret;
}

static inline int ksz_pread8(struct udevice *dev, int port, int reg, u8 *val)
{
	return ksz_read8(dev, PORT_CTRL_ADDR(port, reg), val);
}

static inline int ksz_write8(struct udevice *dev, u32 reg, u8 val)
{
	dev_dbg(dev, "%s 0x%04x>>0x%02x\n", __func__, reg, val);
	return dm_i2c_write(dev, reg, &val, 1);
}

static inline int ksz_pwrite8(struct udevice *dev, int port, int reg, u8 val)
{
	return ksz_write8(dev, PORT_CTRL_ADDR(port, reg), val);
}

static inline int ksz_write16(struct udevice *dev, u32 reg, u16 val)
{
	u8 buf[2];

	buf[1] = val & 0xff;
	buf[0] = val >> 8;
	dev_dbg(dev, "%s 0x%04x>>0x%04x\n", __func__, reg, val);

	return dm_i2c_write(dev, reg, buf, 2);
}

static inline int ksz_pwrite16(struct udevice *dev, int port, int reg, u16 val)
{
	return ksz_write16(dev, PORT_CTRL_ADDR(port, reg), val);
}

static inline int ksz_read16(struct udevice *dev, u32 reg, u16 *val)
{
	u8 buf[2];
	int ret;

	ret = dm_i2c_read(dev, reg, buf, 2);
	*val = (buf[0] << 8) | buf[1];
	dev_dbg(dev, "%s 0x%04x<<0x%04x\n", __func__, reg, *val);

	return ret;
}

static inline int ksz_pread16(struct udevice *dev, int port, int reg, u16 *val)
{
	return ksz_read16(dev, PORT_CTRL_ADDR(port, reg), val);
}

static inline int ksz_read32(struct udevice *dev, u32 reg, u32 *val)
{
	return dm_i2c_read(dev, reg, (u8 *)val, 4);
}

static inline int ksz_pread32(struct udevice *dev, int port, int reg, u32 *val)
{
	return ksz_read32(dev, PORT_CTRL_ADDR(port, reg), val);
}

static inline int ksz_write32(struct udevice *dev, u32 reg, u32 val)
{
	u8 buf[4];

	buf[3] = val & 0xff;
	buf[2] = (val >> 24) & 0xff;
	buf[1] = (val >> 16) & 0xff;
	buf[0] = (val >> 8) & 0xff;
	dev_dbg(dev, "%s 0x%04x>>0x%04x\n", __func__, reg, val);

	return dm_i2c_write(dev, reg, buf, 4);
}

static inline int ksz_pwrite32(struct udevice *dev, int port, int reg, u32 val)
{
	return ksz_write32(dev, PORT_CTRL_ADDR(port, reg), val);
}

static void ksz9477_port_mmd_read(struct udevice *dev, int port, u8 addr, u16 reg, u16 *val)
{
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_SETUP, MMD_SETUP(PORT_MMD_OP_INDEX, addr));
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_INDEX_DATA, reg);
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_SETUP, MMD_SETUP(PORT_MMD_OP_DATA_NO_INCR, addr));
	ksz_pread16(dev, port, REG_PORT_PHY_MMD_INDEX_DATA, val);
	dev_dbg(dev, "%s  P%d 0x%02x:0x%04x<<0x%04x\n", __func__, port + 1, addr, reg, *val);
}

static void ksz9477_port_mmd_write(struct udevice *dev, int port, u8 addr, u16 reg, u16 val)
{
	dev_dbg(dev, "%s P%d 0x%02x:0x%04x>>0x%04x\n", __func__, port + 1, addr, addr, val);
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_SETUP, MMD_SETUP(PORT_MMD_OP_INDEX, addr));
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_INDEX_DATA, addr);
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_SETUP, MMD_SETUP(PORT_MMD_OP_DATA_NO_INCR, addr));
	ksz_pwrite16(dev, port, REG_PORT_PHY_MMD_INDEX_DATA, val);
}

/* Apply PHY settings to address errata listed in KSZ9477, KSZ9897, KSZ9896, KSZ9567
 * Silicon Errata and Data Sheet Clarification documents
 */
static void ksz9477_phy_errata_setup(struct udevice *dev, int port)
{
	dev_dbg(dev, "%s P%d\n", __func__, port + 1);

	/* Register settings are needed to improve PHY receive performance */
	ksz9477_port_mmd_write(dev, port, 0x01, 0x6f, 0xdd0b);
	ksz9477_port_mmd_write(dev, port, 0x01, 0x8f, 0x6032);
	ksz9477_port_mmd_write(dev, port, 0x01, 0x9d, 0x248c);
	ksz9477_port_mmd_write(dev, port, 0x01, 0x75, 0x0060);
	ksz9477_port_mmd_write(dev, port, 0x01, 0xd3, 0x7777);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x06, 0x3008);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x08, 0x2001);

	/* Transmit waveform amplitude can be improved (1000BASE-T, 100BASE-TX, 10BASE-Te) */
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x04, 0x00d0);

	/* Energy Efficient Ethernet (EEE) feature select must be manually disabled */
	ksz9477_port_mmd_write(dev, port, 0x07, 0x3c, 0x0000);

	/* Register settings are required to meet data sheet supply current specifications */
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x13, 0x6eff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x14, 0xe6ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x15, 0x6eff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x16, 0xe6ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x17, 0x00ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x18, 0x43ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x19, 0xc3ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x1a, 0x6fff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x1b, 0x07ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x1c, 0x0fff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x1d, 0xe7ff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x1e, 0xefff);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x20, 0xeeee);
}

static int ksz9477_port_setup(struct phy_device *phydev, int port, bool cpu_port)
{
	struct ksz_phy_priv *priv = (struct ksz_phy_priv *)phydev->bus->priv;
	struct udevice *dev = priv->dev;
	u8 data8;
	u16 data16;

	dev_dbg(dev, "%s P%d %s\n", __func__, port + 1, cpu_port ? "cpu" : "");

	if (port < priv->phy_port_cnt) {
		/* phy port: config errata and leds */
		ksz9477_phy_errata_setup(dev, port);
		ksz9477_port_mmd_read(dev, port, 0x02, 0x00, &data16);
#if (IS_ENABLED(KSZ9477_LED_SINGLE_MODE))
		data16 |= BIT(4); // single-led mode
#else
		data16 &= ~BIT(4); // tri-color dual led mode (default)
#endif
		dev_dbg(dev, "%s setting %s-LED mode\n", __func__,
			(data16 & BIT(4)) ? "single" : "dual");
		ksz9477_port_mmd_write(dev, port, 0x02, 0x00, data16);
	} else {
		/* cpu port: configure MAC interface mode */
		ksz_pread8(dev, port, REG_PORT_XMII_CTRL_1, &data8);
		dev_dbg(dev, "%s P%d cpu interface=%d\n", __func__, port + 1, priv->interface);
		switch (priv->interface) {
		case PHY_INTERFACE_MODE_MII:
			data8 &= ~PORT_MII_SEL_M;
			data8 |= PORT_MII_SEL;
			data8 |= PORT_MII_NOT_1GBIT;
			phydev->speed = SPEED_100;
			break;
		case PHY_INTERFACE_MODE_RMII:
			data8 &= ~PORT_MII_SEL_M;
			data8 |= PORT_RMII_SEL;
			data8 |= PORT_MII_NOT_1GBIT;
			phydev->speed = SPEED_100;
			break;
		case PHY_INTERFACE_MODE_GMII:
			data8 &= ~PORT_MII_SEL_M;
			data8 |= PORT_GMII_SEL;
			data8 &= ~PORT_MII_NOT_1GBIT;
			phydev->speed = SPEED_1000;
			break;
		default:
			data8 &= ~PORT_MII_SEL_M;
			data8 |= PORT_RGMII_SEL;
			data8 &= ~PORT_MII_NOT_1GBIT;
			data8 &= ~PORT_RGMII_ID_IG_ENABLE;
			data8 &= ~PORT_RGMII_ID_EG_ENABLE;
			if (priv->interface == PHY_INTERFACE_MODE_RGMII_ID ||
			    priv->interface == PHY_INTERFACE_MODE_RGMII_RXID)
				data8 |= PORT_RGMII_ID_IG_ENABLE;
			if (priv->interface == PHY_INTERFACE_MODE_RGMII_ID ||
			    priv->interface == PHY_INTERFACE_MODE_RGMII_TXID)
				data8 |= PORT_RGMII_ID_EG_ENABLE;
			phydev->speed = SPEED_1000;
			break;
		}
		phydev->duplex = DUPLEX_FULL;
		phydev->link = 1;
		ksz_write8(dev, PORT_CTRL_ADDR(port, REG_PORT_XMII_CTRL_1), data8);
	}

	return 0;
}

static int ksz9477_phy_mii_read(struct mii_dev *bus, int dev, int addr, int reg)
{
	struct ksz_phy_priv *priv = (struct ksz_phy_priv *)bus->priv;
	u16 val = 0xffff;
	int port = dev;

	ksz_pread16(priv->dev, port, 0x100 + (reg << 1), &val);
	dev_dbg(priv->dev, "%s dev=%d addr=0x%x P%d reg=0x%04x:0x%04x<<0x%04x\n", __func__,
		dev, addr, port + 1, reg, 0x100 + (reg << 1), val);

	return val;
}

static int ksz9477_phy_mii_write(struct mii_dev *bus, int dev, int addr, int reg, u16 data)
{
	struct ksz_phy_priv *priv = (struct ksz_phy_priv *)bus->priv;
	int port = dev;

	dev_dbg(priv->dev, "%s dev=%d addr=0x%x P%d reg=0x%04x:%04x>>0x%04x\n", __func__,
		dev, addr, port + 1, reg, 0x100 + (reg << 1), data);
	ksz_pwrite16(priv->dev, port, 0x100 + (reg << 1), data);

	return 0;
}

static int ksz9477_phy_config(struct phy_device *phydev)
{
	struct ksz_phy_priv *priv = (struct ksz_phy_priv *)phydev->bus->priv;
	int i;
	u8 data8;
	int ret;

	dev_dbg(priv->dev, "%s\n", __func__);
	ret = ksz9477_port_setup(phydev, priv->phy_port_cpu, true);
	if (ret)
		return ret;

	for (i = 0; i < priv->phy_port_cnt; i++) {
		if ((1 << i) & priv->phy_ports) {
			phydev->addr = i;

			if (i == priv->phy_port_cpu)
				continue;
			ret = ksz9477_port_setup(phydev, i, false);
			if (ret)
				return ret;

			ret = phy_reset(phydev);
			if (ret) {
				printf("Error resetting P%d PHY %d\n", i + 1, ret);
				continue;
			}
			ret = genphy_config_aneg(phydev);
			if (ret) {
				printf("Error setting P%d autoneg: %d\n", i + 1, ret);
				return ret;
			}
		} else {
			/* disable port */
			ksz_pread8(priv->dev, i, REG_PORT_MSTP_STATE, &data8);
			data8 &= ~(PORT_TX_ENABLE | PORT_RX_ENABLE | PORT_LEARN_DISABLE);
			ksz_pwrite8(priv->dev, i, REG_PORT_MSTP_STATE, data8);
		}
	}

	/* start switch */
	ksz_read8(priv->dev, REG_SW_OPERATION, &data8);
	data8 |= SW_START;
	ksz_write8(priv->dev, REG_SW_OPERATION, data8);

	return 0;
}

static int ksz9477_phy_startup(struct phy_device *phydev)
{
	struct ksz_phy_priv *priv = (struct ksz_phy_priv *)phydev->bus->priv;
	int i;
	int link = 0;
	int speed = phydev->speed;
	int duplex = phydev->duplex;
	int ret;

	dev_dbg(priv->dev, "%s\n", __func__);
	for (i = 0; i < priv->phy_port_cnt; i++) {
		if ((1 << i) & priv->phy_ports) {
			phydev->addr = i;
			/* skip if not linked to avoid timeout waiting for aneg */
			if (!(ksz9477_phy_mii_read(phydev->bus, i, 0, MII_BMSR) & BMSR_LSTATUS))
				continue;
			ret = genphy_update_link(phydev);
			if (ret < 0)
				continue;
			ret = genphy_parse_link(phydev);
			if (ret < 0)
				continue;
			dev_dbg(priv->dev, "%s P%d link=%d speed=%d duplex=%d\n", __func__,
				i + 1, phydev->link, phydev->speed, phydev->duplex);
			link = (link || phydev->link);
		}
	}
	phydev->link = link;

	/* Restore CPU interface speed and duplex after it was changed for other ports */
	phydev->speed = speed;
	phydev->duplex = duplex;
	dev_dbg(priv->dev, "%s link=%d speed=%d duplex=%d\n", __func__, link, speed, duplex);

	return 0;
}

static struct phy_driver ksz9477_phy_driver = {
	.name = "ksz9477-phy",
	.uid = 0x00221631,
	.mask = 0xffffff00,
	.features = PHY_GBIT_FEATURES,
	.config = &ksz9477_phy_config,
	.startup = &ksz9477_phy_startup,
	.shutdown = &genphy_shutdown,
};

static int ksz9477_probe(struct udevice *dev)
{
	struct ksz_phy_priv *priv = dev_get_priv(dev);
	ofnode ports_node, port_node;
	struct ofnode_phandle_args eth_phandle;
	const char *phy_mode, *label;
	struct udevice *eth_dev = NULL;
	struct gpio_desc reset_gpios;
	u32 reset_assert_us;
	u32 reset_deassert_us;
	u32 id;
	u8 data8;
	int i, ret;

	dev_dbg(dev, "%s %s\n", __func__, dev->name);
	ret = i2c_set_chip_offset_len(dev, 2);
	if (ret) {
		printf("i2c_set_chip_offset_len failed: %d\n", ret);
		return ret;
	}

	/* default config */
	priv->dev = dev;
	priv->phy_port_cnt = 5;
	priv->interface = PHY_INTERFACE_MODE_NONE;

	/* get port config from dt */
	if (IS_ENABLED(CONFIG_DM_GPIO)) {
		gpio_request_by_name(dev, "reset-gpios", 0, &reset_gpios, GPIOD_IS_OUT);
		reset_assert_us = dev_read_u32_default(dev, "reset-assert-us", 0);
		reset_deassert_us = dev_read_u32_default(dev, "reset-deassert-us", 0);
		if (dm_gpio_is_valid(&reset_gpios)) {
			dm_gpio_set_value(&reset_gpios, 1);
			udelay(reset_assert_us);
			dm_gpio_set_value(&reset_gpios, 0);
			udelay(reset_deassert_us);
		}
	}
	ports_node = ofnode_find_subnode(dev_ofnode(dev), "ports");
	if (!ofnode_valid(ports_node)) {
		dev_err(dev, "no ports defined\n");
		return -EINVAL;
	}
	ofnode_for_each_subnode(port_node, ports_node) {
		i = ofnode_read_u32_default(port_node, "reg", -1);
		ret = ofnode_parse_phandle_with_args(port_node, "ethernet", NULL,
						     0, 0, &eth_phandle);
		if (!ret) {
			priv->phy_port_cpu = i;
			uclass_get_device_by_ofnode(UCLASS_ETH, eth_phandle.node, &eth_dev);
		} else {
			priv->phy_ports |= (1 << i);
		}
		label = ofnode_read_string(port_node, "label");
		phy_mode = ofnode_read_string(port_node, "phy-mode");
		priv->interface = phy_get_interface_by_name(phy_mode);
		dev_dbg(dev, "%s:P%d %s %s\n", ofnode_get_name(port_node),
			i, label, phy_mode ? phy_mode : "");
	}
	/* allow env var to override phy port config */
	priv->phy_ports = env_get_hex("ksz_portmask", priv->phy_ports);
	if (!priv->phy_port_cpu) {
		dev_err(dev, "no cpu port defined\n");
		return -EINVAL;
	}
	if (!eth_dev) {
		dev_err(dev, "no ethernet device defined\n");
		return -EINVAL;
	}
	if (!priv->phy_ports) {
		dev_err(dev, "no phy ports defined\n");
		return -EINVAL;
	}
	switch (priv->interface) {
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_RMII:
	case PHY_INTERFACE_MODE_GMII:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
		break;
	default:
		dev_err(dev, "invalid phy_mode\n");
		return -EINVAL;
	}
	dev_dbg(dev, "%s:cpu:P%d ports:0x%x eth_dev:%s\n", __func__,
		priv->phy_port_cpu + 1, priv->phy_ports, eth_dev->name);

	/* chip level reset */
	ksz_read8(priv->dev, REG_SW_OPERATION, &data8);
	data8 |= SW_RESET;
	ksz_write8(priv->dev, REG_SW_OPERATION, data8);

	/* read chip id */
	ret = ksz_read32(dev, REG_CHIP_ID0__1, &id);
	if (ret)
		return ret;
	id = __swab32(id);
	dev_dbg(dev, "%s id=0x%08x\n", __func__, id);
	switch (id & 0xffffff00) {
	case 0x00947700:
		puts("Microchip KSZ9477S\n");
		break;
	case 0x00956700:
		puts("Microchip KSZ9567R\n");
		break;
	case 0x00989700:
		puts("Microchip KSZ9897S\n");
		break;
	default:
		dev_err(dev, "invalid chip id: 0x%08x\n", id);
		return -EINVAL;
	}

	/* register an mii_bus to translate phy read/write to the various ports */
	priv->bus = mdio_alloc();
	if (!priv->bus) {
		printf("mdio_alloc failed\n");
		return -ENOMEM;
	}
	priv->bus->read = ksz9477_phy_mii_read;
	priv->bus->write = ksz9477_phy_mii_write;
	priv->bus->priv = dev->priv;
	sprintf(priv->bus->name, "%s-mii", dev->name);
	ret = mdio_register(priv->bus);
	if (ret) {
		printf("mdio_register failed\n");
		free(priv->bus);
		return ret;
	}

	/* bind the cpu_dev to our dev */
	eth_phy_set_mdio_bus(dev, eth_dev, priv->bus);

	phy_register(&ksz9477_phy_driver);

	return 0;
};

static const struct udevice_id ksz9477_i2creg_ids[] = {
	{ .compatible = "microchip,ksz9897" },
	{ .compatible = "microchip,ksz9477" },
	{ .compatible = "microchip,ksz9567" },
	{ }
};

U_BOOT_DRIVER(ksz9477) = {
	.name		= "ksz9477",
	.id		= UCLASS_ETH_PHY,
	.of_match	= ksz9477_i2creg_ids,
	.probe		= ksz9477_probe,
	.priv_auto_alloc_size = sizeof(struct ksz_phy_priv),
};

