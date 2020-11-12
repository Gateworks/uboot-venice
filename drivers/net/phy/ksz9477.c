// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2020
 * Tim Harvey, Gateworks Corporation
 */
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <miiphy.h>
#include <i2c.h>

#define KSZ9477_I2C_SLAVE		0x5f

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

/* MMD */
#define REG_PORT_PHY_MMD_SETUP		0x011A
#define PORT_MMD_OP_MODE_M		0x3
#define PORT_MMD_OP_MODE_S		14
#define PORT_MMD_OP_INDEX		0
#define PORT_MMD_OP_DATA_NO_INCR	1
#define PORT_MMD_OP_DATA_INCR_RW	2
#define PORT_MMD_OP_DATA_INCR_W		3
#define PORT_MMD_DEVICE_ID_M		0x1F
#define MMD_SETUP(mode, dev)		\
	(((u16)(mode) << PORT_MMD_OP_MODE_S) | (dev))
#define REG_PORT_PHY_MMD_INDEX_DATA	0x011C

/* Operation control */
#define REG_SW_OPERATION		0x0300
#define SW_RESET			BIT(1)
#define SW_START			BIT(0)

#define PORT_CTRL_ADDR(port, addr) ((addr) | (((port) + 1) << 12))

struct ksz_phy_priv {
	struct mii_dev *mdio_bus;
	int smi_addr;
	phy_interface_t interface;
	struct udevice *dev;
	int port_max;
	int phy_port_cnt;
	int phy_port_cpu;
	int phy_ports;
};

static inline int ksz_read8(struct udevice *dev, u32 reg, u8 *val)
{
	int ret;

	ret = dm_i2c_read(dev, reg, val, 1);
	dev_dbg(dev, "%s 0x%04x<<0x%02x\n", __func__, reg, *val);

	return ret;
}

static inline int ksz_pread8(struct udevice *dev, int port, int offset, u8 *val)
{
	return ksz_read8(dev, PORT_CTRL_ADDR(port, offset), val);
}

static inline int ksz_write8(struct udevice *dev, u32 reg, u8 val)
{
	dev_dbg(dev, "%s 0x%04x>>0x%02x\n", __func__, reg, val);
	return dm_i2c_write(dev, reg, &val, 1);
}

static inline int ksz_pwrite8(struct udevice *dev, int port, int offset, u8 val)
{
	return ksz_write8(dev, PORT_CTRL_ADDR(port, offset), val);
}

static inline int ksz_write16(struct udevice *dev, u32 reg, u16 val)
{
	u8 buf[2];

	buf[1] = val & 0xff;
	buf[0] = val >> 8;
	dev_dbg(dev, "%s 0x%04x>>0x%04x\n", __func__, reg, val);

	return dm_i2c_write(dev, reg, buf, 2);
}

static inline int ksz_pwrite16(struct udevice *dev, int port, int offset, u16 val)
{
	return ksz_write16(dev, PORT_CTRL_ADDR(port, offset), val);
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

static inline int ksz_pread16(struct udevice *dev, int port, int offset, u16 *val)
{
	return ksz_read16(dev, PORT_CTRL_ADDR(port, offset), val);
}

static inline int ksz_read32(struct udevice *dev, u32 reg, u32 *val)
{
	return dm_i2c_read(dev, reg, (u8 *)val, 4);
}

static inline int ksz_pread32(struct udevice *dev, int port, int offset, u32 *val)
{
	return ksz_read32(dev, PORT_CTRL_ADDR(port, offset), val);
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

static inline int ksz_pwrite32(struct udevice *dev, int port, int offset, u32 val)
{
	return ksz_write32(dev, PORT_CTRL_ADDR(port, offset), val);
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

static void ksz9477_phy_errata_setup(struct udevice *dev, int port)
{
	dev_dbg(dev, "%s P%d\n", __func__, port + 1);

	/* Apply PHY settings to address errata listed in
	 * KSZ9477, KSZ9897, KSZ9896, KSZ9567, KSZ8565
	 * Silicon Errata and Data Sheet Clarification documents:
	 *
	 * Register settings are needed to improve PHY receive performance
	 */
	ksz9477_port_mmd_write(dev, port, 0x01, 0x6f, 0xdd0b);
	ksz9477_port_mmd_write(dev, port, 0x01, 0x8f, 0x6032);
	ksz9477_port_mmd_write(dev, port, 0x01, 0x9d, 0x248c);
	ksz9477_port_mmd_write(dev, port, 0x01, 0x75, 0x0060);
	ksz9477_port_mmd_write(dev, port, 0x01, 0xd3, 0x7777);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x06, 0x3008);
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x08, 0x2001);

	/* Transmit waveform amplitude can be improved
	 * (1000BASE-T, 100BASE-TX, 10BASE-Te)
	 */
	ksz9477_port_mmd_write(dev, port, 0x1c, 0x04, 0x00d0);

	/* Energy Efficient Ethernet (EEE) feature select must
	 * be manually disabled (except on KSZ8565 which is 100Mbit)
	 */
	ksz9477_port_mmd_write(dev, port, 0x07, 0x3c, 0x0000);

	/* Register settings are required to meet data sheet
	 * supply current specifications
	 */
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

static void ksz9477_set_gbit(struct udevice *dev, bool gbit, u8 *data)
{
	if (gbit)
		*data &= ~PORT_MII_NOT_1GBIT;
	else
		*data |= PORT_MII_NOT_1GBIT;
}

static void ksz9477_set_xmii(struct udevice *dev, int mode, u8 *data)
{
	u8 xmii;

	switch (mode) {
	case 0:
		xmii = PORT_MII_SEL;
		break;
	case 1:
		xmii = PORT_RMII_SEL;
		break;
	case 2:
		xmii = PORT_GMII_SEL;
		break;
	default:
		xmii = PORT_RGMII_SEL;
		break;
	}
	*data &= ~PORT_MII_SEL_M;
	*data |= xmii;
}

static void ksz9477_config_leds(struct udevice *dev, int port)
{
	u16 data16;

	// LED config: MMD LED MODE Register (add 0x02, reg 0x00)
	dev_dbg(dev, "%s setting Single-LED mode\n", __func__);
	ksz9477_port_mmd_read(dev, port, 0x02, 0x00, &data16);
#if (IS_ENABLED(KSZ9477_LED_SINGLE_MODE))
	data16 |= BIT(4); // single-led mode
#else
	data16 &= ~BIT(4); // tri-color dual led mode (default)
#endif
	ksz9477_port_mmd_write(dev, port, 0x02, 0x00, data16);
}

static int ksz9477_port_setup(struct phy_device *phydev, int port, bool cpu_port)
{
	struct ksz_phy_priv *priv = phydev->priv;
	struct udevice *dev = priv->dev;
	u8 data8;

	dev_dbg(dev, "%s P%d %s\n", __func__, port + 1, cpu_port ? "cpu" : "");

	/* phy port */
	if (port < priv->phy_port_cnt) {
		ksz9477_phy_errata_setup(dev, port);
		ksz9477_config_leds(dev, port);
	} else { /* cpu port */
		/* configure MAC interface mode */
		ksz_pread8(dev, port, REG_PORT_XMII_CTRL_1, &data8);
		dev_dbg(dev, "%s interface=%d\n", __func__, priv->interface);
		switch (priv->interface) {
		case PHY_INTERFACE_MODE_MII:
			ksz9477_set_xmii(dev, 0, &data8);
			ksz9477_set_gbit(dev, false, &data8);
			phydev->speed = SPEED_100;
			break;
		case PHY_INTERFACE_MODE_RMII:
			ksz9477_set_xmii(dev, 1, &data8);
			ksz9477_set_gbit(dev, false, &data8);
			phydev->speed = SPEED_100;
			break;
		case PHY_INTERFACE_MODE_GMII:
			ksz9477_set_xmii(dev, 2, &data8);
			ksz9477_set_gbit(dev, true, &data8);
			phydev->speed = SPEED_1000;
			break;
		default:
			ksz9477_set_xmii(dev, 3, &data8);
			ksz9477_set_gbit(dev, true, &data8);
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

static int ksz9477_phy_read_indirect(struct mii_dev *bus, int dev, int addr, int reg)
{
	struct phy_device *phydev = (struct phy_device *)bus->priv;
	struct ksz_phy_priv *priv = phydev->priv;
	u16 val = 0xffff;
	int port = phydev->addr;

	ksz_pread16(priv->dev, port, 0x100 + (reg << 1), &val);
	dev_dbg(priv->dev, "%s dev=%d addr=0x%x P%d reg=0x%04x:0x%04x<<0x%04x\n", __func__,
		dev, addr, port + 1, reg, 0x100 + (reg << 1), val);

	return val;
}

static int ksz9477_phy_write_indirect(struct mii_dev *bus, int dev, int addr, int reg, u16 data)
{
	struct phy_device *phydev = (struct phy_device *)bus->priv;
	struct ksz_phy_priv *priv = phydev->priv;
	int port = phydev->addr;

	dev_dbg(priv->dev, "%s dev=%d addr=0x%x P%d reg=0x%04x:%04x>>0x%04x\n", __func__,
		dev, addr, port + 1, reg, 0x100 + (reg << 1), data);
	ksz_pwrite16(priv->dev, port, 0x100 + (reg << 1), data);

	return 0;
}

static int ksz9477_probe(struct phy_device *phydev)
{
	struct udevice *i2cbus, *dev;
	struct mii_dev *bus;
	struct ksz_phy_priv *priv;
	const char *phy_mode = CONFIG_KSZ9477_INTERFACE_MODE;
	int ret;

	debug("%s phy_id=0x%08x\n", __func__, phydev->phy_id);
	/* probe chip */
	ret = uclass_get_device_by_seq(UCLASS_I2C, CONFIG_KSZ9477_I2C_BUSNO, &i2cbus);
	if (ret)
		return ret;
	ret = i2c_get_chip(i2cbus, KSZ9477_I2C_SLAVE, 2, &dev);
	if (ret)
		return ret;

	priv = malloc(sizeof(*priv));
	if (!priv)
		return -ENOMEM;

	memset(priv, 0, sizeof(*priv));
	priv->dev = dev;
	priv->port_max = 7;
	priv->phy_port_cnt = 5;
	priv->phy_port_cpu = CONFIG_KSZ9477_CPU_PORT;
	priv->phy_ports = CONFIG_KSZ9477_PHY_PORTS;
	priv->interface = phy_get_interface_by_name(phy_mode);
	dev_dbg(dev, "ports=0x%02x: CPU:P%d %s\n", priv->phy_ports,
		priv->phy_port_cpu + 1, phy_mode);
	if (!priv->phy_port_cpu) {
		dev_err(dev, "no cpu port defined\n");
		ret = -EINVAL;
		goto err;
	}
	if (!priv->phy_ports) {
		dev_err(dev, "no phy ports defined\n");
		ret = -EINVAL;
		goto err;
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

	/* This device uses I2C for PHY register acces. Allocate a bus to handle */
	bus = mdio_alloc();
	if (!bus) {
		ret = -ENOMEM;
		goto err;
	}
	priv->mdio_bus = phydev->bus;
	priv->smi_addr = phydev->addr;
	bus->read = ksz9477_phy_read_indirect;
	bus->write = ksz9477_phy_write_indirect;
	bus->priv = phydev;
	snprintf(bus->name, sizeof(bus->name), "%s-mii", dev->name);

	phydev->bus = bus;
	phydev->priv = priv;

	return 0;

err:
	free(priv);
	return ret;
}

static int ksz9477_phy_config(struct phy_device *phydev)
{
	struct ksz_phy_priv *priv = phydev->priv;
	int i;
	u8 data8;
	int ret, res;

	dev_dbg(priv->dev, "%s cpu=P%d\n", __func__, priv->phy_port_cpu + 1);
	res = ksz9477_port_setup(phydev, priv->phy_port_cpu, true);
	if (res)
		return res;

	for (i = 0; i < priv->phy_port_cnt; i++) {
		if ((1 << i) & priv->phy_ports) {
			phydev->addr = i;

			if (i == priv->phy_port_cpu)
				continue;
			res = ksz9477_port_setup(phydev, i, false);
			if (res)
				return res;

			res = phy_reset(phydev);
			if (res) {
				printf("Error resetting P%d PHY %d\n", i + 1, res);
				continue;
			}
			res = genphy_config_aneg(phydev);
			if (res) {
				printf("Error setting P%d autoneg: %d\n", i + 1, res);
				return res;
			}

			/* return success if any PHY succeeds */
			ret = 0;
		}
	}

	/* start switch */
	ksz_read8(priv->dev, REG_SW_OPERATION, &data8);
	data8 |= SW_START;
	ksz_write8(priv->dev, REG_SW_OPERATION, data8);

	return ret;
}

static int ksz9477_phy_is_connected(struct phy_device *phydev)
{
	struct ksz_phy_priv *priv = phydev->priv;
	struct udevice *dev = priv->dev;
	u16 data16;
	int port = phydev->addr;

	ksz_pread16(dev, port, 0x100 + (MII_BMSR << 1), &data16);
	dev_dbg(dev, "%s P%d link=%d\n", __func__, port + 1, (data16 & BMSR_LSTATUS) ? 1 : 0);

	return (data16 & BMSR_LSTATUS) ? 1 : 0;
}

static int ksz9477_phy_startup(struct phy_device *phydev)
{
	struct ksz_phy_priv *priv = phydev->priv;
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
			if (!ksz9477_phy_is_connected(phydev))
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

static struct phy_driver ksz9477_driver = {
	.name = "Microchip KSZ9477S",
	.uid = 0x00947700,
	.mask = 0xffffff00,
	.features = PHY_GBIT_FEATURES,
	.probe = ksz9477_probe,
	.config = ksz9477_phy_config,
	.startup = ksz9477_phy_startup,
	.shutdown = &genphy_shutdown,
};

static struct phy_driver ksz9567_driver = {
	.name = "Microchip KSZ9567R",
	.uid = 0x00956700,
	.mask = 0xffffff00,
	.features = PHY_GBIT_FEATURES,
	.probe = ksz9477_probe,
	.config = ksz9477_phy_config,
	.startup = ksz9477_phy_startup,
	.shutdown = &genphy_shutdown,
};

static struct phy_driver ksz9897_driver = {
	.name = "Microchip KSZ9897S",
	.uid = 0x00989700,
	.mask = 0xffffff00,
	.features = PHY_GBIT_FEATURES,
	.probe = ksz9477_probe,
	.config = ksz9477_phy_config,
	.startup = ksz9477_phy_startup,
	.shutdown = &genphy_shutdown,
};

int phy_ksz9477_init(void)
{
	phy_register(&ksz9477_driver);
	phy_register(&ksz9567_driver);
	phy_register(&ksz9897_driver);

	return 0;
}

/*
 * Overload weak get_phy_id definition since we need non-standard functions
 * to read PHY registers
 */
int get_phy_id(struct mii_dev *miibus, int smi_addr, int devad, u32 *phy_id)
{
	struct udevice *dev, *bus;
	int ret;

	/* probe chip */
	ret = uclass_get_device_by_seq(UCLASS_I2C, CONFIG_KSZ9477_I2C_BUSNO, &bus);
	if (ret)
		return ret;
	ret = i2c_get_chip(bus, KSZ9477_I2C_SLAVE, 2, &dev);
	if (ret)
		return ret;

	/* read chip id */
	ret = ksz_read32(dev, REG_CHIP_ID0__1, phy_id);
	if (ret)
		return ret;
	*phy_id = __swab32(*phy_id);

	return 0;
}
