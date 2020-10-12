// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Gateworks Corporation
 */

#include <common.h>
#include <miiphy.h>
#include <netdev.h>

#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>

#include "gsc.h"

DECLARE_GLOBAL_DATA_PTR;

int board_phys_sdram_size(phys_size_t *size)
{
	int ddr_size = readl(M4_BOOTROM_BASE_ADDR);

	if (ddr_size == 0x4) {
		*size = 0x100000000;
// TODO: mmc and fec drivers broken with 4GiB DRAM (1GB base + 4GB overflows u32)
// clamp at 3GiB to work-around for now
*size = 0xc0000000;
	} else if (ddr_size == 0x3) {
		*size = 0xc0000000;
	} else if (ddr_size == 0x2) {
		*size = 0x80000000;
	} else if (ddr_size == 0x1) {
		*size = 0x40000000;
	} else {
		printf("Unknown DDR type!!!\n");
		*size = 0x40000000;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_FEC_MXC)
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	unsigned short val;
	const char *ethmac;
	char env[32];
	int ret, i;
	uint8_t enetaddr[6];

	/* Set mac addrs */
	i = 0;
	do {
		if (i)
			sprintf(env, "eth%daddr", i);
		else
			sprintf(env, "ethaddr");
		ethmac = env_get(env);
		if (!ethmac) {
			ret = gsc_getmac(i, enetaddr);
			if (!ret)
				eth_env_set_enetaddr(env, enetaddr);
		}
		i++;
	} while (!ret);

        /* TI DP83867 */
	if (phydev->phy_id == 0x2000a231) {
		puts("DP83867 ");
		/* LED configuration */
		val = 0;
		val |= 0x5 << 4; /* LED1(Amber;Speed)   : 1000BT link */
		val |= 0xb << 8; /* LED2(Green;Link/Act): blink for TX/RX act */
		phy_write(phydev, MDIO_DEVAD_NONE, 24, val);
	}

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif // IS_ENABLED(CONFIG_FEC_MXC)

int board_init(void)
{
	gsc_init(1);

	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	gsc_hwmon();

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int ft_board_setup(void *blob, bd_t *bd)
{
	int i;

	/*
	 * remove reset gpio control as we configure the PHY registers
	 * for internal delay, LED config, and clock config in the bootloader
	 */
	i = fdt_node_offset_by_compatible(blob, -1, "fsl,imx8mm-fec");
	if (i)
		fdt_delprop(blob, i, "phy-reset-gpios");

	return 0;
}
