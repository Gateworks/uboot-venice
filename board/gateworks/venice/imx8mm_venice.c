// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 Gateworks Corporation
 */

#include <common.h>
#include <init.h>
#include <led.h>
#include <linux/delay.h>
#include <miiphy.h>
#include <netdev.h>

#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>

#include "gsc.h"

DECLARE_GLOBAL_DATA_PTR;

/* IMX8M integrated peripherals have a 32bit DMA thus can
 * not access over a 32bit boundary which limits DRAM size
 * to 3GiB (because DRAM starts at 1GiB)
 *
 * As a workaround until these drivers (fec/sdhc/usb) are fixed
 * we will clamp to 3GiB and will adjust the dt back to the full
 * DRAM size for Linux before booting
 */
#define DRAM_32BIT_BOUNDARY_WORKAROUND

int board_phys_sdram_size(phys_size_t *size)
{
	int ddr_size = readl(M4_BOOTROM_BASE_ADDR);

	if (ddr_size == 0x4) {
		*size = 0x100000000;
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

#ifdef DRAM_32BIT_BOUNDARY_WORKAROUND
	if (*size >= 0xc0000000)
		*size = 0xc0000000;
#endif

	return 0;
}

int board_fit_config_name_match(const char *name)
{
	int i  = 0;
	const char *dtb;
	static char init;
	char buf[32];

	do {
		dtb = gsc_get_dtb_name(i++, buf, sizeof(buf));
		if (!strcmp(dtb, name)) {
			if (!init++)
				printf("DTB     : %s\n", name);
			return 0;
		}
	} while (dtb);

	return -1;
}

#if (IS_ENABLED(CONFIG_FEC_MXC))
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

	switch (phydev->phy_id) {
	case 0x2000a231: /* TI DP83867 GbE PHY */
		puts("DP83867 ");
		/* LED configuration */
		val = 0;
		val |= 0x5 << 4; /* LED1(Amber;Speed)   : 1000BT link */
		val |= 0xb << 8; /* LED2(Green;Link/Act): blink for TX/RX act */
		phy_write(phydev, MDIO_DEVAD_NONE, 24, val);
		break;
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

int board_late_init(void)
{
	const char *str;
	char env[32];
	int ret, i;
	u8 enetaddr[6];
	char fdt[64];

	led_default_state();

	/* Set fdt_file vars */
	i = 0;
	do {
		str = gsc_get_dtb_name(i, fdt, sizeof(fdt));
		if (str) {
			sprintf(env, "fdt_file%d", i + 1);
			strcat(fdt, ".dtb");
			env_set(env, fdt);
		}
		i++;
	} while (str);

	/* Set mac addrs */
	i = 0;
	do {
		if (i)
			sprintf(env, "eth%daddr", i);
		else
			sprintf(env, "ethaddr");
		str = env_get(env);
		if (!str) {
			ret = gsc_getmac(i, enetaddr);
			if (!ret)
				eth_env_set_enetaddr(env, enetaddr);
		}
		i++;
	} while (!ret);

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

#ifdef DRAM_32BIT_BOUNDARY_WORKAROUND
/*
 * fdt_pack_reg - pack address and size array into the "reg"-suitable stream
 */
static int fdt_pack_reg(const void *fdt, void *buf, u64 address, u64 size)
{
	int address_cells = fdt_address_cells(fdt, 0);
	int size_cells = fdt_size_cells(fdt, 0);
	char *p = buf;

	if (address_cells == 2)
		*(fdt64_t *)p = cpu_to_fdt64(address);
	else
		*(fdt32_t *)p = cpu_to_fdt32(address);
	p += 4 * address_cells;

	if (size_cells == 2)
		*(fdt64_t *)p = cpu_to_fdt64(size);
	else
		*(fdt32_t *)p = cpu_to_fdt32(size);
	p += 4 * size_cells;

	return p - (char *)buf;
}

static void venice_fixup_memory(void *fdt, int size_gb) {
	const void *prop;
	int memory;
	int len;
	u8 buf[16];

	memory = fdt_path_offset(fdt, "/memory");
	prop = fdt_getprop(fdt, memory, "reg", &len);

	if (prop && len >= 16) {
		len = fdt_pack_reg(fdt, buf, CONFIG_SYS_SDRAM_BASE,
				   (u64)size_gb * 0x40000000);
		fdt_setprop(fdt, memory, "reg", buf, len);
	}
}
#endif // ifdef DRAM_32BIT_BOUNDARY_WORKAROUND

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int i;

#ifdef DRAM_32BIT_BOUNDARY_WORKAROUND
	/*
	 * fixup memory
	 */
	i = readl(M4_BOOTROM_BASE_ADDR);
	if (i > 3)
		venice_fixup_memory(blob, i);
#endif

	return 0;
}
