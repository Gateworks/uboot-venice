// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 Gateworks Corporation
 */

#include <common.h>
#include <cpu_func.h>
#include <hang.h>
#include <i2c.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/arch/ddr.h>

#include <dm/uclass.h>
#include <dm/device.h>
#include <dm/uclass-internal.h>
#include <dm/device-internal.h>

#include <power/pmic.h>
#include <power/bd71837.h>

#include "gsc.h"
#include "lpddr4_timing.h"

DECLARE_GLOBAL_DATA_PTR;

int spl_board_boot_device(enum boot_device boot_dev_spl)
{
	switch (boot_dev_spl) {
	case USB_BOOT:
		return BOOT_DEVICE_BOARD;
	case SD2_BOOT:
	case MMC2_BOOT:
		return BOOT_DEVICE_MMC1;
	case SD3_BOOT:
	case MMC3_BOOT:
		return BOOT_DEVICE_MMC2;
	default:
		return BOOT_DEVICE_NONE;
	}
}

static void spl_dram_init(int size)
{
	struct dram_timing_info *dram_timing;

	switch (size) {
	case 1:
		dram_timing = &dram_timing_1gb;
		break;
	case 4:
		dram_timing = &dram_timing_4gb;
		break;
	default:
		printf("Unknown DDR configuration: %d GiB\n", size);
		dram_timing = &dram_timing_1gb;
		size = 1;
	}

	printf("DRAM    : LPDDR4 %d GiB\n", size);
	ddr_init(dram_timing);
	writel(size, M4_BOOTROM_BASE_ADDR);
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	return 0;
}

/* MP5416 registers */
enum {
	MP5416_CTL0		= 0x00,
	MP5416_CTL1		= 0x01,
	MP5416_CTL2		= 0x02,
	MP5416_ILIMIT		= 0x03,
	MP5416_VSET_SW1		= 0x04,
	MP5416_VSET_SW2		= 0x05,
	MP5416_VSET_SW3		= 0x06,
	MP5416_VSET_SW4		= 0x07,
	MP5416_VSET_LDO2	= 0x08,
	MP5416_VSET_LDO3	= 0x09,
	MP5416_VSET_LDO4	= 0x0a,
	MP5416_VSET_LDO5	= 0x0b,
	MP5416_STATUS1		= 0x0d,
	MP5416_STATUS2		= 0x0e,
	MP5416_STATUS3		= 0x0f,
	MP5416_ID2		= 0x11,
	MP5416_NUM_OF_REGS	= 0x12,
};

#define MP5416_VSET_EN          BIT(7)

static int power_init(void)
{
        struct udevice *dev;
        int ret;
#ifndef CONFIG_SPL_DM_PMIC_BD71837
	int slave = 0x69;
	int busno = 1;
	unsigned char reg;

#ifdef CONFIG_SPL_BUILD
        ret = i2c_get_chip_for_busnum(busno + 1, slave, 1, &dev);
        if (ret)
                return ret;
#else
        struct udevice *bus;

        busno--;

        ret = uclass_get_device_by_seq(UCLASS_I2C, busno, &bus);
        if (ret) {
                printf("i2c%d: no bus %d\n", busno + 1, ret);
                return ret;
        }
        ret = i2c_get_chip(bus, slave, 1, &dev);
        if (ret) {
                printf("i2c%d@0x%02x: no chip %d\n", busno + 1, slave, ret);
                return ret;
        }
#endif

	/* configure MP5416 PMIC */
#define MP5416_VSET_SW1_GVAL(x) (((x & 0x7f)*12500)+600000)
#define MP5416_VSET_SW2_GVAL(x) (((x & 0x7f)*25000)+800000)
#define MP5416_VSET_SW3_GVAL(x) (((x & 0x7f)*12500)+600000)
#define MP5416_VSET_SW4_GVAL(x) (((x & 0x7f)*25000)+800000)
#define MP5416_VSET_LDO_GVAL(x) (((x & 0x7f)*25000)+800000)
#define MP5416_VSET_LDO_SVAL(x) (((x & 0x7f)*25000)+800000)
#define MP5416_VSET_SW1_SVAL(x) ((x-600000)/12500)
#define MP5416_VSET_SW2_SVAL(x) ((x-800000)/25000)
#define MP5416_VSET_SW3_SVAL(x) ((x-600000)/12500)
#define MP5416_VSET_SW4_SVAL(x) ((x-800000)/25000)
	puts("PMIC    : MP5416\n");
	/* set VDD_ARM SW3 to 0.92V for 1.6GHz */
	reg = BIT(7) | MP5416_VSET_SW3_SVAL(920000);
	ret = dm_i2c_write(dev, MP5416_VSET_SW3, &reg, 1);
	/* read rails */
/*
        ret = dm_i2c_read(dev, MP5416_VSET_SW1, &reg, 1);
	printf("VDD_0P95: %d.%02dV\n", MP5416_VSET_SW1_GVAL(reg) / 1000000,
		MP5416_VSET_SW1_GVAL(reg) % 1000000);
        ret = dm_i2c_read(dev, MP5416_VSET_SW2, &reg, 1);
	printf("VDD_SOC : %d.%02dV\n", MP5416_VSET_SW2_GVAL(reg) / 1000000,
		MP5416_VSET_SW2_GVAL(reg) % 1000000);
*/
        ret = dm_i2c_read(dev, MP5416_VSET_SW3, &reg, 1);
	printf("VDD_ARM : %d.%02dV\n", MP5416_VSET_SW3_GVAL(reg) / 1000000,
		MP5416_VSET_SW3_GVAL(reg) % 1000000);
/*
        ret = dm_i2c_read(dev, MP5416_VSET_SW4, &reg, 1);
	printf("VDD_1P8 : %d.%02dV\n", MP5416_VSET_SW4_GVAL(reg) / 1000000,
		MP5416_VSET_SW4_GVAL(reg) % 1000000);
        ret = dm_i2c_read(dev, MP5416_VSET_LDO2, &reg, 1);
	printf("VDD_1P8 : %d.%02dV\n", MP5416_VSET_LDO_GVAL(reg) / 1000000,
		MP5416_VSET_LDO_GVAL(reg) % 1000000);
        ret = dm_i2c_read(dev, MP5416_VSET_LDO3, &reg, 1);
	printf("VDD_0P8 : %d.%02dV\n", MP5416_VSET_LDO_GVAL(reg) / 1000000,
		MP5416_VSET_LDO_GVAL(reg) % 1000000);
        ret = dm_i2c_read(dev, MP5416_VSET_LDO4, &reg, 1);
	printf("VDD_0P9 : %d.%02dV\n", MP5416_VSET_LDO_GVAL(reg) / 1000000,
		MP5416_VSET_LDO_GVAL(reg) % 1000000);
        ret = dm_i2c_read(dev, MP5416_VSET_LDO5, &reg, 1);
	printf("VDD_1P8 : %d.%02dV\n", MP5416_VSET_LDO_GVAL(reg) / 1000000,
		MP5416_VSET_LDO_GVAL(reg) % 1000000);
*/
	puts("PMIC    : MP5416\n");
#else
	ret = pmic_get("pmic@4b", &dev);
	if (ret == -ENODEV) {
		puts("No pmic\n");
		return 0;
	}
	if (ret != 0)
		return ret;

	/* unlock the PMIC regs */
	pmic_reg_write(dev, BD718XX_REGLOCK, 0x1);

	/* set switchers to forced PWM mode */
	pmic_clrsetbits(dev, BD718XX_BUCK1_CTRL, 0, 0x8);
	pmic_clrsetbits(dev, BD718XX_BUCK2_CTRL, 0, 0x8);
	pmic_clrsetbits(dev, BD718XX_1ST_NODVS_BUCK_CTRL, 0, 0x8);
	pmic_clrsetbits(dev, BD718XX_2ND_NODVS_BUCK_CTRL, 0, 0x8);
	pmic_clrsetbits(dev, BD718XX_4TH_NODVS_BUCK_CTRL, 0, 0x8);

	/* increase VDD_0P95 (VDD_GPU/VPU/DRAM) to 0.975v for 1.5Ghz DDR */
	pmic_reg_write(dev, BD718XX_1ST_NODVS_BUCK_VOLT, 0x83);

	/* increase VDD_SOC to 0.85v before first DRAM access */
	pmic_reg_write(dev, BD718XX_BUCK1_VOLT_RUN, 0x0f);

	/* increase VDD_ARM to 0.92v for 800 and 1600Mhz */
	pmic_reg_write(dev, BD718XX_BUCK2_VOLT_RUN, 0x16);

	/* Lock the PMIC regs */
	pmic_reg_write(dev, BD718XX_REGLOCK, 0x11);
	puts("PMIC    : BD71847\n");
#endif

	return 0;
}

void board_init_f(ulong dummy)
{
	struct udevice *dev;
	int ret;
	int dram_sz;

	arch_cpu_init();

	init_uart_clk(1);

	board_early_init_f();

	timer_init();

	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	ret = spl_early_init();
	if (ret) {
		debug("spl_early_init() failed: %d\n", ret);
		hang();
	}

	ret = uclass_get_device_by_name(UCLASS_CLK,
					"clock-controller@30380000",
					&dev);
	if (ret < 0) {
		printf("Failed to find clock node. Check device tree\n");
		hang();
	}

	enable_tzc380();

	/* GSC */
	dram_sz = gsc_init(0);

	/* PMIC */
	power_init();

	/* DDR initialization */
	spl_dram_init(dram_sz);

	board_init_r(NULL, 0);
}

void board_boot_order(u32 *spl_boot_list)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_BOARD:
		/*
		 * If the SPL was loaded via serial loader, we try to get
		 * U-Boot proper via USB SDP.
		 */
		spl_boot_list[0] = BOOT_DEVICE_BOARD;
		break;
	default:
		/*
		 * Else, we try to load it from eMMC then SD-card
		 */
		spl_boot_list[0] = BOOT_DEVICE_MMC2;
		spl_boot_list[1] = BOOT_DEVICE_MMC1;
	}
}
