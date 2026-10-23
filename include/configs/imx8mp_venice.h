/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2022 Gateworks Corporation
 */

#ifndef __IMX8MP_VENICE_H
#define __IMX8MP_VENICE_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>

#define CFG_SYS_UBOOT_BASE	\
	(QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

/* Enable Distro Boot */
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1) \
	func(MMC, mmc, 2) \
	func(USB, usb, 0) \
	func(NVME, nvme, 0) \
	func(DHCP, dhcp, na)
#include <config_distro_bootcmd.h>
#define CFG_EXTRA_ENV_SETTINGS \
	BOOTENV

#define CFG_SYS_INIT_RAM_ADDR        0x40000000
#define CFG_SYS_INIT_RAM_SIZE        SZ_2M

/* SDRAM configuration: 8GiB */
#define CFG_SYS_SDRAM_BASE           0x40000000
#define PHYS_SDRAM                   0x40000000
#define PHYS_SDRAM_SIZE              0xc0000000      /* 3 GB Bank 1 */
#define PHYS_SDRAM_2                 0x100000000
#define PHYS_SDRAM_2_SIZE            0x140000000     /* 5 GB Bank 2 */

#endif
