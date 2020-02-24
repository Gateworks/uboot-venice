/*
 * Copyright 2020 Gateworks Corporation
 */

#include <common.h>
#include <hexdump.h>
#include <i2c.h>
#include <dm/uclass.h>

#include "gsc.h"

DECLARE_GLOBAL_DATA_PTR;

#define GSC_PROBE_RETRY_MS 500

struct venice_board_info board_info;
/*
struct venice_board_info *gsc_get_board_info(void) {
	return &board_info;
}
*/

/*
int board_model = GW_UNKNOWN;
int gsc_get_board_model(void) {
	return board_model;
}
*/

/* System Controller registers */
enum {
	GSC_SC_CTRL0		= 0,
	GSC_SC_CTRL1		= 1,
	GSC_SC_STATUS		= 10,
	GSC_SC_FWCRC		= 12,
	GSC_SC_FWVER		= 14,
	GSC_SC_WP		= 15,
	GSC_SC_RST_CAUSE	= 16,
	GSC_SC_THERM_PROTECT	= 19,
};

/* System Controller Control1 bits */
enum {
	GSC_SC_CTRL1_WDTIME	= 4, /* 1 = 60s timeout, 0 = 30s timeout */
	GSC_SC_CTRL1_WDEN	= 5, /* 1 = enable, 0 = disable */
	GSC_SC_CTRL1_BOOT_CHK   = 6, /* 1 = enable alt boot check */
	GSC_SC_CTRL1_WDDIS	= 7, /* 1 = disable boot watchdog */
};

/* System Controller Interrupt bits */
enum {
	GSC_SC_IRQ_PB		= 0, /* Pushbutton switch */
	GSC_SC_IRQ_SECURE	= 1, /* Secure Key erase operation complete */
	GSC_SC_IRQ_EEPROM_WP	= 2, /* EEPROM write violation */
	GSC_SC_IRQ_GPIO		= 4, /* GPIO change */
	GSC_SC_IRQ_TAMPER	= 5, /* Tamper detect */
	GSC_SC_IRQ_WATCHDOG	= 6, /* Watchdog trip */
	GSC_SC_IRQ_PBLONG	= 7, /* Pushbutton long hold */
};

/* System Controller WP bits */
enum {
	GSC_SC_WP_ALL		= 0, /* Write Protect All EEPROM regions */
	GSC_SC_WP_BOARDINFO	= 1, /* Write Protect Board Info region */
};
#define GSC_WP_PASSWD		0x58
#define GSC_WP_PASSWD_MASK	0xF8

/* System Controller Reset Cause */
enum {
	GSC_SC_RST_CAUSE_VIN		= 0,
	GSC_SC_RST_CAUSE_PB		= 1,
	GSC_SC_RST_CAUSE_WDT		= 2,
	GSC_SC_RST_CAUSE_CPU		= 3,
	GSC_SC_RST_CAUSE_TEMP_LOCAL	= 4,
	GSC_SC_RST_CAUSE_TEMP_REMOTE	= 5,
	GSC_SC_RST_CAUSE_SLEEP		= 6,
	GSC_SC_RST_CAUSE_BOOT_WDT	= 7,
	GSC_SC_RST_CAUSE_BOOT_WDT_MAN	= 8,
	GSC_SC_RST_CAUSE_SOFT_PWR	= 9,
	GSC_SC_RST_CAUSE_MAX		= 10,
};

static struct udevice *gsc_get_dev(int busno, int slave)
{
	struct udevice *dev;
	int ret;

#ifdef CONFIG_SPL_BUILD
	ret = i2c_get_chip_for_busnum(busno + 1, slave, 1, &dev);
        if (ret)
                return NULL;
#else
	struct udevice *bus;

	busno--;

	ret = uclass_get_device_by_seq(UCLASS_I2C, busno, &bus);
	if (ret) {
		printf("i2c%d: no bus %d\n", busno + 1, ret);
		return NULL;
	}
	ret = i2c_get_chip(bus, slave, 1, &dev);
	if (ret) {
		printf("i2c%d@0x%02x: no chip %d\n", busno + 1, slave, ret);
		return NULL;
	}
#endif

	return dev;
}

/* read EEPROM:
 * - read EEPROM and check for EEPROM validity
 * - init BDK variables (BDK_CONFIG_BOARD_MODEL, BDK_CONFIG_BOARD_SERIAL,
 *   and BDK_CONFIG_BOARD_REVISION)
 * - return baseboard type
 */
static int gsc_read_eeprom(void)
{
	struct venice_board_info *info = &board_info;
	int i;
	int chksum;
	unsigned char *buf = (unsigned char *)info;
//	int type;
//	char serial_str[8];
	char revision_str[8] = { 0 };
	char rev_pcb = 'A'; /* PCB revision */
	int  rev_bom = 0; /* BOM revision */
	struct udevice *dev;
	int ret;

	/* probe device */
	dev = gsc_get_dev(1, GSC_EEPROM_ADDR);
	if (!dev) {
		puts("ERROR: Failed to probe EEPROM\n");
		return -ENODEV;
	}

	/* read eeprom config section */
	memset(info, 0, sizeof(*info));
	ret = dm_i2c_read(dev, 0x00, buf, sizeof(*info));
	if (ret) {
		puts("EEPROM: Failed to read EEPROM\n");
		return ret;
	}

	/* validate checksum */
	for (chksum = 0, i = 0; i < (int)sizeof(*info)-2; i++)
		chksum += buf[i];
	if ((info->chksum[0] != chksum>>8) ||
	    (info->chksum[1] != (chksum&0xff))) {
                puts("EEPROM: Invalid Model in EEPROM\n");
                print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buf,
                                     sizeof(*info));
		//return GW_UNKNOWN;
		return -EINVAL;
	}

	/* sanity checks */
	if (info->model[0] != 'G' || info->model[1] != 'W') {
		puts("EEPROM: Invalid Model in EEPROM\n");
                print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buf,
                                     sizeof(*info));
		//return GW_UNKNOWN;
		return -EINVAL;
	}

	/* use model as equivalent DTS if not specified */
	if ((info->equiv_dts[0] == 0) || (info->equiv_dts[0] == 0xff)) {
		strncpy(info->equiv_dts, info->model,
		        sizeof(info->equiv_dts) - 1);
	}
//	type = GW_UNKNOWN;
#if 0
	switch (info->equiv_dts[3]) {
	case '1':
		type = GW610x;
		break;
	case '2':
		type = GW620x;
		break;
	case '3':
		type = GW630x;
		break;
	case '4':
		type = GW640x;
		break;
	case '9':
		if (!strncmp("GW6903", info->equiv_dts, 6))
			type = GW6903;
		break;
	}
	if (type == GW_UNKNOWN) {
		puts("EEPROM: Failed model identification\n");
                print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, buf,
                                     sizeof(*info));
	}
#endif

#if 0
	/* store board/serial */
	bdk_config_set_str(info->model, BDK_CONFIG_BOARD_MODEL);
	sprintf(serial_str, "%d", info->serial);
	bdk_config_set_str(serial_str, BDK_CONFIG_BOARD_SERIAL);

	/* store MAC addr */
	int macno = (info->macno == 0xff) ? 2 : info->macno;
	bdk_config_set_int(macno, BDK_CONFIG_MAC_ADDRESS_NUM);
	uint64_t mac = 0;
	for (i = 5; i >= 0; i--)
		mac |= (uint64_t)info->mac[i] << ((5-i)*8);
	bdk_config_set_int(mac, BDK_CONFIG_MAC_ADDRESS);
#endif

	/* determine BOM revision from model */
	for (i = strlen(info->equiv_dts) - 1; i > 0; i--) {
		if (info->equiv_dts[i] == '-')
			break;
		if (info->equiv_dts[i] >= '1' && info->equiv_dts[i] <= '9') {
			rev_bom = info->equiv_dts[i] - '0';
			break;
		}
	}

	/* determine PCB revision from model */
	for (i = strlen(info->equiv_dts) - 1; i > 0; i--) {
		if (info->equiv_dts[i] == '-')
			break;
		if (info->equiv_dts[i] >= 'A') {
			rev_pcb = info->equiv_dts[i];
			break;
		}
	}
	if (rev_bom)
		sprintf(revision_str, "%c.%d", rev_pcb, rev_bom);
	else
		sprintf(revision_str, "%c", rev_pcb);
#if 0
	bdk_config_set_str(revision_str, BDK_CONFIG_BOARD_REVISION);
#endif

	/* board revision specific changes */
//	board_model = type;
//	return type;

	return 0;
}

#if 0
/* write EEPROM with proper checksum */
static int gsc_eeprom_update(void)
{
	struct venice_board_info *info = &board_info;
	unsigned char *buf = (unsigned char *)info;
	struct udevice *dev;
	int i, ret, chksum;
//	uint64_t mac;

	/* probe device */
	dev = gsc_get_dev(1, GSC_EEPROM_ADDR);
	if (!dev) {
		puts("ERROR: Failed to probe EEPROM\n");
		return -ENODEV;
	}

#if 0
	/* update info from dt */
	strncpy(info->model, bdk_config_get_str(BDK_CONFIG_BOARD_MODEL),
		sizeof(info->model));
	info->serial = atoi(bdk_config_get_str(BDK_CONFIG_BOARD_SERIAL));
	mac = bdk_config_get_int(BDK_CONFIG_MAC_ADDRESS);
	for (i = 0; i < 6; i++)
		info->mac[5-i] = (mac >> (i*8)) & 0xff;
#endif

	/* update checksum */
	for (chksum = 0, i = 0; i < (int)sizeof(*info)-2; i++)
		chksum += buf[i];
	info->chksum[0] = chksum >> 8;
	info->chksum[1] = chksum & 0xff;

	/* write eeprom config section */
	memset(info, 0, sizeof(*info));
	ret = dm_i2c_write(dev, 0, buf, sizeof(*info));
	if (ret) {
		puts("EEPROM: Failed to write EEPROM\n");
		return ret;
	}

	return 0;
}
#endif

static int gsc_hwmon_reg(struct udevice *dev, int reg)
{
	uint8_t buf[2];
	int ret;

	memset(buf, 0, sizeof(buf));
	ret = dm_i2c_read(dev, reg, buf, sizeof(buf));
	if (ret) {
		printf("i2c error: %d\n", ret);
		return ret;
	}
	return buf[0] | buf[1]<<8;
}

static const char *gsc_get_rst_cause(struct udevice *dev)
{
	static char str[64];
	const char *names[] = {
		"VIN",
		"PB",
		"WDT",
		"CPU",
		"TEMP_L",
		"TEMP_R",
		"SLEEP",
		"BOOT_WDT1",
		"BOOT_WDT2",
		"SOFT_PWR",
	};
	unsigned char reg;

	/* reset cause */
	str[0] = 0;
	if (!dm_i2c_read(dev, GSC_SC_RST_CAUSE, &reg, 1)) {
		if (reg < ARRAY_SIZE(names))
			sprintf(str, "%s", names[reg]);
		else
			sprintf(str, "0x%02x", reg);
	}

	/* thermal protection */
	if (!dm_i2c_read(dev, GSC_SC_THERM_PROTECT, &reg, 1)) {
		reg |= 1;
		dm_i2c_write(dev, GSC_SC_THERM_PROTECT, &reg, 1);
		strcat(str, " Thermal Protection Enabled");
	}

	return str;
}

#if 0
/* board temp is unsigned int at reg 0 */
static int gsc_board_temp(struct udevice *dev)
{
	int ui;

	ui = gsc_hwmon_reg(dev, 0);
	if (ui > 0x8000)
		ui -= 0xffff;

	return ui;
}

/* update MAX6642 remote threshold per CN80XXI-AD-1.0 */
static int init_max6642(void)
{
	int tj_max, i, ret;
	unsigned char reg;
	struct udevice *dev;
	unsigned char temp_remote = 0, temp_local = 0;
	int temp_gsc = 0;

	/* probe max6642 device */
	dev = gsc_get_dev(1, MAX6642_SLAVE);
	if (!dev) {
		puts("ERROR: Failed to probe MAX6642\n");
		return -ENODEV;
	}

	/*
	 * CN80XX industrial temp max Junction Temp (Tj):
	 *  800MHz: 100C
	 * 1000MHz: 100C
	 * 1200MHz: 100C
	 * 1500MHz: 95C
	 */
	tj_max = 100;
#if 0
	if ( (bdk_clock_get_rate(node, BDK_CLOCK_RCLK) / 1000000) >= 1500)
		tj_max -= 5;
#endif

	/* set new limits:
	 *  Note disabling local sense keeps you from ever being able to
	 *  clear the alert if it has tripped so instead we just set the
	 *  local limit to the remote limit so remote will always trip first.
	 */
	reg = 0x10;
	ret = dm_i2c_write(dev, MAX6642_W_CONFIG, &reg, 1);
	if (ret)
		return ret;
	reg = tj_max;
	ret = dm_i2c_write(dev, MAX6642_W_LIMIT_LOCAL, &reg, 1);
	if (ret)
		return ret;
	ret = dm_i2c_write(dev, MAX6642_W_LIMIT_REMOTE, &reg, 1);
	if (ret)
		return ret;

	/* loop over status until clear */
	for (i = 0; i < 20; i++) {
		ret = dm_i2c_read(dev, MAX6642_R_STATUS, &reg, 1);
		if (ret)
			return ret;
		if (!(reg & 0x50))
			break;
		udelay(500000);
	}

	/* read remote temp */
	ret = dm_i2c_read(dev, MAX6642_R_TEMP_REMOTE, &temp_remote, 1);
	ret = dm_i2c_read(dev, MAX6642_R_LIMIT_REMOTE, &temp_local, 1);

	/* probe gsc hwmon device */
	dev = gsc_get_dev(1, GSC_HWMON_ADDR);
	if (dev) 
		temp_gsc = gsc_board_temp(dev);

	printf("Temp    : Board:%dC/86C CPU:%dC/%dC\n", temp_gsc / 10,
	       temp_local, temp_remote);

	return 0;
}
#endif

static int gsc_boot_wd_disable(void)
{
	uint8_t reg;
	struct udevice *dev;
	int ret;

	/* probe device */
	dev = gsc_get_dev(1, GSC_SC_ADDR);
	if (!dev)
		return -ENODEV;

	ret = dm_i2c_read(dev, GSC_SC_CTRL1, &reg, 1);
	if (ret)
		goto err;
	reg |= (1 << GSC_SC_CTRL1_WDDIS);
	reg &= ~(1 << GSC_SC_CTRL1_BOOT_CHK);
	ret = dm_i2c_write(dev, GSC_SC_CTRL1, &reg, 1);
	if (ret)
		goto err;
	puts("GSC     : boot watchdog disabled\n");

	return 0;

err:
	printf("i2c error");
	return ret;
}

#if 0
static int gsc_get_fwver(void)
{
	unsigned char reg;
	struct udevice *dev;
	int ret;

	/* probe device */
	dev = gsc_get_dev(1, GSC_SC_ADDR);
	if (!dev)
		return -ENODEV;

	ret = dm_i2c_read(dev, GSC_SC_FWVER, &reg, 1);
	if (ret)
		return ret;

	return reg;
}
#endif

int gsc_hwmon(void)
{
	const void *fdt = gd->fdt_blob;
	struct udevice *dev;
	int node, reg, mode, len, val, offset;
	const char *label;

	node = fdt_node_offset_by_compatible(fdt, -1, "gw,gsc-adc");
	if (node <= 0)
		return node;

	/* probe device */
#if 1 // works in spl but not uboot
	dev = gsc_get_dev(1, GSC_HWMON_ADDR);
	if (!dev) {
		puts("ERROR: Failed to probe GSC HWMON\n");
		return -ENODEV;
	}
#else
int ret;
int busno = 0;
struct udevice *bus;

ret = uclass_get_device_by_seq(UCLASS_I2C, busno, &bus);
if (ret) {
	printf("no bus %d\n", ret);
	return ret;
}
ret = i2c_get_chip(bus, GSC_HWMON_ADDR, 1, &dev);
if (ret) {
	printf("no chip %d\n", ret);
	return ret;
}
#endif

	/* iterate over hwmon nodes */
	node = fdt_first_subnode(fdt, node);
	while (node > 0) {
		reg = fdtdec_get_int(fdt, node, "reg", -1);
		mode = fdtdec_get_int(fdt, node, "gw,mode", -1);
		offset = fdtdec_get_int(fdt, node, "gw,voltage-offset-microvolt", 0);
		label = fdt_stringlist_get(fdt, node, "label", 0, NULL);

		if ((reg == -1) || (mode == -1) || !label)
			printf("invalid dt:%s\n", fdt_get_name(fdt, node, NULL));

		val = gsc_hwmon_reg(dev, reg);
	if (val >= 0) {
		switch (mode) {
		case 0: /* temperature (C*10) */
			if (val > 0x8000)
				val -= 0xffff;
			printf("%-8s: %d.%ldC\n", label, val / 10,
				abs(val % 10));
			break;
		case 1: /* prescaled voltage */
			if (val != 0xffff) {
				printf("%-8s: %d.%03dV\n", label,
					val / 1000, val % 1000);
			}
			break;
		case 2: /* scaled based on ref volt and resolution */
			val *= 2500;
			val /= 1<<12;

			/* apply pre-scaler voltage divider */
			const uint32_t *div;
			int r[2];
			div  = fdt_getprop(fdt, node, "gw,voltage-divider-ohms",
					   &len);
			if (div && (len == sizeof(uint32_t) * 2)) {
				r[0] = fdt32_to_cpu(div[0]);
				r[1] = fdt32_to_cpu(div[1]);
				if (r[0] && r[1]) {
					val *= (r[0] + r[1]);
					val /= r[1];
				}
			}

			/* adjust by offset */
			val += offset;

			printf("%-8s: %d.%03dV\n", label,
				val / 1000, val % 1000);
			break;
		}
	}

		node = fdt_next_subnode(fdt, node);
	}

	return 0;
}

/* gsc_init:
 *
 * This is called from early init (boot stub) to determine board model
 * and perform any critical early init including:
 *  - configure early GPIO (ie front panel GRN LED, default states)
 *  - display GSC details banner
 *  - display EEPROM model/mfgdate/serial
 *  - configuring temperature sensor thresholds
 */
int gsc_init(void)
{
	struct venice_board_info *info = &board_info;
	unsigned char buf[16];
	struct udevice *dev;
	int i, ret;

	/*
	 * On a board with a missing/depleted backup battery for GSC, the
	 * board may be ready to probe the GSC before its firmware is
	 * running.  We will wait here indefinately for the GSC/EEPROM.
	 */
	for (i = GSC_PROBE_RETRY_MS; i > 0; i--) {
		/* probe device */
		dev = gsc_get_dev(1, GSC_SC_ADDR);
		if (dev)
			break;
		mdelay(1);
	}
	if (!dev) {
		puts("ERROR: Failed probing GSC\n");
		return -ENODEV;
	}

	ret = dm_i2c_read(dev, 0, buf, sizeof(buf)); 
	if (ret) {
		puts("ERROR: Failed reading GSC\n");
		return ret;
	}

	/* banner */
	printf("GSC     : v%d 0x%04x", buf[GSC_SC_FWVER],
		buf[GSC_SC_FWCRC] | buf[GSC_SC_FWCRC+1]<<8);
	printf(" RST:%s", gsc_get_rst_cause(dev));
	printf("\n");

	ret = gsc_read_eeprom();
	if (ret)
#if 0 // allow failure for now
		return ret;
#else
		memset(info, 0, sizeof(*info));
#endif

	//printf("Temp    : Board:%dC/86C\n", gsc_board_temp(dev) / 10);
	printf("Model   : %s\n", info->model);
	printf("MFGDate : %02x-%02x-%02x%02x\n",
		info->mfgdate[0], info->mfgdate[1],
		info->mfgdate[2], info->mfgdate[3]);
	printf("Serial  : %d\n", info->serial);

	/* Display RTC */
	puts("RTC     : ");
	dev = gsc_get_dev(1, GSC_RTC_ADDR);
	if (!dev) {
		puts("Failed to probe GSC RTC\n");
	} else {
		dm_i2c_read(dev, 0, buf, 6);
		printf("%d\n", buf[0] | buf[1]<<8 | buf[2]<<16 | buf[3]<<24);
	}

#if 0
	/*
	 * Configure early GPIO
	 */
	/* Enable front-panel LEDs */
	if (cfg->gpio_ledblu != -1) {
		/* tri-color LED stack individually controlled active low */
		if (cfg->gpio_ledgrn != -1)
			gpio_output(cfg->gpio_ledgrn, 0);
		if (cfg->gpio_ledred != -1)
			gpio_output(cfg->gpio_ledred, 1);
		if (cfg->gpio_ledblu != -1)
			gpio_output(cfg->gpio_ledblu, 1);
	} else {
		/* bi-color LED stack push-pull active high */
		if (cfg->gpio_ledgrn != -1)
			gpio_output(cfg->gpio_ledgrn, 1);
		if (cfg->gpio_ledred != -1)
			gpio_output(cfg->gpio_ledred, 0);
	}
	/* Configure Mezzanine IO */
	if (cfg->gpio_mezz_pwrdis != -1)
		gpio_output(cfg->gpio_mezz_pwrdis, 0);
	if (cfg->gpio_mezz_irq != -1)
		gpio_input(cfg->gpio_mezz_irq);
#endif

	gsc_boot_wd_disable();
//	gsc_hwmon();

	return 0;
}

#if 0
/*
 * dtb based on model:
 *   try full first        (ie gw6300-b.1)
 *   try pcb level next    (ie gw6300-b)
 *   try base model next   (ie gw6300)
 *   try generic base next (ie gw630x)
 *   try generic base last (ie gw63xx)
 */
const char *gsc_get_dtb_name(int level)
{
	static char file[64];
	char base[32];
	char rev_pcb = 'a'; /* PCB revision */
	int  rev_bom = 0; /* BOM revision */
	char *p;
	int i;
	struct venice_board_info *info = &board_info;
	const char *model = info->equiv_dts;

	/* determine base model from model */
	for (i = 0;i < (int)strlen(model) && i < ((int)sizeof(base) - 1);i++) {
		char c = model[i];
		base[i] = (c >= 'A' && c <= 'Z') ? (c+32) : c;
	}
	base[i+1] = 0;
	p = strchr(base, '-');
	if (p)
		*p = 0;

	/* determine BOM revision from model */
	for (i = strlen(model); i > 0; i--) {
		if (model[i] == '-')
			break;
		if (model[i] >= '1' && model[i] <= '9') {
			rev_bom = model[i] - '0';
			break;
		}
	}

	/* determine PCB revision from model */
	for (i = sizeof(model) - 1; i > 0; i--) {
		if (model[i] == '-')
			break;
		if (model[i] >= 'A' && model[i] <= 'Z') {
			rev_pcb = model[i] + 32;
			break;
		}
	}

	switch (level) {
	case 0: /* full model first (ie GW6300-A.1) */
		if (rev_bom)
			sprintf(file, "%s-%c.%d", base, rev_pcb, rev_bom);
		else
			sprintf(file, "%s-%c", base, rev_pcb);
		break;
	case 1: /* look for model and pcb rev (ie GW6300-A) */
		sprintf(file, "%s-%c", base, rev_pcb);
		break;
	case 2: /* look for base model (ie GW6300) */
		sprintf(file, "%s", base);
		break;
	case 3: /* look for generic model (ie gw630x) */
		base[5] = 'x';
		sprintf(file, "%s", base);
		break;
	case 4: /* look for more generic model (ie gw63xx) */
		base[4] = 'x';
		sprintf(file, "%s", base);
		break;
	case 5: /* look for more generic model (ie gw6xxx) */
		base[3] = 'x';
		sprintf(file, "%s", base);
		break;
	default: /* give it up */
		return NULL;
	}
	return file;
}

static int gsc_sleep(unsigned long secs)
{
	unsigned char reg;
	struct udevice *dev;
	int ret;

	/* probe device */
	dev = gsc_get_dev(1, GSC_SC_ADDR);
	if (!dev)
		return -ENODEV;

	printf("GSC Sleeping for %ld seconds\n", secs);
	reg = (secs >> 24) & 0xff;
	ret = dm_i2c_write(dev, 9, &reg, 1);
	if (ret)
		goto err;
	reg = (secs >> 16) & 0xff;
	ret = dm_i2c_write(dev, 8, &reg, 1);
	if (ret)
		goto err;
	reg = (secs >> 8) & 0xff;
	ret = dm_i2c_write(dev, 7, &reg, 1);
	if (ret)
		goto err;
	reg = secs & 0xff;
	ret = dm_i2c_write(dev, 6, &reg, 1);
	if (ret)
		goto err;
	ret = dm_i2c_read(dev, GSC_SC_CTRL1, &reg, 1);
	if (ret)
		goto err;
	reg |= (1 << 2);
	ret = dm_i2c_write(dev, GSC_SC_CTRL1, &reg, 1);
	if (ret)
		goto err;
	reg &= ~(1 << 2);
	reg |= 0x3;
	ret = dm_i2c_write(dev, GSC_SC_CTRL1, &reg, 1);
	if (ret)
		goto err;

	return 0;

err:
	printf("i2c error\n");
	return ret;
}
#endif
