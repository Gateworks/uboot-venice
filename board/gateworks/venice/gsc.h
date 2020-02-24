#ifndef _GSC_H_
#define _GSC_H_

/* i2c slave addresses */
#define GSC_SC_ADDR             0x20
#define GSC_RTC_ADDR            0x68
#define GSC_HWMON_ADDR          0x29
#define GSC_EEPROM_ADDR         0x51

/* MAX6642 temperature sensor */
#define MAX6642_SLAVE           0x4a
#define MAX6642_R_TEMP_LOCAL    0x00
#define MAX6642_R_TEMP_REMOTE   0x01
#define MAX6642_R_STATUS        0x02
#define MAX6642_R_CONFIG        0x03
#define MAX6642_R_LIMIT_LOCAL   0x05
#define MAX6642_R_LIMIT_REMOTE  0x07
#define MAX6642_W_CONFIG        0x09
#define MAX6642_W_LIMIT_LOCAL   0x0b
#define MAX6642_W_LIMIT_REMOTE  0x0d
#define MAX6642_R_TEMP_LOCAL_E  0x10
#define MAX6642_R_TEMP_REMOTE_E 0x11

struct venice_board_info {
	uint8_t mac[6];		/* 0x00: MAC base */
	char equiv_dts[16];	/* 0x06: equivalent device-tree */
	uint8_t res0[2];	/* 0x16: reserved */
	uint32_t serial;	/* 0x18: Serial Number */
	uint8_t res1[4];	/* 0x1C: reserved */
	uint8_t mfgdate[4];	/* 0x20: MFG date */
	uint8_t macno;		/* 0x24: number of mac addrs */
	uint8_t res2[6];	/* 0x25 */
	/* sdram config */
	uint8_t sdram_size;	/* 0x2B: (16 << n) MB */
	uint8_t sdram_speed;	/* 0x2C: (33.333 * n) MHz */
	uint8_t sdram_width;	/* 0x2D: (8 << n) bit */
	uint8_t res3[2];	/* 0x2E */
	char model[16];		/* 0x30: model string */
	uint8_t res4[14];	/* 0x40 */

	uint8_t chksum[2];	/* 0x4E */
};

int gsc_init(void);
int gsc_hwmon(void);
const char *gsc_get_dtb_name(int level);

#endif

