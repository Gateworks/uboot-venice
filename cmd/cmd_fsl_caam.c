// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 */

#include <common.h>
#include <command.h>
#include <fsl_caam.h>

static int do_caam(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int ret, i;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (strcmp(argv[1], "genblob") == 0) {
		if (argc != 5)
			return CMD_RET_USAGE;

		void *data_addr;
		void *blob_addr;
		int size;

		data_addr = (void *)simple_strtoul(argv[2], NULL, 16);
		blob_addr = (void *)simple_strtoul(argv[3], NULL, 16);
		size = simple_strtoul(argv[4], NULL, 10);
		if (size < 1) {
			printf("bad size (%d), expecting at least 1 byte\n", size);
			return CMD_RET_USAGE;
		}

		caam_open();
		ret = caam_gen_blob((uintptr_t)data_addr, (uintptr_t)blob_addr, (uint32_t)size);

		if (ret != SUCCESS) {
			printf("Error during blob encap operation: 0x%x\n", ret);
			return 0;
		}

		/* Print the generated DEK blob */
		printf("DEK blob is available at 0x%0lx and equals:\n", (uintptr_t)blob_addr);
		for (i = 0; i < size+48; i++)
			printf("%02X ", ((uint8_t *)blob_addr)[i]);
		printf("\n\n");

		return 1;
	} else if (strcmp(argv[1], "decap") == 0) {
		if (argc != 5)
			return CMD_RET_USAGE;

		void *blob_addr;
		void *data_addr;
		int size;

		blob_addr = (void *)simple_strtoul(argv[2], NULL, 16);
		data_addr = (void *)simple_strtoul(argv[3], NULL, 16);
		size      = simple_strtoul(argv[4], NULL, 10);
		if (size < 1) {
			printf("bad size (%d), expecting at least 1 byte\n", size);
			return CMD_RET_USAGE;
		}

		caam_open();
		ret = caam_decap_blob((uintptr_t)(data_addr), (uintptr_t)(blob_addr), (uint32_t)size);
		if (ret != SUCCESS) {
			printf("Error during blob decap operation: 0x%x\n", ret);
		} else {
			printf("Success, blob decap at SM PAGE1 original data is:\n");
			int i = 0;

			for (i = 0; i < size; i++) {
				printf("0x%x  ", *(unsigned char *)(data_addr + i));
				if (i % 16 == 0)
					printf("\n");
			}
			printf("\n");
		}

		return 1;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(
	caam, 5, 1, do_caam,
	"Freescale i.MX CAAM command",
	"caam genblob data_addr blob_addr data_size\n \
	caam decap blobaddr data_addr data_size\n \
	\n "
	);