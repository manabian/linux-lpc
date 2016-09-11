/*
 * NXP LPC18xx/43xx boot ROM API
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

struct nxp_lpc_part {
	const char *name;
	u32 flash_size[2];
	u32 id[2];
};

struct iap_rom;

struct iap_rom *nxp_rom_iap_lookup(struct device_node *fw_node,
				   const char *prop);
const struct nxp_lpc_part *iap_get_part_info(struct iap_rom *iap);
int iap_erase_sectors(struct iap_rom *iap, u32 start_sector, u32 end_sector,
		      u32 freq_khz, u32 flash_bank);
int iap_copy_to_flash(struct iap_rom *iap, u32 start_sector, u32 end_sector,
		      void __iomem *dst, const void *src, u32 size,
		      u32 freq_khz, u32 flash_bank);
