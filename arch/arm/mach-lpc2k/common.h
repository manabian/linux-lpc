/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LPC2K_COMMON_H
#define LPC2K_COMMON_H

struct sys_timer;

extern void lpc2k_init_clocks(unsigned long xtal, unsigned long rtc);
extern void lpc2k_mfp_config(unsigned long *mfp_cfgs, int num);
extern void lpc2k_init_irq(void);

extern void lpc2k_add_uart(int nr);
extern void lpc2k_add_ssp(int nr, void *platform_data);
extern void lpc2k_add_mci(void *platform_data, int pwr_high);

extern struct sys_timer lpc2k_timer;

#endif /* LPC2K_COMMON_H */
