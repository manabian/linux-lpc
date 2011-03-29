/*
 *  Copyright (C) 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef MACH_LPC2K_GPIO_H
#define MACH_LPC2K_GPIO_H

#include <asm-generic/gpio.h>

#define gpio_get_value		__gpio_get_value
#define gpio_set_value		__gpio_set_value
#define gpio_cansleep		__gpio_cansleep
#define gpio_to_irq		__gpio_to_irq

/* Convert from PX.Y notation to GPIO number */
#define LPC2K_GPIO(port,pin)	(((port) * 32) + (pin))

#endif /* MACH_LPC2K_GPIO_H */
