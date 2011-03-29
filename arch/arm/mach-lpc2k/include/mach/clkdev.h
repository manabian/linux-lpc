/*
 *  Copyright (C) 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef MACH_LPC2K_CLKDEV_H
#define MACH_LPC2K_CLKDEV_H

#define __clk_get(clk) ({ 1; })
#define __clk_put(clk) do { } while (0)

#endif /* MACH_LPC2K_CLKDEV_H */
