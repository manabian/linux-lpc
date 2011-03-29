/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_LPC2K_IO_H
#define MACH_LPC2K_IO_H

#define IO_SPACE_LIMIT	0xffffffff

#define __mem_pci(a)	(a)
#define __io(a)		__typesafe_io(a)

#endif /* MACH_LPC2K_IO_H */
