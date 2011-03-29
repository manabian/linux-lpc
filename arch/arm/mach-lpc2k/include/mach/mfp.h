/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_LPC2K_MFP_H
#define ARCH_LPC2K_MFP_H

#define mfp_to_gpio(m)	((m) % 160)

/* pin number; currently only 0-159 are used */
#define MFP_PIN(x)	((x) & 0x3ff)

/* Alternate functions, currently only 0-3 are used */
#define MFP_AF0		(0x0 << 10)
#define MFP_AF1		(0x1 << 10)
#define MFP_AF2		(0x2 << 10)
#define MFP_AF3		(0x3 << 10)
#define MFP_AF_MASK	(0x3 << 10)
#define MFP_AF(x)	(((x) >> 10) & 0x3)

/* Pullup/down configuration, currently only none/high/low are used */
#define MFP_PULL_NONE	(0x2 << 21)
#define MFP_PULL_LOW	(0x3 << 21)
#define MFP_PULL_HIGH	(0x0 << 21)
#define MFP_PULL_MASK	(0x3 << 21)
#define MFP_PULL(x)	(((x) >> 21) & 0x3)

#define MFP_CFG_DEFAULT	(MFP_PULL_HIGH | MFP_AF0)

#define MFP_CFG(pin, af)		\
	((MFP_CFG_DEFAULT & ~MFP_AF_MASK) |\
	 (MFP_PIN(pin) | MFP_##af))

#define MFP_CFG_PULL(pin, af, pull)	\
	((MFP_CFG_DEFAULT & ~(MFP_AF_MASK | MFP_PULL_MASK)) |\
	 (MFP_PIN(pin) | MFP_##af | MFP_##pull))

#endif /* ARCH_LPC2K_MFP_H */
