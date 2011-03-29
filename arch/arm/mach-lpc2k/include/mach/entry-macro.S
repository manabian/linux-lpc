/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/hardware.h>

        .macro  disable_fiq
        .endm

		.macro	get_irqnr_preamble, base, tmp
		ldr	\base, =APH_VIC_BASE
		.endm

		.macro	arch_ret_to_user, tmp1, tmp2
		.endm

		.macro get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr	\irqstat, [\base, #0]
		mov     \irqnr, #0
1001:
		tst     \irqstat, #1
		bne     1002f
		add     \irqnr, \irqnr, #1
		mov     \irqstat, \irqstat, lsr #1
		cmp     \irqnr, #32
		bcc     1001b
1002:
		.endm
