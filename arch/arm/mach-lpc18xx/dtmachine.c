#include <linux/kernel.h>

#include <asm/v7m.h>
#include <asm/mach/arch.h>

static const char *const lpc18xx_43xx_compat[] __initconst = {
	"ea,lpc4357-developers-kit",
	NULL
};

DT_MACHINE_START(LPC18XXDT, "NXP LPC18xx/43xx (Device Tree Support)")
	.dt_compat = lpc18xx_43xx_compat,
	.restart = armv7m_restart,
MACHINE_END
