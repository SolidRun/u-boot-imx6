/*
 * watchdog.c - driver for i.mx on-chip watchdog
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <asm/io.h>
#include <watchdog.h>
#include <asm/arch/imx-regs.h>

struct watchdog_regs {
	u16	wcr;	/* Control */
	u16	wsr;	/* Service */
	u16	wrsr;	/* Reset Status */
};

#define WCR_WDZST	0x01
#define WCR_WDBG	0x02
#define WCR_WDE		0x04	/* WDOG enable */
#define WCR_WDT		0x08
#define WCR_WDW		0x80
#define SET_WCR_WT(x)	(x << 8)

void hw_watchdog_reset(void)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;

	writew(0x5555, &wdog->wsr);
	writew(0xaaaa, &wdog->wsr);
}

#ifdef CONFIG_IMX_WATCHDOG
void hw_watchdog_init(void)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;
	u16 val = readw(&wdog->wcr);
	u16 timeout;

	/*
	 * The timer watchdog can be set between
	 * 0.5 and 128 Seconds. If not defined
	 * in configuration file, sets 128 Seconds
	 */
#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS 128000
#endif
	timeout = (CONFIG_WATCHDOG_TIMEOUT_MSECS / 500) - 1;
	val |= WCR_WDZST;
	val &= ~(0xFF << 8);
	val &= ~WCR_WDE;
	val &= ~WCR_WDT;
	val |= SET_WCR_WT(timeout);
	writew(val, &wdog->wcr);

	val |= WCR_WDE;
	writew(val, &wdog->wcr);

	hw_watchdog_reset();
}
#endif

#ifndef CONFIG_SPL_BUILD
void reset_cpu(ulong addr)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;

	writew(WCR_WDE, &wdog->wcr);
	hw_watchdog_reset();
	while (1) {
		/*
		 * spin for .5 seconds before reset
		 */
	}
}
#endif
