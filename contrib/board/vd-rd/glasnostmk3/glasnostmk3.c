/* ----------------------------------------------------------------------------
 *         WHATCLOUD SRL - EMBEDDED WORKS
 * ----------------------------------------------------------------------------
 * Copyright (c) 2006, Microchip Technology Inc. and its subsidiaries
 * Copyright (c) 2024, Whatcloud SRL
 *
 * Based on at91sam9g10ek (AT91SAM9G10-EK) board support code.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Microchip's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY WHATCLOUD "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL WHATCLOUD BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "hardware.h"
#include "arch/at91_ccfg.h"
#include "arch/at91sam9g10_matrix.h"
#include "arch/at91_rstc.h"
#include "arch/at91_pmc/pmc.h"
#include "arch/at91_smc.h"
#include "arch/at91_pio.h"
#include "arch/at91_sdramc.h"
#include "spi.h"
#include "gpio.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "sdramc.h"
#include "timer.h"
#include "watchdog.h"
#include "glasnostmk3.h"

static inline void matrix_writel(const unsigned int value, unsigned int reg)
{
	writel(value, reg + AT91C_BASE_MATRIX);
}

static inline unsigned int matrix_readl(unsigned int reg)
{
	return readl(reg + AT91C_BASE_MATRIX);
}

static void at91_matrix_hw_init(void)
{
	unsigned int reg;

	reg = matrix_readl(MATRIX_SCFG3);
	reg &= ~AT91C_MATRIX_SLOT_CYCLE;
	reg |= AT91C_MATRIX_SLOT_CYCLE_(0x40);
	matrix_writel(reg, MATRIX_SCFG3);

	reg = matrix_readl(MATRIX_SCFG0);
	reg |= AT91C_MATRIX_DEFMSTR_TYPE_FIXED_DEFMSTR;
	reg |= AT91C_MATRIX_FIXED_DEFMSTR_ARM926D;
	matrix_writel(reg, MATRIX_SCFG0);

	reg = matrix_readl(MATRIX_SCFG3);
	reg |= AT91C_MATRIX_DEFMSTR_TYPE_FIXED_DEFMSTR;
	reg |= AT91C_MATRIX_FIXED_DEFMSTR_ARM926D;
	matrix_writel(reg, MATRIX_SCFG3);
}

static void led_hw_init(void)
{
	writel(((0x01 << 26) | (0x01 << 28) | (0x01 << 29)), AT91C_BASE_PIOA + PIO_ASR);
	writel(((0x01 << 26) | (0x01 << 28) | (0x01 << 29)), AT91C_BASE_PIOA + PIO_PDR);

	pmc_enable_periph_clock(AT91C_ID_PIOA, PMC_PERIPH_CLK_DIVIDER_NA);
	// enable red led
	writel((0x01 << 29), AT91C_BASE_PIOA + PIO_OER);
}

static void initialize_dbgu(void)
{
	/* const struct pio_desc dbgu_pins[] = {
		{"RXD", AT91C_PIN_PA(9), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TXD", AT91C_PIN_PA(10), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	}; */
	/* Configure the dbgu pins */
	writel(((0x01 << 9) | (0x01 << 10)), AT91C_BASE_PIOA + PIO_ASR);
	writel(((0x01 << 9) | (0x01 << 10)), AT91C_BASE_PIOA + PIO_PDR);

	pmc_enable_periph_clock(AT91C_ID_PIOA, PMC_PERIPH_CLK_DIVIDER_NA);

	usart_init(BAUDRATE(MASTER_CLOCK, 115200));
}

#ifdef CONFIG_SDRAM
static void sdramc_init(void)
{
	struct sdramc_register sdramc_config;
	unsigned int reg;

#if defined(CONFIG_CPU_CLK_266MHZ)
	sdramc_config.cr = AT91C_SDRAMC_NC_9
			| AT91C_SDRAMC_NR_13 | AT91C_SDRAMC_CAS_3
			| AT91C_SDRAMC_NB_4_BANKS | AT91C_SDRAMC_DBW_32_BITS
			| AT91C_SDRAMC_TWR_2 | AT91C_SDRAMC_TRC_9
			| AT91C_SDRAMC_TRP_3 | AT91C_SDRAMC_TRCD_3
			| AT91C_SDRAMC_TRAS_6 | AT91C_SDRAMC_TXSR_10;

#endif
	sdramc_config.tr = (MASTER_CLOCK * 7) / 1000000;
	sdramc_config.mdr = AT91C_SDRAMC_MD_SDRAM;

	/* configure sdramc pins */
	writel(0xFFFF0000, AT91C_BASE_PIOC + PIO_ASR);
	writel(0xFFFF0000, AT91C_BASE_PIOC + PIO_PDR);

	pmc_enable_periph_clock(AT91C_ID_PIOC, PMC_PERIPH_CLK_DIVIDER_NA);

	/* Initialize the matrix (memory voltage = 3.3) */
	reg = readl(AT91C_BASE_CCFG + CCFG_EBICSA);
	reg |= AT91C_EBI_CS1A_SDRAMC;
	writel(reg, AT91C_BASE_CCFG + CCFG_EBICSA);

	sdramc_initialize(&sdramc_config, AT91C_BASE_CS1);
}
#endif  /* #ifdef CONFIG_SDRAM */

#if defined(CONFIG_NANDFLASH_RECOVERY) || defined(CONFIG_DATAFLASH_RECOVERY)
static void recovery_buttons_hw_init(void)
{
	/* Configure recovery button PINs */
	const struct pio_desc recovery_button_pins[] = {
		{"RECOVERY_BUTTON", CONFIG_SYS_RECOVERY_BUTTON_PIN, 0, PIO_PULLUP, PIO_INPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pmc_enable_periph_clock(AT91C_ID_PIOA, PMC_PERIPH_CLK_DIVIDER_NA);
	pio_configure(recovery_button_pins);
}
#endif /* #if defined(CONFIG_NANDFLASH_RECOVERY) || defined(CONFIG_DATAFLASH_RECOVERY) */

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	/* Disable watchdog */
	at91_disable_wdt();

	/*
	 * At this stage the main oscillator is supposed to be enabled
	 * PCK = MCK = MOSC
	 */

	/* Configure PLLA = MOSC * (PLL_MULA + 1) / PLL_DIVA */
	pmc_cfg_plla(PLLA_SETTINGS);

	/* PCK = PLLA = 2 * MCK */
	pmc_mck_cfg_set(0, MCKR_SETTINGS, AT91C_PMC_PRES | AT91C_PMC_MDIV);

	/* Switch MCK on PLLA output */
	pmc_mck_cfg_set(0, MCKR_CSS_SETTINGS,
			AT91C_PMC_PRES | AT91C_PMC_MDIV | AT91C_PMC_CSS);

	/* Enable External Reset */
	writel(AT91C_RSTC_KEY_UNLOCK | AT91C_RSTC_URSTEN, AT91C_BASE_RSTC + RSTC_RMR);

	/* Initialize matrix */
	at91_matrix_hw_init();

	/* Init timer */
	timer_init();

	/* Initialize dbgu */
	initialize_dbgu();

	/* Initialize led */
	led_hw_init();


#ifdef CONFIG_SDRAM
	/* Initlialize sdram controller */
	sdramc_init();
#endif

#if defined(CONFIG_NANDFLASH_RECOVERY) || defined(CONFIG_DATAFLASH_RECOVERY)
	/* Init the recovery buttons pins */
	recovery_buttons_hw_init();
#endif
}
#endif /* #ifdef CONFIG_HW_INIT */

#ifdef CONFIG_SDCARD
void at91_mci0_hw_init(void)
{
	/*
	const struct pio_desc mci_pins[] = {
		{"MCCK", AT91C_PIN_PA(2), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"MCCDA", AT91C_PIN_PA(1), 0, PIO_PULLUP, PIO_PERIPH_B},
		{"MCDA0", AT91C_PIN_PA(0), 0, PIO_PULLUP, PIO_PERIPH_B},
		{"MCDA1", AT91C_PIN_PA(4), 0, PIO_PULLUP, PIO_PERIPH_B},
		{"MCDA2", AT91C_PIN_PA(5), 0, PIO_PULLUP, PIO_PERIPH_B},
		{"MCDA3", AT91C_PIN_PA(6), 0, PIO_PULLUP, PIO_PERIPH_B},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_B},
	};
	*/

	/* configure mci0 pins */
	writel(((0x01 << 0) | (0x01 << 1) | (0x01 << 2) | (0x01 << 4)
		| (0x01 << 5) | (0x01 << 6)), AT91C_BASE_PIOA + PIO_BSR);
	writel(((0x01 << 0) | (0x01 << 1) | (0x01 << 2) | (0x01 << 4)
		| (0x01 << 5) | (0x01 << 6)), AT91C_BASE_PIOA + PIO_PDR);

	pmc_enable_periph_clock(AT91C_ID_PIOA, PMC_PERIPH_CLK_DIVIDER_NA);

	/* Enable the clock */
	pmc_enable_periph_clock(AT91C_ID_MCI, PMC_PERIPH_CLK_DIVIDER_NA);
}
#endif /* #ifdef CONFIG_SDCARD */
