/** @defgroup rcc_file RCC peripheral API

@ingroup peripheral_apis

@brief <b>libopencm3 STM32F1xx Reset and Clock Control</b>

@version 1.0.0

@author @htmlonly &copy; @endhtmlonly 2009
Federico Ruiz-Ugalde \<memeruiz at gmail dot com\>
@author @htmlonly &copy; @endhtmlonly 2009 Uwe Hermann <uwe@hermann-uwe.de>
@author @htmlonly &copy; @endhtmlonly 2010 Thomas Otto <tommi@viadmin.org>

@date 18 August 2012

This library supports the Reset and Clock Control System in the STM32F1xx
series of ARM Cortex Microcontrollers by ST Microelectronics.

@note Full support for connection line devices is not yet provided.

Clock settings and resets for many peripherals are given here rather than in
the corresponding peripheral library.

The library also provides a number of common configurations for the processor
system clock. Not all possible configurations are included.

LGPL License Terms @ref lgpl_license
 */
/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Federico Ruiz-Ugalde <memeruiz at gmail dot com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**@{*/

#include <libopencm3/cm3/assert.h>
#include <libopencm3/air32/rcc.h>
#include <libopencm3/air32/flash.h>

/** Set the default clock frequencies */
uint32_t rcc_apb1_frequency = 8000000;
uint32_t rcc_apb2_frequency = 8000000;
uint32_t rcc_ahb_frequency = 8000000;

const struct rcc_clock_scale rcc_hse_configs[RCC_CLOCK_HSE_END] = {
	{
		/* hse-12, pll to 72 */
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL6,
		.prediv1 = RCC_CFGR2_PREDIV_NODIV,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV6,
		.flash_waitstates = 2,
		.ahb_frequency	= 72000000,
		.apb1_frequency = 36000000,
		.apb2_frequency = 72000000,
	},
	{
		/* hse16, pll to 72 */
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV6,
		.flash_waitstates = 2,
		.prediv1 = RCC_CFGR2_PREDIV_DIV2,
		.ahb_frequency = 72e6,
		.apb1_frequency = 36e6,
		.apb2_frequency = 72e6,
	},
	{
		/* hse25 to 72, this was a f105 config originally! intention preserved */
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,
		.pll_source = RCC_CFGR_PLLSRC_PREDIV1_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV6,
		.flash_waitstates = 2,
		.prediv1 = RCC_CFGR2_PREDIV_DIV5,
		.prediv1_source = RCC_CFGR2_PREDIV1SRC_PLL2_CLK,
		.pll2_mul = RCC_CFGR2_PLL2MUL_PLL2_CLK_MUL8,
		.prediv2 = RCC_CFGR2_PREDIV2_DIV5,
		.usbpre = RCC_CFGR_USBPRE_PLL_VCO_CLK_DIV3,
		.ahb_frequency = 72e6,
		.apb1_frequency = 36e6,
		.apb2_frequency = 72e6,
	},
	{
		/* hse8, pll to 24 (f100 value line max) */
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL3,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_NODIV,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV2,
		.flash_waitstates = 0,
		.prediv1 = RCC_CFGR2_PREDIV_NODIV,
		.ahb_frequency = 24e6,
		.apb1_frequency = 24e6,
		.apb2_frequency = 24e6,
	},
	{
		/* hse8, pll to 72 */
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL9,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV8,
		.flash_waitstates = 2,
		.prediv1 = RCC_CFGR2_PREDIV_NODIV,
		.ahb_frequency = 72e6,
		.apb1_frequency = 36e6,
		.apb2_frequency = 72e6,
	},
};

const struct rcc_clock_scale rcc_hsi_configs[RCC_CLOCK_HSI_END] = {
	{
		/* hsi to 24Mhz, max for f100 */
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL6,
		.prediv1 = RCC_CFGR2_PREDIV_NODIV,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_NODIV,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV2,
		.flash_waitstates = 0,
		.ahb_frequency	= 24e6,
		.apb1_frequency = 24e6,
		.apb2_frequency = 24e6,
	},
	{
		/* hsi to 48Mhz, allows usb, but out of spec */
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL12,
		.prediv1 = RCC_CFGR2_PREDIV_NODIV,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV8,
		.usbpre = RCC_CFGR_USBPRE_PLL_CLK_NODIV,
		.flash_waitstates = 1,
		.ahb_frequency	= 48e6,
		.apb1_frequency = 24e6,
		.apb2_frequency = 48e6,
	},
	{
		/* hsi to 64Mhz, max possible from hsi */
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK_DIV2,
		.pll_mul = RCC_CFGR_PLLMUL_PLL_CLK_MUL16,
		.prediv1 = RCC_CFGR2_PREDIV_NODIV,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.adcpre = RCC_CFGR_ADCPRE_DIV8,
		.flash_waitstates = 2,
		.ahb_frequency	= 64e6,
		.apb1_frequency = 32e6,
		.apb2_frequency = 64e6,
	},
};

/*---------------------------------------------------------------------------*/
/** @brief RCC Clear the Oscillator Ready Interrupt Flag

Clear the interrupt flag that was set when a clock oscillator became ready to
use.

@param[in] osc Oscillator ID
*/

void rcc_osc_ready_int_clear(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR |= RCC_CIR_PLLRDYC;
		break;
	case RCC_PLL2:
		RCC_CIR |= RCC_CIR_PLL2RDYC;
		break;
	case RCC_PLL3:
		RCC_CIR |= RCC_CIR_PLL3RDYC;
		break;
	case RCC_HSE:
		RCC_CIR |= RCC_CIR_HSERDYC;
		break;
	case RCC_HSI:
		RCC_CIR |= RCC_CIR_HSIRDYC;
		break;
	case RCC_LSE:
		RCC_CIR |= RCC_CIR_LSERDYC;
		break;
	case RCC_LSI:
		RCC_CIR |= RCC_CIR_LSIRDYC;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Enable the Oscillator Ready Interrupt

@param osc Oscillator ID
*/

void rcc_osc_ready_int_enable(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR |= RCC_CIR_PLLRDYIE;
		break;
	case RCC_PLL2:
		RCC_CIR |= RCC_CIR_PLL2RDYIE;
		break;
	case RCC_PLL3:
		RCC_CIR |= RCC_CIR_PLL3RDYIE;
		break;
	case RCC_HSE:
		RCC_CIR |= RCC_CIR_HSERDYIE;
		break;
	case RCC_HSI:
		RCC_CIR |= RCC_CIR_HSIRDYIE;
		break;
	case RCC_LSE:
		RCC_CIR |= RCC_CIR_LSERDYIE;
		break;
	case RCC_LSI:
		RCC_CIR |= RCC_CIR_LSIRDYIE;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Disable the Oscillator Ready Interrupt

@param[in] osc Oscillator ID
*/

void rcc_osc_ready_int_disable(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CIR &= ~RCC_CIR_PLLRDYIE;
		break;
	case RCC_PLL2:
		RCC_CIR &= ~RCC_CIR_PLL2RDYIE;
		break;
	case RCC_PLL3:
		RCC_CIR &= ~RCC_CIR_PLL3RDYIE;
		break;
	case RCC_HSE:
		RCC_CIR &= ~RCC_CIR_HSERDYIE;
		break;
	case RCC_HSI:
		RCC_CIR &= ~RCC_CIR_HSIRDYIE;
		break;
	case RCC_LSE:
		RCC_CIR &= ~RCC_CIR_LSERDYIE;
		break;
	case RCC_LSI:
		RCC_CIR &= ~RCC_CIR_LSIRDYIE;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Read the Oscillator Ready Interrupt Flag

@param[in] osc Oscillator ID
@returns int. Boolean value for flag set.
*/

int rcc_osc_ready_int_flag(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		return ((RCC_CIR & RCC_CIR_PLLRDYF) != 0);
		break;
	case RCC_PLL2:
		return ((RCC_CIR & RCC_CIR_PLL2RDYF) != 0);
		break;
	case RCC_PLL3:
		return ((RCC_CIR & RCC_CIR_PLL3RDYF) != 0);
		break;
	case RCC_HSE:
		return ((RCC_CIR & RCC_CIR_HSERDYF) != 0);
		break;
	case RCC_HSI:
		return ((RCC_CIR & RCC_CIR_HSIRDYF) != 0);
		break;
	case RCC_LSE:
		return ((RCC_CIR & RCC_CIR_LSERDYF) != 0);
		break;
	case RCC_LSI:
		return ((RCC_CIR & RCC_CIR_LSIRDYF) != 0);
		break;
	}

	cm3_assert_not_reached();
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Clear the Clock Security System Interrupt Flag

*/

void rcc_css_int_clear(void)
{
	RCC_CIR |= RCC_CIR_CSSC;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Read the Clock Security System Interrupt Flag

@returns int. Boolean value for flag set.
*/

int rcc_css_int_flag(void)
{
	return ((RCC_CIR & RCC_CIR_CSSF) != 0);
}

bool rcc_is_osc_ready(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		return RCC_CR & RCC_CR_PLLRDY;
	case RCC_PLL2:
		return RCC_CR & RCC_CR_PLL2RDY;
	case RCC_PLL3:
		return RCC_CR & RCC_CR_PLL3RDY;
	case RCC_HSE:
		return RCC_CR & RCC_CR_HSERDY;
	case RCC_HSI:
		return RCC_CR & RCC_CR_HSIRDY;
	case RCC_LSE:
		return RCC_BDCR & RCC_BDCR_LSERDY;
	case RCC_LSI:
		return RCC_CSR & RCC_CSR_LSIRDY;
	}
	return false;
}

void rcc_wait_for_osc_ready(enum rcc_osc osc)
{
	while (!rcc_is_osc_ready(osc));
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Turn on an Oscillator.

Enable an oscillator and power on. Each oscillator requires an amount of time
to settle to a usable state. Refer to datasheets for time delay information. A
status flag is available to indicate when the oscillator becomes ready (see
@ref rcc_osc_ready_int_flag and @ref rcc_wait_for_osc_ready).

@note The LSE clock is in the backup domain and cannot be enabled until the
backup domain write protection has been removed (see @ref
pwr_disable_backup_domain_write_protect).

@param[in] osc Oscillator ID
*/

void rcc_osc_on(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CR |= RCC_CR_PLLON;
		break;
	case RCC_PLL2:
		RCC_CR |= RCC_CR_PLL2ON;
		break;
	case RCC_PLL3:
		RCC_CR |= RCC_CR_PLL3ON;
		break;
	case RCC_HSE:
		RCC_CR |= RCC_CR_HSEON;
		break;
	case RCC_HSI:
		RCC_CR |= RCC_CR_HSION;
		break;
	case RCC_LSE:
		RCC_BDCR |= RCC_BDCR_LSEON;
		break;
	case RCC_LSI:
		RCC_CSR |= RCC_CSR_LSION;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Turn off an Oscillator.

Disable an oscillator and power off.

@note An oscillator cannot be turned off if it is selected as the system clock.
@note The LSE clock is in the backup domain and cannot be disabled until the
backup domain write protection has been removed (see
@ref pwr_disable_backup_domain_write_protect) or the backup domain has been
(see reset @ref rcc_backupdomain_reset).

@param[in] osc Oscillator ID
*/

void rcc_osc_off(enum rcc_osc osc)
{
	switch (osc) {
	case RCC_PLL:
		RCC_CR &= ~RCC_CR_PLLON;
		break;
	case RCC_PLL2:
		RCC_CR &= ~RCC_CR_PLL2ON;
		break;
	case RCC_PLL3:
		RCC_CR &= ~RCC_CR_PLL3ON;
		break;
	case RCC_HSE:
		RCC_CR &= ~RCC_CR_HSEON;
		break;
	case RCC_HSI:
		RCC_CR &= ~RCC_CR_HSION;
		break;
	case RCC_LSE:
		RCC_BDCR &= ~RCC_BDCR_LSEON;
		break;
	case RCC_LSI:
		RCC_CSR &= ~RCC_CSR_LSION;
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Enable the Clock Security System.

*/

void rcc_css_enable(void)
{
	RCC_CR |= RCC_CR_CSSON;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Disable the Clock Security System.

*/

void rcc_css_disable(void)
{
	RCC_CR &= ~RCC_CR_CSSON;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the Source for the System Clock.

@param[in] clk Unsigned int32. System Clock Selection @ref rcc_cfgr_scs
*/

void rcc_set_sysclk_source(uint32_t clk)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW) |
			(clk << RCC_CFGR_SW_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL Multiplication Factor.

@note This only has effect when the PLL is disabled.

@param[in] mul Unsigned int32. PLL multiplication factor @ref rcc_cfgr_pmf
*/

void rcc_set_pll_multiplication_factor(uint32_t mul)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PLLMUL) |
			(mul << RCC_CFGR_PLLMUL_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL2 Multiplication Factor.

@note This only has effect when the PLL is disabled.

@param[in] mul Unsigned int32. PLL multiplication factor @ref rcc_cfgr_pmf
*/

void rcc_set_pll2_multiplication_factor(uint32_t mul)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PLL2MUL) |
			(mul << RCC_CFGR2_PLL2MUL_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL3 Multiplication Factor.

@note This only has effect when the PLL is disabled.

@param[in] mul Unsigned int32. PLL multiplication factor @ref rcc_cfgr_pmf
*/

void rcc_set_pll3_multiplication_factor(uint32_t mul)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PLL3MUL) |
			(mul << RCC_CFGR2_PLL3MUL_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the PLL Clock Source.

@note This only has effect when the PLL is disabled.

@param[in] pllsrc Unsigned int32. PLL clock source @ref rcc_cfgr_pcs
*/

void rcc_set_pll_source(uint32_t pllsrc)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PLLSRC) |
			(pllsrc << 16);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the HSE Frequency Divider used as PLL Clock Source.

@note This only has effect when the PLL is disabled.

@param[in] pllxtpre Unsigned int32. HSE division factor @ref rcc_cfgr_hsepre
*/

void rcc_set_pllxtpre(uint32_t pllxtpre)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PLLXTPRE) |
			(pllxtpre << 17);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC RTC Clock Enabled Flag

@returns uint32_t. Nonzero if the RTC Clock is enabled.
*/

uint32_t rcc_rtc_clock_enabled_flag(void)
{
	return RCC_BDCR & RCC_BDCR_RTCEN;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Enable the RTC clock

*/

void rcc_enable_rtc_clock(void)
{
	RCC_BDCR |= RCC_BDCR_RTCEN;
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the Source for the RTC clock

@param[in] clock_source RTC clock source. Only HSE/128, LSE and LSI.
*/

void rcc_set_rtc_clock_source(enum rcc_osc clock_source)
{
	uint32_t reg32;

	switch (clock_source) {
	case RCC_LSE:
		/* Turn the LSE on and wait while it stabilises. */
		RCC_BDCR |= RCC_BDCR_LSEON;
		while ((reg32 = (RCC_BDCR & RCC_BDCR_LSERDY)) == 0);

		/* Choose LSE as the RTC clock source. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		RCC_BDCR |= (1 << 8);
		break;
	case RCC_LSI:
		/* Turn the LSI on and wait while it stabilises. */
		RCC_CSR |= RCC_CSR_LSION;
		while ((reg32 = (RCC_CSR & RCC_CSR_LSIRDY)) == 0);

		/* Choose LSI as the RTC clock source. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		RCC_BDCR |= (1 << 9);
		break;
	case RCC_HSE:
		/* Turn the HSE on and wait while it stabilises. */
		RCC_CR |= RCC_CR_HSEON;
		while ((reg32 = (RCC_CR & RCC_CR_HSERDY)) == 0);

		/* Choose HSE as the RTC clock source. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		RCC_BDCR |= (1 << 9) | (1 << 8);
		break;
	case RCC_PLL:
	case RCC_PLL2:
	case RCC_PLL3:
	case RCC_HSI:
		/* Unusable clock source, here to prevent warnings. */
		/* Turn off clock sources to RTC. */
		RCC_BDCR &= ~((1 << 8) | (1 << 9));
		break;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief ADC Setup the A/D Clock

The ADC's have a common clock prescale setting.

@param[in] adcpre uint32_t. Prescale divider taken from @ref rcc_cfgr_adcpre
*/

void rcc_set_adcpre(uint32_t adcpre)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_ADCPRE) |
			(adcpre << RCC_CFGR_ADCPRE_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the APB2 Prescale Factor.

@param[in] ppre2 Unsigned int32. APB2 prescale factor @ref rcc_cfgr_apb2pre
*/

void rcc_set_ppre2(uint32_t ppre2)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PPRE2) |
			(ppre2 << RCC_CFGR_PPRE2_SHIFT);
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the APB1 Prescale Factor.

@note The APB1 clock frequency must not exceed 36MHz.

@param[in] ppre1 Unsigned int32. APB1 prescale factor @ref rcc_cfgr_apb1pre
*/

void rcc_set_ppre1(uint32_t ppre1)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_PPRE1) |
			(ppre1 << RCC_CFGR_PPRE1_SHIFT);

}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the AHB Prescale Factor.

@param[in] hpre Unsigned int32. AHB prescale factor @ref rcc_cfgr_ahbpre
*/

void rcc_set_hpre(uint32_t hpre)
{
	RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_HPRE) |
			(hpre << RCC_CFGR_HPRE_SHIFT);

}

/*---------------------------------------------------------------------------*/
/** @brief RCC Set the USB Prescale Factor.

The prescale factor can be set to 1 (no prescale) for use when the PLL clock is
48MHz, or 1.5 to generate the 48MHz USB clock from a 72MHz PLL clock.

@note This bit cannot be reset while the USB clock is enabled.

@param[in] usbpre Unsigned int32. USB prescale factor @ref rcc_cfgr_usbpre
*/

void rcc_set_usbpre(uint32_t usbpre)
{

  RCC_CFGR |= (RCC_CFGR & ~RCC_CFGR_USBPRE) |
    usbpre << RCC_CFGR_USB_SHIFT  ;

}

void rcc_set_prediv1(uint32_t prediv)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PREDIV1) |
			(prediv << RCC_CFGR2_PREDIV1_SHIFT);
}

void rcc_set_prediv2(uint32_t prediv)
{
	RCC_CFGR2 = (RCC_CFGR2 & ~RCC_CFGR2_PREDIV2) |
			(prediv << RCC_CFGR2_PREDIV2_SHIFT);
}

void rcc_set_prediv1_source(uint32_t rccsrc)
{
	if (rccsrc) {
		RCC_CFGR2 |= RCC_CFGR2_PREDIV1SRC;
	} else {
		RCC_CFGR2 &= ~RCC_CFGR2_PREDIV1SRC;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief RCC Get the System Clock Source.

@returns Unsigned int32. System clock source:
@li 00 indicates HSE
@li 01 indicates LSE
@li 02 indicates PLL
*/

uint32_t rcc_system_clock_source(void)
{
	/* Return the clock source which is used as system clock. */
	return (RCC_CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_SHIFT;
}

/*---------------------------------------------------------------------------*/
/*
 * These functions are setting up the whole clock system for the most common
 * input clock and output clock configurations.
 */
/*---------------------------------------------------------------------------*/

void rcc_clock_setup_pll(const struct rcc_clock_scale *clock)
{
  RCC_CIR = 0;
  // clock sec off
  RCC_CR &= ~RCC_CR_CSSON;
  // do the config under HSI
  rcc_osc_on(RCC_HSI);	
  rcc_wait_for_osc_ready(RCC_HSI);
  rcc_osc_on(RCC_LSI);	
  rcc_wait_for_osc_ready(RCC_LSI);
  RCC_CFGR &= ~RCC_CFGR_SW_SYSCLKSEL_MASK ; // use HSI
  // wait for HSI to be SYSCLK
  while((RCC_CFGR & (RCC_CFGR_SWS_SYSCLKSEL_MASK<<RCC_CFGR_SWS_SYSCLKSEL_SHIFT))!=0);
  rcc_osc_off(RCC_HSE);
  rcc_osc_off(RCC_PLL);
  while(RCC_CR & RCC_CR_HSERDY);	
  
  FLASH_ACR |=  clock->flash_waitstates ; // 1 ws
  //FLASH_ACR |= 1<<5; // enable prefetch


  if(clock->pll_mul > RCC_MCO_VAL_PLLDIV_16){
    // bullshit secret registers knock unlock
    *(volatile uint32_t *)(0x400210F0) = 1;//开启sys_cfg门控
    *(volatile uint32_t *)(0x40016C00) = 0xa7d93a86;//解一、二、三级锁
    *(volatile uint32_t *)(0x40016C00) = 0xab12dfcd;
    *(volatile uint32_t *)(0x40016C00) = 0xcded3526;
    *(volatile uint32_t *)(0x4002228C) = 0xa5a5a5a5;//QSPI解锁
    *(volatile uint32_t *)(0x40022214) |= 1;
    *(volatile uint32_t *)(0x40016C18) = 0x200183FF;
  }  
  


  
  rcc_set_pll_multiplication_factor(clock->pll_mul);
  rcc_set_pll_source(clock->pll_source);
  
  rcc_set_hpre(clock->hpre);
  rcc_set_ppre1(clock->ppre1);
  rcc_set_ppre2(clock->ppre2);
  rcc_set_adcpre(clock->adcpre);
  rcc_set_usbpre(clock->usbpre);


  

  /*
   * Magically handle F105/7 parts too.
   * xtpre == prediv1 bit 0.
   */
  if (clock->prediv1 > RCC_CFGR2_PREDIV_DIV2) {
    rcc_set_prediv1(clock->prediv1);
  } else {
    rcc_set_pllxtpre(clock->prediv1);
  }
  if (clock->prediv1_source) {
    rcc_set_prediv1_source(clock->prediv1_source);
  }

  /*
   * Magically handle other plls/prescalers on other parts
   */
  /*if (clock->prediv2) {
    rcc_set_prediv2(clock->prediv2);
    }*/
  /*
    if (clock->pll2_mul) {
    rcc_set_pll2_multiplication_factor(clock->pll2_mul);
    rcc_osc_on(RCC_PLL2);
    rcc_wait_for_osc_ready(RCC_PLL2);
    }
    if (clock->pll3_mul) {
    rcc_set_pll3_multiplication_factor(clock->pll3_mul);
    rcc_osc_on(RCC_PLL3);
    rcc_wait_for_osc_ready(RCC_PLL3);
    }
  */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
  if (clock->pll_source == RCC_CFGR_PLLSRC_HSE_CLK) {
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
  } else {
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);
  }


  /* Enable PLL oscillator and wait for it to stabilize. */
  rcc_osc_on(RCC_PLL);
  rcc_wait_for_osc_ready(RCC_PLL);

  /* Select PLL as SYSCLK source. */
  while((RCC_CFGR & (RCC_CFGR_SWS_SYSCLKSEL_MASK<<RCC_CFGR_SWS_SYSCLKSEL_SHIFT))==0);

  /* Set the peripheral clock frequencies used. */
  rcc_ahb_frequency  = clock->ahb_frequency;
  rcc_apb1_frequency = clock->apb1_frequency;
  rcc_apb2_frequency = clock->apb2_frequency;
  if(clock->pll_mul > RCC_MCO_VAL_PLLDIV_16){
    // bullshit secret registers knock relock
    *(volatile uint32_t *)(0x400210F0) = 0;//开启sys_cfg门控
    *(volatile uint32_t *)(0x40016C00) = ~0xa7d93a86;//加一、二、三级锁
    *(volatile uint32_t *)(0x40016C00) = ~0xab12dfcd;
    *(volatile uint32_t *)(0x40016C00) = ~0xcded3526;
    *(volatile uint32_t *)(0x4002228C) = ~0xa5a5a5a5;//QSPI解锁
  }

  
  // RCC_CR |= RCC_CR_CSSON;
}


/*---------------------------------------------------------------------------*/
/** @brief RCC Reset the Backup Domain

The backup domain registers are reset to disable RTC controls and clear user
data.
*/

void rcc_backupdomain_reset(void)
{
	/* Set the backup domain software reset. */
	RCC_BDCR |= RCC_BDCR_BDRST;

	/* Clear the backup domain software reset. */
	RCC_BDCR &= ~RCC_BDCR_BDRST;
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the USART at base specified.
 * @param usart  Base address of USART to get clock frequency for.
 */
uint32_t rcc_get_usart_clk_freq(uint32_t usart)
{
	if (usart == USART1_BASE) {
		return rcc_apb2_frequency;
	} else {
		return rcc_apb1_frequency;
	}
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the Timer at base specified.
 * @param timer  Base address of TIM to get clock frequency for.
 */
uint32_t rcc_get_timer_clk_freq(uint32_t timer)
{
	/* Handle APB1 timer clocks. */
	if (timer >= TIM2_BASE && timer <= TIM14_BASE) {
		uint8_t ppre1 = (RCC_CFGR >> RCC_CFGR_PPRE1_SHIFT) & RCC_CFGR_PPRE1_MASK;
		return (ppre1 == RCC_CFGR_PPRE1_HCLK_NODIV) ? rcc_apb1_frequency
			: 2 * rcc_apb1_frequency;
	} else {
		uint8_t ppre2 = (RCC_CFGR >> RCC_CFGR_PPRE2_SHIFT) & RCC_CFGR_PPRE2_MASK;
		return (ppre2 == RCC_CFGR_PPRE2_HCLK_NODIV) ? rcc_apb2_frequency
			: 2 * rcc_apb2_frequency;
	}
	cm3_assert_not_reached();
}

/*---------------------------------------------------------------------------*/
/** @brief Get the peripheral clock speed for the I2C device at base specified.
 * @param i2c  Base address of I2C to get clock frequency for.
 */
uint32_t rcc_get_i2c_clk_freq(uint32_t i2c __attribute__((unused)))
{
	return rcc_apb1_frequency;
}
/**@}*/

