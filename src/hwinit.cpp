/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hwinit.h"

void rcc_clock_setup_in_hse_8mhz_out_64mhz(void)
{

    flash_prefetch_enable();
	/*
	 * Sysclk runs with 72MHz -> 2 waitstates.
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_ACR_LATENCY_2WS);

	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 8MHz. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	
	/*
	 * Set prescalers for AHB, ADC, APB1, APB2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);    /* Set. 64MHz Max. */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_DIV8);  /* Set.  8MHz Max.*/	
    rcc_set_ppre1(RCC_CFGR_PPRE_DIV2);     /* Set. 32MHz Max.*/
	rcc_set_ppre2(RCC_CFGR_PPRE_NODIV);    /* Set. 64MHz Max. */

    rcc_osc_off(RCC_PLL);

	/*
	 * Set the PLL multiplication factor to 8.
	 * 8MHz (external) * 8 (multiplier) = 64MHz
	 */	
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL8); //we need 64Mhz to get 2Mhz SPI!!
    //rcc_set_pll2_multiplication_factor(RCC_CFGR2_PREDIV1SRC_HSE_CLK);
    //rcc_set_pll3_multiplication_factor();
    
	/* Select HSE as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	/*
	 * External frequency undivided before entering PLL
	 * (only valid/needed for HSE).
	 */
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency  = 64000000;
	rcc_apb1_frequency = 32000000;
	rcc_apb2_frequency = 64000000;

}

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{

    rcc_clock_setup_in_hse_8mhz_out_64mhz();

//Activate PORTS
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
 
    // rcc_periph_clock_enable(RCC_TIM1); //GS450H oil pump pwm
    // rcc_periph_clock_enable(RCC_TIM2); //GS450H 500khz usart clock
    // rcc_periph_clock_enable(RCC_TIM3); //Scheduler
    // rcc_periph_clock_enable(RCC_TIM4); //
    
    // rcc_periph_clock_enable(RCC_DMA1);  //ADC, and UARTS
    // rcc_periph_clock_enable(RCC_DMA2);

    rcc_periph_clock_enable(RCC_CRC);
    rcc_periph_clock_enable(RCC_AFIO); //CAN AND USART3
    rcc_periph_clock_enable(RCC_CAN1); //CAN1
    rcc_periph_clock_enable(RCC_CAN2); //CAN2

}


/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xe << 4); //second lowest priority
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); //CAN RX
    
    nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xe << 4); //second lowest priority
    nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ); //CAN TX
}

void rtc_setup()
{
    //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
    //62.5kHz / (624 + 1) = 100Hz
    rtc_auto_awake(RCC_HSE, 624); //10ms tick
    rtc_set_counter_val(0);
    //* Enable the RTC interrupt to occur off the SEC flag.
    rtc_clear_flag(RTC_SEC);
	rtc_interrupt_enable(RTC_SEC);
}

//Left over for future usage in an inverter style device..
// Todo: read an application configuration based on fingerprint?
//
void initialize_pins()
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* pincommands = (struct pincommands*)pindefAddr;

   uint32_t crc = crc_calculate_block(((uint32_t*)pincommands), PINDEF_NUMWORDS);

   if (crc == pincommands->crc)
   {
      for (int idx = 0; idx < NUM_PIN_COMMANDS && pincommands->pindef[idx].port > 0; idx++)
      {
         uint8_t cnf = pincommands->pindef[idx].inout ? GPIO_CNF_OUTPUT_PUSHPULL : GPIO_CNF_INPUT_PULL_UPDOWN;
         uint8_t mode = pincommands->pindef[idx].inout ? GPIO_MODE_OUTPUT_50_MHZ : GPIO_MODE_INPUT;
         gpio_set_mode(pincommands->pindef[idx].port, mode, cnf, pincommands->pindef[idx].pin);

         if (pincommands->pindef[idx].level)
         {
            gpio_set(pincommands->pindef[idx].port, pincommands->pindef[idx].pin);
         }
      }
   }
}