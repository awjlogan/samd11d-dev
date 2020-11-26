/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
                 2020-2021 Angus Logan <awjlogan@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd11.h"
#include "hal_gpio.h"

#include "emonTxCM-samd.h"
#include "emonTxCM-conf.h"
#include "device_samd11.h"
#include "emonTxCM-lib.h"
#include "./printf/printf.h"

// ______________________________________________________PIN CONFIGURATION ____

/* Pin mapping: (SOIC20, WCLSP20, QFN24)
 *
 *  PA02 (AIN0)             Virtual ground
 *  PA03 (AIN1)             ADC Vref
 *  PA04-07, PA14 (AIN2-6)  Voltage inputs
 *  PA08-09 (SERCOM0)       UART
 *  PA22-25 (SERCOM1)       SPI     // NB remap if USB implemented
 *
 *  REVISIT - for higher pin count controller
 *  PAxx (GPIO)             Temperature input (one-wire)
 *  PAxx (GPIO)             Pulse counter
 *  PAxx ()
*/

HAL_GPIO_PIN(LED0,      A, 4)
HAL_GPIO_PIN(LED1,      A, 2)

HAL_GPIO_PIN(UART_TX,   A, 8)
HAL_GPIO_PIN(UART_RX,   A, 9)

/* Function prototypes */

static void timer_set_ticks (uint16_t ticks);
static void uart_send(const char *buffer, uint8_t len);

/* Variables visible to ISRs */
volatile int16_t    adc_results[NUM_I_CHAN + 1];  /* Accumulate ADC results, V channel always present */
volatile bool       results_ready = false;          /* All V/I channels sampled */
volatile bool       accumulated_ready = false;      /* All samples in period taken */

//-----------------------------------------------------------------------------
void irq_handler_tc1(void) {

    /* Start ADC conversion loop */
    if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC0) {
        // ADC->SWTRIG.reg |= ADC_SWTRIG_START;
    }

    /* Clear interrupt */
    TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
}

//-----------------------------------------------------------------------------
void irq_handler_dmac(void) {

    // REVISIT handle multple DMAC channels
    /* Check interrupt source */
    return;
}

//-----------------------------------------------------------------------------
void irq_handler_adc(void) {

    /* Track number of full conversions taken */
    static volatile uint16_t num_completed_conversions = 0;

    if (ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY) {

        /* Fill result array dependent on input INPUTCTRL.INPUTOFFSET */
        uint8_t input_channel = ADC->INPUTCTRL.bit.INPUTOFFSET;
        adc_results[input_channel] = ADC->RESULT.reg;

        /* When all channels have been sampled, do processing */
        if (input_channel == NUM_I_CHAN) {

            /* Set ready flag */
            results_ready = true;

            /* Disable and reset ADC for next sample */
            ADC->CTRLA.bit.ENABLE = false;
            while (ADC->STATUS.bit.SYNCBUSY);

            ADC->INPUTCTRL.bit.INPUTOFFSET = 0; // REVISIT is this needed?
        }

        /* Check for complete sampling */
        num_completed_conversions++;
        if (num_completed_conversions == SAMPLE_RATE) {
            accumulated_ready = true;
        }
    }

    /* Clear interrupt flag */
    ADC->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
}

//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
  while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE));
  SERCOM0->USART.DATA.reg = c;
}

//-----------------------------------------------------------------------------
static void uart_puts(const char *s)
{
  while (*s)
    uart_putc(*s++);
}

//-----------------------------------------------------------------------------
int main(void) {

    setup_device();

    HAL_GPIO_LED0_out();
    HAL_GPIO_LED0_clr();
    HAL_GPIO_LED1_out();
    HAL_GPIO_LED1_clr();

    HAL_GPIO_LED0_write(true);

    /*  Set up DMAC
     *  Descriptors must be in contiguous memory, Fig 19-3
     */
    // REVISIT refactor into set up function, should just be able to pass the
    // DMAC descriptor array
    // volatile DmacDescriptor DmacDescriptorArray[DMA_CHAN_IN_USE];
    // DmacDescriptor DmacDescriptorWBArray[DMA_CHAN_IN_USE];

    // /* ADC -> memory */
    // DMAC_BTCTRL_Type btctrl_adc_mem = {
    //     .bit.VALID = true,                              /* Descriptor valid */
    //     .bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val,   /* Event output selection */
    //     .bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_INT_Val,   /* Block action */
    //     .bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_HWORD_Val, /* Beat size */
    //     .bit.SRCINC = false,                            /* Source address increment enable */
    //     .bit.DSTINC = true,                             /* Destination address increment enable */
    //     .bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val,     /* Step selection */
    //     .bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val     /* Address increment step size */
    // };

    // DmacDescriptorArray[DMA_CHAN_ADC_MEM].BTCTRL.reg = btctrl_adc_mem.reg;
    // DmacDescriptorArray[DMA_CHAN_ADC_MEM].BTCNT.reg = (NUM_I_CHAN + 1);           /* Block transfer count */
    // DmacDescriptorArray[DMA_CHAN_ADC_MEM].SRCADDR.reg = (uint32_t)ADC + ADC_RESULT_OFFSET;    /* Transfer source address */
    // /*  DSTADDR should be the _last_ element in the destination array, Fig 19-9
    //  *  Half word transfers, so each increment is 2
    //  */
    // DmacDescriptorArray[DMA_CHAN_ADC_MEM].DSTADDR.reg = (uint32_t)&adc_results + (NUM_I_CHAN + 1) * 2;

    // DMAC->BASEADDR.reg = (uint32_t)DmacDescriptorArray;
    // DMAC->WRBADDR.reg = (uint32_t)DmacDescriptorWBArray;

    // Test of UART system

    struct EmonLibCM_Config cm_conf;
    struct EmonLibCM_Config *p_cm_conf = &cm_conf;
    cm_conf.ac_freq = 50U;
    cm_conf.adc_bits = 12U;

    volatile uint16_t raw_results[NUM_I_CHAN];
    (void)EmonLibCM_init(p_cm_conf, raw_results);

    struct EmonLibCM_Readings cm_readings;
    EmonLibCM_get_readings(&cm_readings);
    uart_puts(cm_readings.Vrms);

    // char unique_id_str[32]; // 128bits -> 32 hex characters + \0
    // volatile uint32_t unique_id_int;

    // /* Get unique ID number - Section 9.6 */
    // for (uint8_t i = 0; i < 4; i++) {
    //     unique_id_int = *(UNIQUE_ID + (i * 4));

    //     /* Convert to hex characters in 4bit chunks */
    //     for (uint8_t j = 0; i < 8; j++) {
    //         unique_id_str[((i * 8) + j)] = (unique_id_int & 0xF) + 65;  // A = 65
    //         unique_id_int >>= 4;
    //     }
    // }

    // uart_send(unique_id_str, 32);
    // uart_send("\r\n", 0);

    // // const uint8_t sw_version = VERSION_MAJOR << 6 | VERSION_MINOR << 3 | VERSION_POINT;
    // // char sw_version_str[2];

    // // sw_version_str[0] = (sw_version & 0xF) + 65;
    // // sw_version_str[1] = ((sw_version >> 4) & 0xF) + 65;
    // // uart_send(sw_version_str, 2);
    // // uart_send("\r\n", 0);

    // timer_set_ticks(32768);
    // TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

    while (1) {
        __WFI();
        uart_putc('.');
    }

    return 0;
}

//-----------------------------------------------------------------------------
static void timer_set_ticks(uint16_t ticks) {
    TC1->COUNT16.CC[0].reg = ticks;
    TC1->COUNT16.COUNT.reg = 0;
}

//-----------------------------------------------------------------------------
static void uart_send(const char *buffer, uint8_t len) {

    /* Can specify a length, or drain until null terminator encountered */
    /* Send _by length_ */
    // REVISIT specify maximum range of len
    if (len > 0) {
        for (uint8_t i = 0; i < len; i++) {
            uart_putc(buffer[i]);
        }
    } else {
        uart_puts(buffer);
    }
}

void _putchar(char character) {
    uart_putc(character);
}
