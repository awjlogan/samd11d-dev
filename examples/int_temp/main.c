/*
 * Copyright (c) 2020, Angus Logan <awjlogan@gmail.com>
                 2016, Alex Taradov <alex@taradov.com>
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
#include "samd-dev.h"


//-----------------------------------------------------------------------------
#define SAMPLE_PERIOD   10000ul /* ms */
#define SAMPLE_PERIOD_S SAMPLE_PERIOD / 1000ul
#define ADC_RESOLUTION  4096
#define ADC_VREF        1.0

HAL_GPIO_PIN(LED0,      A, 14)
HAL_GPIO_PIN(LED1,      A, 15)
HAL_GPIO_PIN(BUTTON,    A, 16)
HAL_GPIO_PIN(UART_TX,   A, 8)
HAL_GPIO_PIN(UART_RX,   A, 9)


//-----------------------------------------------------------------------------
static void adc_init(void) {

    /* APB C Mask reset to 1 for ADC (15.8.11) */
    // PM->APBCMASK.reg |= PM_APBCMASK_ADC;

    /* Set up clock source */
    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = ADC_GCLK_ID,  /* Generic clock selection ID */
        .bit.GEN = 0,           /* Generic clock generator */
        .bit.CLKEN = true,      /* Clock enable */
        .bit.WRTLOCK = false    /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

    /* REFCTRL -> reset values correct, INT1V is reference source */
    /* Sampling length control (57 us) */
    ADC_SAMPCTRL_Type sampctrl = {
        .bit.SAMPLEN = 0x1E
    };
    ADC->SAMPCTRL.reg = sampctrl.reg;

    ADC_CTRLB_Type ctrlb = {
        .bit.DIFFMODE = false,                          /* Differential mode */
        .bit.LEFTADJ = false,                           /* Left-adjusted result */
        .bit.FREERUN = false,                           /* Free running mode */
        .bit.CORREN = false,                            /* Digital correction logic enabled */
        .bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val,       /* Conversion result resolution */
        .bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val   /* Prescaler configuration */
    };
    ADC->CTRLB.reg = ctrlb.reg;
    while (ADC->STATUS.bit.SYNCBUSY);   // Requires sync on write (31.6.13)

    ADC_INPUTCTRL_Type inputctrl = {
        .bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_TEMP_Val,    /* Positive Mux input selection */
        .bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val,     /* Negative Mux Input Selection */
        .bit.INPUTSCAN = 0,                             /* Number of input channels included in scan */
        .bit.INPUTOFFSET = 0,                           /* Positive mux setting offset */
        .bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val           /* Gain factor selection */
    };
    ADC->INPUTCTRL.reg = inputctrl.reg;
    while (ADC->STATUS.bit.SYNCBUSY);   // Requires sync on write (31.6.13)

    /* Set up averaging (31.6.7) */
    ADC_AVGCTRL_Type avgctrl = {
        .bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_8_Val,   /* Number of samples to be collected */
        .bit.ADJRES = 0x3                               /* Adjusting result / division coefficient */
    };
    ADC->AVGCTRL.reg = avgctrl.reg;

    /* Load factory calibration values from NVM (9.5) */
    uint32_t nvm_calib_low = *(volatile uint32_t *) 0x00806020;
    uint32_t nvm_calib_high = *(volatile uint32_t *) 0x00806024;

    ADC_CALIB_Type calib = {
        .bit.LINEARITY_CAL = ((nvm_calib_high & 0x3) << 5) | (nvm_calib_low >> 27),
        .bit.BIAS_CAL = (nvm_calib_low >> 4) & 0x7
    };
    ADC->CALIB.reg = calib.reg;
}


//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud) {

    uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

    HAL_GPIO_UART_TX_out();
    HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_D_Val); // SERCOM-ALT
    HAL_GPIO_UART_RX_in();
    HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_D_Val); // SERCOM-ALT

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    /* Set up clock source */
    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = SERCOM0_GCLK_ID_CORE, /* Generic clock selection ID */
        .bit.GEN = 0,                   /* Generic clock generator */
        .bit.CLKEN = true,              /* Clock enable */
        .bit.WRTLOCK = false            /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

    /* Set up USART controls A (25.8.1) */
    SERCOM_USART_CTRLA_Type ctrla = {
        .bit.SWRST = false,                                     /* Software reset */
        .bit.ENABLE = false,                                    /* Enable */
        .bit.MODE = SERCOM_USART_CTRLA_MODE_USART_INT_CLK_Val,  /* Operating mode */
        .bit.RUNSTDBY = false,                                  /* Run during standby */
        .bit.IBON = false,                                      /* Immediate buffer overflow notification */
        .bit.SAMPR = 0,                                         /* Sample */
        .bit.TXPO = 0,                                          /* TX pinout (0) */
        .bit.RXPO = 2,                                          /* RX pinout (2) */
        .bit.SAMPA = 0,                                         /* Sample adjustment */
        .bit.FORM = 0,                                          /* Frame format (USART frame) */
        .bit.CMODE = 0,                                         /* Communication mode */
        .bit.CPOL = 0,                                          /* Clock polarity */
        .bit.DORD = 1                                           /* Data order (LSB first) */
    };
    SERCOM0->USART.CTRLA.reg = ctrla.reg;

    /* Set up USART controls B (25.8.2) */
    SERCOM_USART_CTRLB_Type ctrlb = {
        .bit.CHSIZE = 0,                                        /* Character size (8) */
        .bit.SBMODE = 0,                                        /* Stop bit mode */
        .bit.COLDEN = false,                                    /* Collision detection enable */
        .bit.SFDE = false,                                      /* Start of frame detection enable */
        .bit.ENC = false,                                       /* IR Encoding */
        .bit.PMODE = 0,                                         /* Parity mode (even) */
        .bit.TXEN = true,                                       /* TX enable */
        .bit.RXEN = true                                        /* RX enable */
    };
    SERCOM0->USART.CTRLB.reg = ctrlb.reg;

    /* Set up baud rate, and enable */
    SERCOM0->USART.BAUD.reg = (uint16_t)br+1;
    SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}


//-----------------------------------------------------------------------------
void adc_to_temp_str(char *buf, uint16_t adc_res) {

    /* Temperature sensor characteristics (35.10.8)
     * Absolute:    0.688 V at 25 C
     * T variation: 2.16 +/- 0.1 mV/C => 8.8 LSB/C
    */

    /* Get temperature from code by simple linear fit
     * Assuming INT1V == 1.000 V, and ADC is perfect:
     *
     * T = (ADC - 2598) / 8.8
     *
     * REVISIT Use calibration method from datasheet
     */

    uint8_t decimals[3] = {};
    float temperature = (float)adc_res - 2598.0f;
    temperature = temperature / 8.8f;
    
    /* Convert f -> decimal value 
     * Assume 100 > T > 0 
     */
    temperature = temperature * 10.0;
    for (uint8_t i = 3; i > 0; i--) {
        float sub_val = 1.0f;
        for (uint8_t j = 0; j < (i - 1); j++) {
            sub_val = sub_val * 10;
        }
        while ((temperature - sub_val) > 0.0) {
            temperature = temperature - sub_val;
            decimals[i - 1]++;
        }
    }

    /* Pack into string */
    buf[0] = decimals[2] + '0';
    buf[1] = decimals[1] + '0';
    buf[2] = '.';
    buf[3] = decimals[0] + '0';
    buf[4] = '\0';
}

//-----------------------------------------------------------------------------
int main(void) {

    const uint32_t max_count = 500000;

    uint32_t count = 0;
    uint16_t result = 0;
    char tx_buf[16];

    sys_init();
    adc_init();
    uart_init(115200);

    HAL_GPIO_LED0_out();
    HAL_GPIO_LED0_clr();
    HAL_GPIO_LED1_out();
    HAL_GPIO_LED1_clr();

    HAL_GPIO_LED0_write(0x1U);

    /* Enable temperature sensor and bandgap */
    SYSCTRL->VREF.reg |=   SYSCTRL_VREF_TSEN
                         | SYSCTRL_VREF_BGOUTEN;

    /* Enable ADC */
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;
    while (ADC->STATUS.bit.SYNCBUSY);

    uart_puts("\r\nHello! Starting temperature reading...\r\n\r\n");

    while (1) {

        count++;
        if (count == max_count) {
            count = 0;

            /* Start conversion and poll for result ready */
            ADC->SWTRIG.reg |= ADC_SWTRIG_START;
            while (!(ADC->INTFLAG.reg & ADC_INTFLAG_RESRDY));

            /* Get result, and convert to T */
            result = ADC->RESULT.reg;
            while (ADC->STATUS.bit.SYNCBUSY);
            adc_to_temp_str(tx_buf, result);
            uart_puts(tx_buf);
            HAL_GPIO_LED0_toggle();
            HAL_GPIO_LED1_toggle();
        }
    }

   return 0;
}
