/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
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
#define PERIOD_FAST     100
#define PERIOD_SLOW     500

HAL_GPIO_PIN(LED0,     A, 14)
HAL_GPIO_PIN(LED1,     A, 15)
HAL_GPIO_PIN(BUTTON,   A, 16)
HAL_GPIO_PIN(UART_TX,  A, 8)
HAL_GPIO_PIN(UART_RX,  A, 9)


//-----------------------------------------------------------------------------
static void timer_set_period(uint16_t i)
{
    TC1->COUNT16.CC[0].reg = (F_CPU / 1000ul / 256) * i;
    TC1->COUNT16.COUNT.reg = 0;
}


//-----------------------------------------------------------------------------
void irq_handler_tc1(void)
{
    if (TC1->COUNT16.INTFLAG.reg & TC_INTFLAG_MC(1)) {
        HAL_GPIO_LED0_toggle();
        TC1->COUNT16.INTFLAG.reg = TC_INTFLAG_MC(1);
    }
}


//-----------------------------------------------------------------------------
static void timer_init(void)
{
    PM->APBCMASK.reg |= PM_APBCMASK_TC1;

    /* Setup clock system */
    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = TC1_GCLK_ID,  /* Generic clock selection ID */
        .bit.GEN = 0,           /* Generic clock generator */
        .bit.CLKEN = true,      /* Clock enable */
        .bit.WRTLOCK = false    /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

    /* Setup TC1 */
    TC_CTRLA_Type ctrla = {
        .bit.SWRST = false,                             /* Software reset */
        .bit.ENABLE = false,                            /* Enable */
        .bit.MODE = TC_CTRLA_MODE_COUNT16_Val,          /* TC mode */
        .bit.WAVEGEN = TC_CTRLA_WAVEGEN_MFRQ_Val,       /* Waveform generation operation */
        .bit.PRESCALER = TC_CTRLA_PRESCALER_DIV256_Val, /* Prescalar */
        .bit.RUNSTDBY = false,                          /* Run in standby */
        .bit.PRESCSYNC = TC_CTRLA_PRESCSYNC_RESYNC_Val  /* Prescalar/counter synchronisation */
    };
    TC1->COUNT16.CTRLA.reg = ctrla.reg;

    /* Set period, and start timer */
    TC1->COUNT16.COUNT.reg = 0;
    timer_set_period(PERIOD_SLOW);
    TC1->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;

    /* Enable interrupt */
    TC1->COUNT16.INTENSET.reg = TC_INTENSET_MC(1);
    NVIC_EnableIRQ(TC1_IRQn);
}


//-----------------------------------------------------------------------------
static void uart_init(uint32_t baud)
{
    uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

    HAL_GPIO_UART_TX_out();
    HAL_GPIO_UART_TX_pmuxen(PORT_PMUX_PMUXE_D_Val); // SERCOM-ALT
    HAL_GPIO_UART_RX_in();
    HAL_GPIO_UART_RX_pmuxen(PORT_PMUX_PMUXE_D_Val); // SERCOM-ALT

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    GCLK_CLKCTRL_Type clkctrl = {
        .bit.ID = SERCOM0_GCLK_ID_CORE, /* Generic clock selection ID */
        .bit.GEN = 0,                   /* Generic clock generator */
        .bit.CLKEN = true,              /* Clock enable */
        .bit.WRTLOCK = false            /* Write lock */
    };
    GCLK->CLKCTRL.reg = clkctrl.reg;

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

    SERCOM0->USART.BAUD.reg = (uint16_t)br+1;
    SERCOM0->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}


//-----------------------------------------------------------------------------
int main(void) {

    uint32_t cnt = 0;
    uint32_t total_chars = 0;
    bool fast = false;

    sys_init();
    timer_init();
    uart_init(115200);

    HAL_GPIO_LED0_out();
    HAL_GPIO_LED0_clr();
    HAL_GPIO_LED1_write(0x1U);

    /* 128bits of unique ID -> 32 hex characters + NULL */
    uint32_t unique_id[4];
    char unique_id_str[33];

    /* Get unique ID (Section 9.6)
     * Unfortunately, it's not contiguous so just enter address
     */
    unique_id[0] = *(volatile uint32_t *) 0x0080A00Cu;
    unique_id[1] = *(volatile uint32_t *) 0x0080A040u;
    unique_id[2] = *(volatile uint32_t *) 0x0080A044u;
    unique_id[3] = *(volatile uint32_t *) 0x0080A048u;

    /* Convert the unique ID into hex characters */
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t nibble;
            nibble = (unique_id[i] & (0xF << j * 4)) >> (j * 4);
            unique_id_str[7 - j + i * 8] = "0123456789abcdef"[nibble];
        }
    }
    unique_id_str[32] = (char) 0;

    uart_puts("\r\nHello! I am SAMD 0x");
    uart_puts(unique_id_str);
    uart_puts("\r\n");

    while (1) {
        cnt++;
        if (cnt == 5000) {
            cnt = 0;
            timer_set_period(fast ? PERIOD_SLOW : PERIOD_FAST);
            fast = !fast;
            __WFI();

            /* Print up to 64 . */
            if (total_chars++ < 64) {
                uart_putc('.');
            }
        }
    }

   return 0;
}
