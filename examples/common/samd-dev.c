#include "samd11.h"


//-----------------------------------------------------------------------------
static void uart_putc(char c)
{
    while (!(SERCOM0->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)) {
        SERCOM0->USART.DATA.reg = c;
    }
}

//-----------------------------------------------------------------------------
void uart_puts(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}
