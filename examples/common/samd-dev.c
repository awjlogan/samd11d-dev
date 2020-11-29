#include "samd11.h"
#include "samd-dev.h"

//-----------------------------------------------------------------------------
inline void sys_init(void)
{
    /* Disable prescalar -> 8 MHz core clock */
    SYSCTRL->OSC8M.bit.PRESC = 0;
}

//-----------------------------------------------------------------------------
void uart_putc(const char c)
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
