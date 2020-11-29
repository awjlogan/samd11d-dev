/* Copyright (c) 2020, Angus Logan <awjlogan@gmail.com> */

/* Collection of common routines for SAMD1x development board */

#ifndef _SAMD_DEV_H_
#define _SAMD_DEV_H_

void sys_init(void);
void uart_puts(const char *s);
void uart_putc(const char c);

#endif
