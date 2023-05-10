/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __SWUART_H
#define __SWUART_H

void uart_init(void);
void uart_putc(const char c);
void uart_puts(const char *s);
void uart_puts_P(const char *s);

#endif /* __USI_H */
