/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "uart.h"

#define BAUD 9600

void uart_init(void)
{
	/* turn output on */
	PORTA_OUTSET = PIN6_bm;
	PORTA_DIRSET = PIN6_bm;

	USART0_BAUD = 64UL * F_CPU / 16 / BAUD;
	USART0_CTRLB |= USART_TXEN_bm;
}

void uart_putc(const char c)
{
	if (c == '\n')
		uart_putc('\r');

	USART0_STATUS = USART_TXCIF_bm;
	USART0_TXDATAL = c;

	/* loop until byte is sent */
	loop_until_bit_is_set(USART0_STATUS, USART_TXCIF_bp);
}

void uart_puts(const char *s)
{
	while (*s)
		uart_putc(*(s++));
}

void uart_puts_P(const char *s)
{
	char c;
	while ((c = pgm_read_byte(s++)))
		uart_putc(c);
}
