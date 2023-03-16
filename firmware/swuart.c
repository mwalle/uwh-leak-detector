/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "swuart.h"

static volatile uint16_t tx_char;
static volatile uint8_t tx_cnt;

#define BAUD 9600
#define TIMER0_BIT_CNT ((uint8_t)((F_CPU / BAUD) + 0.5))

void uart_init(void) {
	/* turn output on */
	PORTB |= _BV(PB3);
	DDRB |= _BV(PB3);

	OCR0A = TIMER0_BIT_CNT;
	TIMSK |= _BV(OCIE0A); /* enable compare interrupt */
	TCCR0A = _BV(WGM01); /* CTC mode */
}

void uart_putc(const char c)
{
	if (c == '\n')
		uart_putc('\r');

	/* loop until timer is disabled */
	while (TCCR0B);

	tx_char = 0x1801 | c << 2;
	tx_cnt = 13;

	/* now start the timer, everything else will be interrupt driven */
	TCCR0B = _BV(CS00); /* start Timer0, no prescaler */
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

ISR(TIM0_COMPA_vect)
{
	if (tx_char & 1)
		PORTB |= _BV(PB3);
	else
		PORTB &= ~_BV(PB3);
	tx_char >>= 1;

	if (--tx_cnt == 0)
		TCCR0B = 0; /* stop timer0 */
}
