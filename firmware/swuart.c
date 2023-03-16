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

ISR(TIM1_COMPA_vect)
{
	if (tx_char & 1)
		PORTB |= _BV(PB3);
	else
		PORTB &= ~_BV(PB3);
	tx_char >>= 1;

	if (--tx_cnt == 0)
		TCCR1 &= ~_BV(CS10); /* stop timer1 */
}

void uart_init(void) {
	/* turn output on */
	PORTB |= _BV(PB3);
	DDRB |= _BV(PB3);

	OCR1A = ((uint8_t)((F_CPU / BAUD) + 0.5)) - 1;
	OCR1C = ((uint8_t)((F_CPU / BAUD) + 0.5)) - 1;
	TIMSK |= _BV(OCIE1A); /* enable compare interrupt */
	TCCR1 = _BV(CTC1); /* CTC mode */
}

void uart_putc(const char c)
{
	if (c == '\n')
		uart_putc('\r');

	tx_char = 0x1801 | c << 2;
	tx_cnt = 13;

	TCNT1 = 0;

	/* now start the timer, everything else will be interrupt driven */
	TCCR1 |= _BV(CS10); /* start Timer1, no prescaler */

	/* loop until timer is disabled */
	loop_until_bit_is_clear(TCCR1, CS10);
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
