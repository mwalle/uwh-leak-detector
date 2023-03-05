/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "twi.h"

static void twi_start(void)
{
	/* start, scl will now held low automatically */
	PORTB |= _BV(PB0);
	PORTB |= _BV(PB2);
	PORTB &= ~_BV(PB0);
	PORTB &= ~_BV(PB2);  /* pull scl low */
	USISR |= _BV(USISIF); /* release automatic scl lock */
}

static void twi_stop(void)
{
	PORTB &= ~_BV(PB0);
	PORTB |= _BV(PB2);
	PORTB |= _BV(PB0);
}

static void twi_tx_byte(uint8_t b)
{
	uint8_t n = 7;

	USIDR = b;
	PORTB |= _BV(PB0); /* release sda, so it gets fed from the USIDR */

	/* first clock cycle (rising edge) */
	USICR |= _BV(USITC);
	USICR |= _BV(USITC);

	while (n--) {
		USICR |= _BV(USICLK);
		USICR |= _BV(USITC);
		USICR |= _BV(USITC);
	}

	/* make sure USIDR doesn't pull SDA low anymore */
	USIDR = 0xff;

	/* receive ack */
	DDRB &= ~_BV(PB0);
	USICR |= _BV(USITC);
	USICR |= _BV(USITC);
	DDRB |= _BV(PB0);
}

static void twi_ack(void)
{
	USIDR = 0x00;
	USICR |= _BV(USITC);
	USICR |= _BV(USITC);
}

static void twi_nack(void)
{
	USIDR = 0xff;
	USICR |= _BV(USITC);
	USICR |= _BV(USITC);
}

static void twi_rx_byte(uint8_t *b)
{
	uint8_t n = 8;

	/* disable output */
	DDRB &= ~_BV(PB0);

	while (n--) {
		USICR |= _BV(USITC);
		USICR |= _BV(USICLK);
		USICR |= _BV(USITC);
	}

	DDRB |= _BV(PB0);
	*b = USIDR;
}

void twi_transfer(uint8_t addr, uint8_t *wr_data, uint8_t wr_len,
				  uint8_t *rd_data, uint8_t rd_len)
{
	/* two wire mode, manual strobe clock */
	USICR = _BV(USIWM1);

	/* clear interrupt flags and clear counter */
	USISR = _BV(USISIF) | _BV(USIOIF);

	/* make sure USIDR doesn't pull SDA low */
	USIDR = 0xff;

	/* enable ports */
	DDRB |= _BV(PB2); /* SCL */
	DDRB |= _BV(PB0); /* SDA */

	/* start */
	twi_start();

	if (wr_len) {
		/* address + write */
		twi_tx_byte(addr << 1);

		/* data */
		while (wr_len--)
			twi_tx_byte(*(wr_data++));
	}

	if (rd_len) {
		/* (repeated) start */
		twi_start();

		/* address + read */
		twi_tx_byte(addr << 1 | 1);

		/* data */
		while (rd_len--) {
			twi_rx_byte(rd_data++);
			if (rd_len)
				twi_ack();
			else
				twi_nack();
		}
	}

	twi_stop();

	/* disable ports */
	DDRB &= ~_BV(PB2); /* SCL */
	DDRB &= ~_BV(PB0); /* SDA */

	USICR = 0; /* stop USI */
}
