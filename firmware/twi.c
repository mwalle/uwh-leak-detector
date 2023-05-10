/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#include <stdbool.h>
#include <string.h>
#include <avr/io.h>

#include "twi.h"

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include "uart.h"

void twi_init(void)
{
	/* ~100kHz */
	TWI0_MBAUD = F_CPU / 2 / 100000;

	TWI0_MCTRLA = TWI_ENABLE_bm;
	TWI0_MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

void twi_transfer(uint8_t addr, uint8_t *wr_data, uint8_t wr_len,
				  uint8_t *rd_data, uint8_t rd_len)
{
	/* start, addr + write */
	TWI0_MADDR = addr << 1;
	loop_until_bit_is_set(TWI0_MSTATUS, TWI_WIF_bp);

	if (TWI0_MSTATUS & TWI_RXACK_bm)
		goto out;

	/* data */
	while (wr_len--) {
		TWI0_MDATA = *(wr_data++);
		loop_until_bit_is_set(TWI0_MSTATUS, TWI_WIF_bp);
	}

	if (rd_len) {
		/* (repeated) start + read */
		TWI0_MADDR = addr << 1 | 1;
		loop_until_bit_is_set(TWI0_MSTATUS, TWI_RIF_bp);
		if (TWI0_MSTATUS & TWI_RXACK_bm)
			goto out;

		/* data */
		while (rd_len--) {
			*(rd_data++) = TWI0_MDATA;

			if (rd_len) {
				TWI0_MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;
				loop_until_bit_is_set(TWI0_MSTATUS, TWI_RIF_bp);
			}
		}
	}

	/* stop */
out:
	TWI0_MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
	while ((TWI0_MSTATUS & TWI_BUSSTATE_gm) != TWI_BUSSTATE_IDLE_gc);
}
