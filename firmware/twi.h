/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (c) 2023, Michael Walle <michael@walle.cc>
 */

#ifndef __TWI_H
#define __TWI_H

void twi_init(void);
void twi_transfer(uint8_t addr, uint8_t *wr_data, uint8_t wr_len,
				  uint8_t *rd_data, uint8_t rd_len);

#endif /* __TWI_H */
