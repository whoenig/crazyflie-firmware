/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * nrf24l01.h: nRF24L01(-p) PRX mode low level driver
 */

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include <stdbool.h>

// Init and test of the connection to the chip
void nrfInit(void);
bool nrfTest(void);

// Interrupt routine
void nrfIsr();

typedef enum {
  NRF_DATARATE_250K = 0x26,
  NRF_DATARATE_1M   = 0x06,
  NRF_DATARATE_2M   = 0x0E
} nrfDatarate_t;

#define NRF_FEATURE_EN_DPL     (1<<2)
#define NRF_FEATURE_EN_ACK_PAY (1<<1)

//Interrupt access
void nrfSetInterruptCallback(void (*cb)(void));

// Low level functionality of the nrf chip
unsigned char nrfNop();
unsigned char nrfFlushRx();
unsigned char nrfFlushTx();
unsigned char nrfRxLength(unsigned int pipe);
unsigned char nrfActivate();
unsigned char nrfWriteAck(unsigned int pipe, char *buffer, int len);
unsigned char nrfReadRX(char *buffer, int len);
void nrfSetChannel(unsigned int channel);
void nrfSetDatarate(nrfDatarate_t datarate);
void nrfSetAddress(unsigned int pipe, char* address);
void nrfSetEnable(bool enable);
unsigned char nrfGetStatus();
bool nrfIsRxFull();
bool nrfIsRxEmpty();
bool nrfIsTxFull();
bool nrfIsTxEmpty();
void nrfSetConfig(uint8_t config);
void nrfEnableDynamicPayload(uint8_t pipeMask);
void nrfSetFeature(uint8_t features);

#endif
