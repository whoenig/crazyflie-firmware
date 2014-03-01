/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * radiolink.h: nRF24L01 task for communication
 */

#ifndef __RADIOLINK_H__
#define __RADIOLINK_H__

typedef struct
{
  uint8_t size;
  uint8_t data[32];
} RadioPacket;

void radiolinkInit();
bool radiolinkTest();

int radiolinkSetEnable(bool enable);
int radiolinkSendPacket(RadioPacket* pk);
int radiolinkReceivePacket(RadioPacket* pk);
int radiolinkReset(void);
bool radiolinkIsConnected(void);

#endif
