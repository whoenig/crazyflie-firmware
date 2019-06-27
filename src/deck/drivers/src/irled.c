/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 BitCraze AB
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
 * irled.c - Deck driver for ir led deck
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

// #include "uart2.h"
#include "configblock.h"
#include "deck.h"
#include "param.h"

// #include "config.h"
#include "nvic.h"
#include "uart2.h"
// #include "cfassert.h"
#include "config.h"
#include "nvicconf.h"

static bool isInit;

static xTimerHandle timer;
static uint8_t my_id;
static bool parity;

static enum
{
  Start,
  TransmitHigh1,
  TransmitBit1,
  TransmitHigh2,
  TransmitBit2,
  TransmitHigh3,
  TransmitBit3,
  TransmitHigh4,
  TransmitBit4,
  TransmitHigh5,
  TransmitBit5,
  TransmitHigh6,
  TransmitBit6,
  TransmitHigh7,
  TransmitBit7,
  TransmitHigh8,
  TransmitBit8,
  TransmitHigh9,
  TransmitParity,
  Stop,
  Wait,
} irLedState;

static void setLed(bool on) {
  if (on) {
    digitalWrite(DECK_GPIO_TX2, LOW);
  } else {
    digitalWrite(DECK_GPIO_TX2, HIGH);
  }
}

static void irLedTimer(xTimerHandle timer)
{
  switch(irLedState) {
  case TransmitHigh1:
  case TransmitHigh2:
  case TransmitHigh3:
  case TransmitHigh4:
  case TransmitHigh5:
  case TransmitHigh6:
  case TransmitHigh7:
  case TransmitHigh8:
  case TransmitHigh9:
  case Stop:
    setLed(true);
    break;
  case Start:
    setLed(false);
    break;
  case TransmitBit1:
    setLed((my_id >> 0) & 0x1);
    break;
  case TransmitBit2:
    setLed((my_id >> 1) & 0x1);
    break;
  case TransmitBit3:
    setLed((my_id >> 2) & 0x1);
    break;
  case TransmitBit4:
    setLed((my_id >> 3) & 0x1);
    break;
  case TransmitBit5:
    setLed((my_id >> 4) & 0x1);
    break;
  case TransmitBit6:
    setLed((my_id >> 5) & 0x1);
    break;
  case TransmitBit7:
    setLed((my_id >> 6) & 0x1);
    break;
  case TransmitBit8:
    setLed((my_id >> 7) & 0x1);
    break;
  case TransmitParity:
    setLed(parity);
    break;
  default:
    setLed(true);
    break;
  }

  if (irLedState == Wait + 200) {
    irLedState = Start;
  } else {
    irLedState += 1;
  }
}

static void irLedInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  pinMode(DECK_GPIO_TX2, OUTPUT);
  setLed(true);

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  parity = false;
  for (int i = 0; i < 8; ++i) {
    parity = parity ^ ((my_id >> i) & 0x1);
  }

  irLedState = Start;

  timer = xTimerCreate( "irLedTimer", M2T(10),
                                     pdTRUE, NULL, irLedTimer );
  xTimerStart(timer, 100);


  isInit = true;
}

static const DeckDriver irled_deck = {
  .vid = 0xCA,
  .pid = 0x01,
  .name = "caIrLed",

  .usedGpio = DECK_USING_TX2,

  .init = irLedInit,
};

DECK_DRIVER(irled_deck);

// PARAM_GROUP_START(irled)
// PARAM_ADD(PARAM_UINT32, baudrate, &baudrate)
// PARAM_GROUP_STOP(irled)
