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

// #define USE_DIGITAL
#define USE_PWM

static bool isInit;

#ifdef USE_DIGITAL
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
#endif

static void irLedTimer(xTimerHandle timer)
{
#ifdef USE_DIGITAL
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
#endif

#ifdef USE_PWM
  irLedSetRatio(ratio);
#endif

}


#ifdef USE_PWM

// HW defines
#define SERVO_TIM_PERIF       RCC_APB1Periph_TIM5
#define SERVO_TIM             TIM5
#define SERVO_TIM_DBG         DBGMCU_TIM2_STOP
#define SERVO_TIM_SETCOMPARE  TIM_SetCompare2
#define SERVO_TIM_GETCAPTURE  TIM_GetCapture2

#define SERVO_GPIO_PERIF      RCC_AHB1Periph_GPIOA
#define SERVO_GPIO_PORT       GPIOA
#define SERVO_GPIO_PIN        GPIO_Pin_2
#define SERVO_GPIO_AF_PIN     GPIO_PinSource2
#define SERVO_GPIO_AF         GPIO_AF_TIM5

#define SERVO_PWM_BITS      (8)
#define SERVO_PWM_PERIOD    ((1<<SERVO_PWM_BITS) - 1)
#define SERVO_PWM_PRESCALE (0)

/* This should be calculated.. */
#define SERVO_BASE_FREQ (329500)

static uint8_t ratio = 237;

static void irLedSetRatio(uint8_t ratio)
{
  TIM_SetCompare3(SERVO_TIM, ratio);
}

static void irLedSetFreq(uint16_t freq)
{
  TIM_PrescalerConfig(SERVO_TIM, (SERVO_BASE_FREQ/freq), TIM_PSCReloadMode_Update);
}

#endif

static void irLedInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

#ifdef USE_DIGITAL

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
#endif

#ifdef USE_PWM
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(SERVO_GPIO_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(SERVO_TIM_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Pin = SERVO_GPIO_PIN;
  GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(SERVO_GPIO_PORT, SERVO_GPIO_AF_PIN, SERVO_GPIO_AF);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = SERVO_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  TIM_OC3Init(SERVO_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);

  // Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(SERVO_TIM, ENABLE);
  TIM_SetCompare3(SERVO_TIM, 0x00);

  // Enable the timer
  TIM_Cmd(SERVO_TIM, ENABLE);

  irLedSetFreq(1000);
  irLedSetRatio(ratio);

  timer = xTimerCreate( "irLedTimer", M2T(500),
                                     pdTRUE, NULL, irLedTimer );
  xTimerStart(timer, 100);
#endif


  isInit = true;
}

static const DeckDriver irled_deck = {
  .vid = 0xCA,
  .pid = 0x01,
  .name = "caIrLed",

#ifdef USE_PWM
  .usedPeriph = DECK_USING_TIMER5,
#endif

  .usedGpio = DECK_USING_TX2,

  .init = irLedInit,
};

DECK_DRIVER(irled_deck);

#ifdef USE_PWM
PARAM_GROUP_START(irled)
PARAM_ADD(PARAM_UINT32, baudrate, &baudrate)
PARAM_GROUP_STOP(irled)
#endif