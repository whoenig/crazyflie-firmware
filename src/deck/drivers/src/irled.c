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

#if 0
static uint32_t baudrate = 25; // this leads to 1ms long pulses (checked with oscilloscope)
static void initUart(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UART2_GPIO_PERIF, ENABLE);
  ENABLE_UART2_RCC(UART2_PERIF, ENABLE);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UART2_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

  //Map uart to alternate functions
  GPIO_PinAFConfig(UART2_GPIO_PORT, UART2_GPIO_AF_TX_PIN, UART2_GPIO_AF_TX);

  USART_InitStructure.USART_BaudRate            = baudrate;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_Even;//USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART2_TYPE, &USART_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MID_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  //Enable UART
  USART_Cmd(UART2_TYPE, ENABLE);
}

void sendData(int size, uint8_t* data)
{
  for(int i = 0; i < size; i++)
  {
    while (!(UART2_TYPE->SR & USART_FLAG_TXE));
    UART2_TYPE->DR = (data[i] & 0x00FF);
  }
}

static void irLedTask(void *param)
{
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id = address & 0xFF;

  // UART sends LSB first
  // start bit is HIGH -> LOW
  // 0x55 = 01010101b
  // encode the data such that every other bit is guaranteed to be a 1
  uint16_t to_send = 0x5555 | 
                     (((my_id >> 0) & 0x1) << 1) |
                     (((my_id >> 1) & 0x1) << 3) |
                     (((my_id >> 2) & 0x1) << 5) |
                     (((my_id >> 3) & 0x1) << 7) |
                     (((my_id >> 4) & 0x1) << 9) |
                     (((my_id >> 5) & 0x1) << 11) |
                     (((my_id >> 6) & 0x1) << 13) |
                     (((my_id >> 7) & 0x1) << 15);

  uint32_t lastWakeTime = xTaskGetTickCount ();
  while(1) {
    vTaskDelayUntil(&lastWakeTime, T2M(2 * 1000)); // wait 2 secs

    initUart(baudrate);
    sendData(sizeof(to_send), (uint8_t*)&to_send);
  }
}
#endif

static void setLed(bool on) {
  if (on) {
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
  } else {
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
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

#if 0
  // 50 -> 1.25ms pulse

  // uart2Init(45); // Need at least 100 Hz sampling rate
  // initUart(50);

  xTaskCreate(irLedTask, "IRLEDTASK",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/0, NULL);

#else
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

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


  isInit = true;
}

static const DeckDriver irled_deck = {
  .vid = 0xCA,
  .pid = 0x01,
  .name = "caIrLed",

  .usedPeriph = DECK_USING_UART2,
  .usedGpio = DECK_USING_TX2,

  .init = irLedInit,
};

DECK_DRIVER(irled_deck);

// PARAM_GROUP_START(irled)
// PARAM_ADD(PARAM_UINT32, baudrate, &baudrate)
// PARAM_GROUP_STOP(irled)
