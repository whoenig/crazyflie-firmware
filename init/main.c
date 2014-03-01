/**
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
 * main.c - Containing the main function.
 */

/* Project includes */
#include "config.h"
#include "nvic.h"
#include "led.h"
#include "usec_time.h"

/* ST includes */
#include "stm32f10x.h"

/* Private functions */
static void prvClockInit(void);
static void radiolinkInitNRF24L01P(void);
static void delay(uint32_t us);

int main()
{
  //Low level init: Clock and Interrupt controller
  prvClockInit();
  nvicInit();
  ledInit();
  initUsecTimer();

  nrfInit();
  radiolinkInitNRF24L01P();
  nrfSetEnable(true);

  uint8_t dataLen;
  uint8_t data[32];

  while(true)
  {
    ledSet(LED_GREEN, 1);

    while(nrfIsRxFull());
    ledSet(LED_RED, 1);
    dataLen = nrfRxLength(0);
    if (dataLen>32)          //If a packet has a wrong size it is dropped
      nrfFlushRx();
    else                     //Else, it is processed
    {
      //Fetch the data
      nrfReadRX((char *)data, dataLen);
      while(nrfIsTxFull());
      nrfWriteAck(0, (char*)data, dataLen);
    }
    ledSet(LED_RED, 0);
  }

  return 0;
}

static void delay(uint32_t us)
{
  uint64_t endTime = usecTimestamp() + us;
  while (usecTimestamp() < endTime);
}

static void radiolinkInitNRF24L01P(void)
{
  int i;
  char radioAddress[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

  //Set the radio channel
  nrfSetChannel(80);
  //Set the radio data rate
  nrfSetDatarate(NRF_DATARATE_2M);
  //Set radio address
  nrfSetAddress(0, radioAddress);

  //Power the radio, Enable the DS interruption, set the radio in PRX mode
  nrfSetConfig(0x3F);

  // Wait for the chip to be ready
  delay(2000);

  // Enable the dynamic payload size and the ack payload for the pipe 0
  nrfSetFeature(NRF_FEATURE_EN_DPL | NRF_FEATURE_EN_ACK_PAY);
  nrfEnableDynamicPayload(0x01);

  //Flush RX
  for(i=0;i<3;i++)
    nrfFlushRx();
  //Flush TX
  for(i=0;i<3;i++)
    nrfFlushTx();
}

//Clock configuration
static void prvClockInit(void)
{
  ErrorStatus HSEStartUpStatus;

  RCC_DeInit();
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);
  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/6 = 72 / 6 = 12 MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PLLCLK = 16MHz/2 * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08);
  } else {
    //Cannot start xtal oscillator!
    while(1);
  }
}
