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
 * system.c - Top level module implementation
 */
#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "system.h"
#include "radiolink.h"
#include "motors.h"

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  xTaskCreate(systemTask, (const signed char * const)"SYSTEM",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
}

bool systemTest()
{
  return true;
}

/* Private functions implementation */
static void systemTask(void *arg)
{
  radiolinkInit();
  motorsInit();
  radiolinkSetEnable(true);
  RadioPacket packet;

  while(true)
  {
    radiolinkReceivePacket(&packet);
    radiolinkSendPacket(&packet);
    if (packet.size == 4 * sizeof(uint16_t))
    {
      struct data
      {
        uint16_t m1;
        uint16_t m2;
        uint16_t m3;
        uint16_t m4;
      };
      struct data* d = (struct data*)packet.data;
      motorsSetRatio(MOTOR_M1, d->m1);
      motorsSetRatio(MOTOR_M2, d->m2);
      motorsSetRatio(MOTOR_M3, d->m3);
      motorsSetRatio(MOTOR_M4, d->m4);
    }
  }

  //Should never reach this point!
  while(1)
    vTaskDelay(portMAX_DELAY);
}
