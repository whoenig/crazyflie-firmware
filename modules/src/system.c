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
#include "mpu6050.h"
#include "led.h"

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
  ledInit();
  radiolinkInit();
  motorsInit();
  radiolinkSetEnable(true);
  RadioPacket packet;

  // wait for mpu6050 to startup (check if really needed)
  vTaskDelay(M2T(1000));
  i2cdevInit(I2C1);
  mpu6050Init(I2C1);
  mpu6050Reset();
  vTaskDelay(M2T(50));
  // Activate MPU6050
  mpu6050SetSleepEnabled(FALSE);
  // Enable temp sensor
  mpu6050SetTempSensorEnabled(TRUE);
  // Disable interrupts
  mpu6050SetIntEnabled(FALSE);
  // Connect the HMC5883L to the main I2C bus
  mpu6050SetI2CBypassEnabled(TRUE);
  // Set x-axis gyro as clock source
  mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
  // Set gyro full scale range
  mpu6050SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  // Set accelerometer full scale range
  mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);

//#ifdef IMU_MPU6050_DLPF_256HZ
//  // 256Hz digital low-pass filter only works with little vibrations
//  // Set output rate (15): 8000 / (1 + 15) = 500Hz
//  mpu6050SetRate(15);
//  // Set digital low-pass bandwidth
//  mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
//#else
  // To low DLPF bandwidth might cause instability and decrease agility
  // but it works well for handling vibrations and unbalanced propellers
  // Set output rate (1): 1000 / (1 + 1) = 500Hz
  mpu6050SetRate(1);
  // Set digital low-pass bandwidth
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_188);
//#endif

  struct sendData
  {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
  };

  ledSet(LED_GREEN, true);

  while(true)
  {
    struct sendData* d = (struct sendData*)packet.data;
    mpu6050GetMotion6(
      &d->accelX, &d->accelY, &d->accelZ,
      &d->gyroX, &d->gyroY, &d->gyroZ);
    packet.size = sizeof(struct sendData);
    radiolinkSendPacket(&packet);

    ledSet(LED_GREEN, true);
    radiolinkReceivePacket(&packet);
    ledSet(LED_GREEN, false);

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
