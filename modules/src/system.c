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
#define DEBUG_MODULE "SYS"

#include <stdbool.h>
#include <string.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "led.h"
#include "imu.h"
 #include "math.h"

// #include "debug.h"
// #include "version.h"
// #include "config.h"
// #include "param.h"
// #include "log.h"
// #include "ledseq.h"
// #include "pm.h"

#include "config.h"
#include "system.h"
#include "uart_syslink.h"
#include "syslink.h"
// #include "configblock.h"
// #include "worker.h"
// #include "freeRTOSdebug.h"
// #include "uart_syslink.h"
// #include "comm.h"
// #include "stabilizer.h"
// #include "commander.h"
// #include "neopixelring.h"
// #include "console.h"
// #include "usb.h"
// #include "expbrd.h"
// #include "mem.h"

 #include "SEGGER_RTT.h"

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  xTaskCreate(systemTask, (const signed char * const)SYSTEM_TASK_NAME,
              SYSTEM_TASK_STACKSIZE, NULL,
              SYSTEM_TASK_PRI, NULL);

}

/* Private functions implementation */

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float deltat = 1.0/500.0;

static const float PI = 3.14159265358979323846f;
static const float beta = 1.5;

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;
  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // const float GyroMeasError = PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
  // const float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;
  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;
  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;
  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;
  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void systemTask(void *arg)
{
  uint32_t lastWakeTime, lastOutputTime;
  Axis3f gyro; // Gyro axis data in deg/s
  Axis3f acc;  // Accelerometer axis data in mG
  Axis3f mag;  // Magnetometer axis data in tesla
  float yaw, pitch, roll;
  SyslinkPacket packet;
  float magMaxX = -1e9;
  float magMaxY = -1e9;
  float magMaxZ = -1e9;
  float magMinX = 1e9;
  float magMinY = 1e9;
  float magMinZ = 1e9;

  //float magCalib[] = {-429,-237,-251,-63,-102,47}; //WH2
  float magCalib[] = {-72,118,-36,146,-35,122};//CF4

  float magOffsetX = (magCalib[0] + magCalib[1]) / 2.0f / 1000.0f;
  float magOffsetY = (magCalib[2] + magCalib[3]) / 2.0f / 1000.0f;
  float magOffsetZ = (magCalib[4] + magCalib[5]) / 2.0f / 1000.0f;
  float magDeltaX = (magCalib[1] - magCalib[0]);
  float magDeltaY = (magCalib[3] - magCalib[2]);
  float magDeltaZ = (magCalib[5] - magCalib[4]);
  float magMaxDelta = magDeltaX;
  if (magDeltaY > magMaxDelta) magMaxDelta = magDeltaY;
  if (magDeltaZ > magMaxDelta) magMaxDelta = magDeltaZ;
  float magScaleX = magMaxDelta / magDeltaX;
  float magScaleY = magMaxDelta / magDeltaY;
  float magScaleZ = magMaxDelta / magDeltaZ;

  ledInit();
  uartInit();
  syslinkInit();

  ledSet(CHG_LED, 1);

  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

  lastWakeTime = xTaskGetTickCount ();
  lastOutputTime = xTaskGetTickCount();

  SEGGER_RTT_printf(0, "Startup\n");

  imu6Init();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(500)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    // correct magnetometer
    if (! (mag.x == 0 && mag.y == 0 && mag.z == 0))
    {
      if (mag.x > magMaxX) magMaxX = mag.x;
      if (mag.y > magMaxY) magMaxY = mag.y;
      if (mag.z > magMaxZ) magMaxZ = mag.z;
      if (mag.x < magMinX) magMinX = mag.x;
      if (mag.y < magMinY) magMinY = mag.y;
      if (mag.z < magMinZ) magMinZ = mag.z;
    }

    mag.x = (mag.x - magOffsetX) * magScaleX;
    mag.y = (mag.y - magOffsetY) * magScaleY;
    mag.z = (mag.z - magOffsetZ) * magScaleZ;

    MadgwickQuaternionUpdate(acc.x, acc.y, acc.z, gyro.x*PI/180.0f, gyro.y*PI/180.0f, gyro.z*PI/180.0f, mag.y, mag.x, mag.z);


    yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw *= 180.0f / PI;
    //yaw -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll *= 180.0f / PI;

    if (xTaskGetTickCount() > lastOutputTime + M2T(100))
    {
      SEGGER_RTT_printf(0, "A: %d, %d, %d\n", (int)(roll*1.0), (int)(pitch*1.0), (int)(yaw*1.0));
      SEGGER_RTT_printf(0, "V: %d,%d,%d\n", (int)(mag.x*1000.0), (int)(mag.y*1000.0), (int)(mag.z*1000.0));
      SEGGER_RTT_printf(0, "MM: %d,%d,%d,%d,%d,%d\n",
        (int)(magMinX*1000.0), (int)(magMaxX*1000.0),
        (int)(magMinY*1000.0), (int)(magMaxY*1000.0),
        (int)(magMinZ*1000.0), (int)(magMaxZ*1000.0));

      packet.type = SYSLINK_SENSORS_POSE;
      packet.length = 3 * sizeof(float);
      memcpy(&packet.data[0], &roll, sizeof(float));
      memcpy(&packet.data[4], &pitch, sizeof(float));
      memcpy(&packet.data[8], &yaw, sizeof(float));
      syslinkSendPacket(&packet);

      lastOutputTime = xTaskGetTickCount();
    }
  }

  // while(1)
  // {
  //   vTaskDelay(portMAX_DELAY);
  // }
}

void vApplicationIdleHook( void )
{
  extern size_t debugPrintTCBInfo(void);
  static uint32_t timeToPrint = M2T(5000);

  if (xTaskGetTickCount() - timeToPrint > M2T(10000))
  {
    timeToPrint = xTaskGetTickCount();
    debugPrintTCBInfo();
  }
  // Enter sleep mode
//  { __asm volatile ("wfi"); }
}
