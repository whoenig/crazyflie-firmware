/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "motors.h"

static bool motorSetEnable = false;
static uint8_t saturationStatus = 0;

enum saturationBits
{
  SaturationOffsetThrust = 1,
  SaturationRollPitch    = 2,
  SaturationYaw          = 4,
};

struct motorPower_s {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

float motorForce[4];

// static float g_thrustpart;
static float g_rollpart;
static float g_pitchpart;
static float g_yawpart;

static float thrust;
static struct vec torque;

void powerDistributionInit(void)
{
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

static void powerDistributionLegacy(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);
}

static void powerDistributionForceTorque(const control_t *control)
{
  // On CF2, thrust is mapped 65536 <==> 60 grams
  thrust = control->thrustSI;
  torque = mkvec(control->torque[0], control->torque[1], control->torque[2]);

  // torque.x = clamp(torque.x, -0.002, 0.002);
  // torque.y = clamp(torque.y, -0.002, 0.002);
  // torque.z = clamp(torque.z, -0.0005, 0.0005);

  // see https://github.com/jpreiss/libquadrotor/blob/master/src/quad_control.c
  const float thrust_to_torque = 0.006f;
  const float arm_length = 0.046f; // m
  const float thrustpart = 0.25f * control->thrustSI; // N (per rotor)
  const float yawpart = -0.25f * torque.z / thrust_to_torque;

  float const arm = 0.707106781f * arm_length;
  const float rollpart = 0.25f / arm * torque.x;
  const float pitchpart = 0.25f / arm * torque.y;

  // Thrust for each motor in N
  saturationStatus = 0;

  // Simple thrust mixing
#if 1
  motorForce[0] = thrustpart - rollpart - pitchpart + yawpart;
  motorForce[1] = thrustpart - rollpart + pitchpart - yawpart;
  motorForce[2] = thrustpart + rollpart + pitchpart + yawpart;
  motorForce[3] = thrustpart + rollpart - pitchpart - yawpart;

  // g_thrustpart = thrustpart;
  g_rollpart = rollpart;
  g_pitchpart = pitchpart;
  g_yawpart = yawpart;
#else
  // Thrust mixing similar to PX4 (see https://px4.github.io/Firmware-Doxygen/d7/d2a/mixer__multirotor_8cpp_source.html)
  // 1. Mix thrust, roll, pitch without yaw
  motorForce[0] = thrustpart - rollpart - pitchpart;
  motorForce[1] = thrustpart - rollpart + pitchpart;
  motorForce[2] = thrustpart + rollpart + pitchpart;
  motorForce[3] = thrustpart + rollpart - pitchpart;

  float minForce = motorForce[0];
  float maxForce = motorForce[0];
  for (int i = 1; i < 4; ++i) {
    minForce = fminf(minForce, motorForce[i]);
    maxForce = fmaxf(maxForce, motorForce[i]);
  }

  float deltaForce = maxForce - minForce;
  float thrustOffset = 0;
  float rollPitchScale = 1.0f;

  // 2. Check if we can shift thrust to avoid saturation
  if (deltaForce <= max_thrust) {
    if (minForce < 0) {
      thrustOffset = -minForce;
    } else if (maxForce > max_thrust) {
      thrustOffset = -(maxForce - max_thrust);
    }
  } else {
    // shifting thrust is not sufficient => scale roll and pitch as well
    rollPitchScale = max_thrust / deltaForce;
    thrustOffset = max_thrust - ((maxForce - thrustpart) * rollPitchScale + thrustpart);
  }

  // 3. Mix thrust (with offset), roll/pitch (with scale) to identify margin for yaw
  motorForce[0] = thrustpart+thrustOffset - rollpart*rollPitchScale - pitchpart*rollPitchScale;
  motorForce[1] = thrustpart+thrustOffset - rollpart*rollPitchScale + pitchpart*rollPitchScale;
  motorForce[2] = thrustpart+thrustOffset + rollpart*rollPitchScale + pitchpart*rollPitchScale;
  motorForce[3] = thrustpart+thrustOffset + rollpart*rollPitchScale - pitchpart*rollPitchScale;

  float maxYawPart = max_thrust - motorForce[0]; // positive yaw can be at most
  float minYawPart = 0 - motorForce[0]; // negative yaw can be at most
  maxYawPart = fmaxf(maxYawPart, motorForce[1]);
  maxYawPart = fmaxf(maxYawPart, max_thrust - motorForce[2]);
  maxYawPart = fmaxf(maxYawPart, motorForce[3]);
  minYawPart = fminf(minYawPart, motorForce[1] - max_thrust);
  minYawPart = fminf(minYawPart, 0 - motorForce[2]);
  minYawPart = fminf(minYawPart, motorForce[3] - max_thrust);

  float clamped_yawpart = clamp(yawpart, minYawPart, maxYawPart);
  if (thrustOffset != 0) {
    saturationStatus |= SaturationOffsetThrust;
  }
  if (rollPitchScale != 1.0f) {
    saturationStatus |= SaturationRollPitch;
  }
  if (yawpart != clamped_yawpart) {
    saturationStatus |= SaturationYaw;
  }

  // 4. Final thrust mixing (unlike PX4, we do not allow to reduce thrust to get yaw response)
  motorForce[0] = thrustpart+thrustOffset - rollpart*rollPitchScale - pitchpart*rollPitchScale + clamped_yawpart;
  motorForce[1] = thrustpart+thrustOffset - rollpart*rollPitchScale + pitchpart*rollPitchScale - clamped_yawpart;
  motorForce[2] = thrustpart+thrustOffset + rollpart*rollPitchScale + pitchpart*rollPitchScale + clamped_yawpart;
  motorForce[3] = thrustpart+thrustOffset + rollpart*rollPitchScale - pitchpart*rollPitchScale - clamped_yawpart;
#endif
  // for CF2, motorratio directly maps to thrust (not rpm etc.)
  // Thus, we only need to scale the values here

  const float maxThrustInGram = motorsGetMaxThrust(); // g

#if 0
  const float maxThrust = maxThrustInGram * 9.81f / 1000.0f; // N

  // yaw-torque saturation
  // a) mix without yaw
  motorForce[0] = thrustpart - rollpart - pitchpart;
  motorForce[1] = thrustpart - rollpart + pitchpart;
  motorForce[2] = thrustpart + rollpart + pitchpart;
  motorForce[3] = thrustpart + rollpart - pitchpart;

  // b) find maxYawPart
  float maxYawPart = maxThrust - motorForce[0]; // positive yaw can be at most
  float minYawPart = 0 - motorForce[0]; // negative yaw can be at most
  maxYawPart = fmaxf(maxYawPart, motorForce[1]);
  maxYawPart = fmaxf(maxYawPart, maxThrust - motorForce[2]);
  maxYawPart = fmaxf(maxYawPart, motorForce[3]);
  minYawPart = fminf(minYawPart, motorForce[1] - maxThrust);
  minYawPart = fminf(minYawPart, 0 - motorForce[2]);
  minYawPart = fminf(minYawPart, motorForce[3] - maxThrust);

  float clamped_yawpart = 0;
  if (minYawPart < maxYawPart) {
    clamped_yawpart = clamp(yawpart, minYawPart, maxYawPart);
  }

  // c) re-mix
  motorForce[0] = thrustpart - rollpart - pitchpart + clamped_yawpart;
  motorForce[1] = thrustpart - rollpart + pitchpart - clamped_yawpart;
  motorForce[2] = thrustpart + rollpart + pitchpart + clamped_yawpart;
  motorForce[3] = thrustpart + rollpart - pitchpart - clamped_yawpart;

  // collective-thrust saturation: skip for now
#endif
  for (int i = 0; i < 4; ++i) {
    float forceInGrams = clamp(motorForce[i] / 9.81f * 1000.0f, 0, maxThrustInGram);
    motorsSetThrust(i, forceInGrams);
  }
}

void powerDistribution(const control_t *control)
{
  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  } else {

    switch (control->controlMode)
    {
      case controlModeLegacy:
        powerDistributionLegacy(control);
        break;
      case controlModeForceTorque:
        powerDistributionForceTorque(control);
        break;
    }
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT16, m4, &motorPower.m4)
LOG_ADD(LOG_UINT16, m1, &motorPower.m1)
LOG_ADD(LOG_UINT16, m2, &motorPower.m2)
LOG_ADD(LOG_UINT16, m3, &motorPower.m3)
LOG_ADD(LOG_UINT8, saturation, &saturationStatus)


// LOG_ADD(LOG_FLOAT, thrust, &g_thrustpart)
LOG_ADD(LOG_FLOAT, roll, &g_rollpart)
LOG_ADD(LOG_FLOAT, pitch, &g_pitchpart)
LOG_ADD(LOG_FLOAT, yaw, &g_yawpart)

LOG_ADD(LOG_FLOAT, thrust, &thrust)
LOG_ADD(LOG_FLOAT, torquex, &torque.x)
LOG_ADD(LOG_FLOAT, torquey, &torque.y)
LOG_ADD(LOG_FLOAT, torquez, &torque.z)

LOG_ADD(LOG_FLOAT, f1, &motorForce[0])
LOG_ADD(LOG_FLOAT, f2, &motorForce[1])
LOG_ADD(LOG_FLOAT, f3, &motorForce[2])
LOG_ADD(LOG_FLOAT, f4, &motorForce[3])

LOG_GROUP_STOP(motor)
