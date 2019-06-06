/*
The MIT License (MIT)

Copyright (c) 2019 Wolfgang Hoenig

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication (section 7):

Daniel Morgan, Giri P Subramanian, Soon-Jo Chung, Fred Y Hadaegh
Swarm assignment and trajectory optimization using variable-swarm, distributed auction assignment and sequential convex programming 
IJRR 2016

Notes:
  * There is a typo in the paper, in eq 71: It should be -K (omega - omega_r) (i.e., no dot)

  * BIG TODO: swith to math3d after intial version works
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_sjc.h"

// #define GRAVITY_MAGNITUDE (9.81f)

static float g_vehicleMass = 0.032; // TODO: should be CF global for other modules

// Attitude P-gain
static float k1 = 5e-4;
static float k2 = 5e-4;
static float k3 = 5e-4;

// Attitude D-gain
static float l1 = 5;
static float l2 = 5;
static float l3 = 1;

// Inertia matrix, see
// System Identification of the Crazyflie 2.0 Nano Quadrocopter
// BA theses, Julian Foerster, ETHZ
// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
static float Ixx = 16.571710e-6; // kg m^2
static float Iyy = 16.655602e-6; // kg m^2
static float Izz = 29.261652e-6; // kg m^2

// logging variables
static struct vec moment;

void controllerSJCReset(void)
{
}

void controllerSJCInit(void)
{
  controllerSJCReset();
}

bool controllerSJCTest(void)
{
  return true;
}

void controllerSJC(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  float dt = (float)(1.0f/ATTITUDE_RATE);

  // Rate-controlled YAW is moving YAW angle setpoint
  float desiredYaw = 0; //deg
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = state->attitude.yaw + setpoint->attitudeRate.yaw * dt;
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);
    desiredYaw = degrees(rpy.z);
  }

  // Attitude controller

  float phi = radians(state->attitude.roll);
  // This is in the legacy coordinate system where pitch is inverted
  float theta = radians(-state->attitude.pitch);
  float psi = radians(state->attitude.yaw);

  float omega_x = radians(sensors->gyro.x);
  float omega_y = radians(sensors->gyro.y);
  float omega_z = radians(sensors->gyro.z);

  // q_dot = Z(q)*omega
  // TODO WH: check where Z(q) comes from
  float phi_dot = omega_x + sinf(phi)*tanf(theta)*omega_y + cosf(phi)*tanf(theta)*omega_z;
  float theta_dot = cosf(phi)*omega_y - sinf(phi)*omega_z;
  // TODO WH: add check for division by zero?
  float psi_dot = sinf(phi)*omega_y/cosf(theta) + cosf(phi)*omega_z/cosf(theta);

  // Desired angles in radians
  float eulerRollDesired = radians(setpoint->attitude.roll);
  // This is in the legacy coordinate system where pitch is inverted
  float eulerPitchDesired = -radians(setpoint->attitude.pitch);
  float eulerYawDesired = radians(desiredYaw);

  float RollDotDesired = radians(setpoint->attitudeRate.roll);
  float PitchDotDesired = radians(setpoint->attitudeRate.pitch);
  float YawDotDesired = radians(setpoint->attitudeRate.yaw);
  float RollDDotDesired = 0;
  float PitchDDotDesired = 0;
  float YawDDotDesired = 0;

  // qr dot
  float qr1_dot = RollDotDesired + l1*(eulerRollDesired - phi);
  float qr2_dot = PitchDotDesired + l2*(eulerPitchDesired - theta);
  float qr3_dot = YawDotDesired + l3*(eulerYawDesired - psi);

  // qr double dot
  float qr1_ddot = RollDDotDesired + l1*(RollDotDesired - phi_dot);
  float qr2_ddot = PitchDDotDesired + l2*(PitchDotDesired - theta_dot);
  float qr3_ddot = YawDDotDesired + l3*(YawDotDesired - psi_dot);

  // omega_r = Z(q)^-1 * q_r_dot
  float omega_r_1 = qr1_dot - sinf(theta)*qr3_dot;
  float omega_r_2 = cosf(phi)*qr2_dot + sinf(phi)*cosf(theta)*qr3_dot;
  float omega_r_3 = -sinf(phi)*qr2_dot + cosf(phi)*cosf(theta)*qr3_dot;
  
  // omega_r_dot = Z(q)^-1_dot*q_r_dot + Z(q)^-1*q_r_ddot
  float omega_r_1_dot = -cosf(theta)*theta_dot*qr3_dot + qr1_ddot - sinf(theta)*qr3_ddot;
  float omega_r_2_dot = -sinf(phi)*phi_dot*qr2_dot + (cosf(phi)*cosf(theta)*phi_dot - sinf(phi)*sinf(theta)*theta_dot)*qr3_dot + cosf(phi)*qr2_ddot + sinf(phi)*cosf(theta)*qr3_ddot;
  float omega_r_3_dot = -cosf(phi)*phi_dot*qr2_dot + (-sinf(phi)*cosf(theta)*phi_dot - cosf(phi)*sinf(theta)*theta_dot)*qr3_dot - sinf(phi)*qr2_ddot + cosf(phi)*cosf(theta)*qr3_ddot;

  // compute moments
  // u = J*omega_r_dot - S(J*omega)*omega_r - K(omega - omega_r)
  float actuatorRoll = Ixx*omega_r_1_dot - (-Izz*omega_z*omega_r_2 + Iyy*omega_y*omega_r_3) - k1*(omega_x - omega_r_1);
  float actuatorPitch = Iyy*omega_r_2_dot - (Izz*omega_z*omega_r_1 - Ixx*omega_x*omega_r_3) - k2*(omega_y - omega_r_2);
  float actuatorYaw = Izz*omega_r_3_dot - (-Iyy*omega_y*omega_r_1 + Ixx*omega_x*omega_r_2) - k3*(omega_z - omega_r_3);

  // On CF2, thrust is mapped 65536 <==> 60 grams
  float linAcc = 0.0; // vertical acceleration m/s^2
  moment = mkvec(actuatorRoll, actuatorPitch, actuatorYaw);
  if (setpoint->mode.z == modeDisable) {
    linAcc = setpoint->thrust / 65536.0f * 22.2f;
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSI = g_vehicleMass * linAcc;
  control->torque[0] = moment.x;
  control->torque[1] = moment.y;
  control->torque[2] = moment.z;
}

PARAM_GROUP_START(ctrlSJC)
PARAM_ADD(PARAM_FLOAT, k1, &k1)
PARAM_ADD(PARAM_FLOAT, k2, &k2)
PARAM_ADD(PARAM_FLOAT, k3, &k3)
PARAM_ADD(PARAM_FLOAT, l1, &l1)
PARAM_ADD(PARAM_FLOAT, l2, &l2)
PARAM_ADD(PARAM_FLOAT, l3, &l3)
PARAM_GROUP_STOP(ctrlSJC)

LOG_GROUP_START(ctrlSJC)
LOG_ADD(LOG_FLOAT, momentRoll, &moment.x)
LOG_ADD(LOG_FLOAT, momentPitch, &moment.y)
LOG_ADD(LOG_FLOAT, momentYaw, &moment.z)
LOG_GROUP_STOP(ctrlSJC)