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
  * This implementation is inspired by the implementation of the controller by 
    Giri P Subramanian for CF 1.0

  * Runtime attitude controller: 6us

  * BIG TODO2: switch to quaternion-based control
  * BIG TODO3: position controller
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_sjc.h"

// #define GRAVITY_MAGNITUDE (9.81f)

static float g_vehicleMass = 0.032; // TODO: should be CF global for other modules

// Attitude D-gain (diagonal matrix)
static struct vec K = {5e-4, 5e-4, 5e-4};

// Attitude P-gain (diagonal matrix)
static struct vec lambda = {5, 5, 1};

// Inertia matrix (diagonal matrix), see
// System Identification of the Crazyflie 2.0 Nano Quadrocopter
// BA theses, Julian Foerster, ETHZ
// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
static struct vec J = {16.571710e-6, 16.655602e-6, 29.261652e-6}; // kg m^2

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

  // q: vector of three-dimensional attitude representation
  //    here: Euler angles in rad
  struct vec q = mkvec(
    radians(state->attitude.roll),
    radians(-state->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
    radians(state->attitude.yaw));

  // omega: angular velocity in rad/s
  struct vec omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  // q_dot = Z(q)*omega
  // For euler angles Z(q) is defined in Giri P Subramanian MS thesis, equation 8
  //        [1  sin(r)tan(p)  cos(r)tan(p)]
  // Z(q) = [0  cos(r)        -sin(r)     ]
  //        [0  sin(r)sec(p)  cos(r)sec(p)]
  // TODO: add check for division by zero?
  struct vec q_dot = mkvec(
    omega.x + sinf(q.x)*tanf(q.y)*omega.y + cosf(q.x)*tanf(q.y)*omega.z,
              cosf(q.x)          *omega.y - sinf(q.x)          *omega.z,
              sinf(q.x)/cosf(q.y)*omega.y + cosf(q.x)/cosf(q.y)*omega.z);

  // qr: Desired/reference angles in rad
  struct vec qr = mkvec(
    radians(setpoint->attitude.roll),
    -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
    radians(desiredYaw));

  // qr_dot desired/reference angular velocity in rad/s
  struct vec qr_dot = mkvec(
    radians(setpoint->attitudeRate.roll),
    radians(setpoint->attitudeRate.pitch),
    radians(setpoint->attitudeRate.yaw));

  // qr_ddot desired/reference angular acceleration in rad/s^2
  struct vec qr_ddot = vzero();

  // omega_r: desired/reference angular velocity in rad/s
  // omega_r = Z(q)^-1 * (qr_dot + lambda (qr - q)) = Z(q)^-1 * qrp
  // For euler angles Z(q)^-1 is:
  //           [1  0       -sin(p)     ]
  // Z(q)^-1 = [0  cos(r)  cos(p)sin(r)]
  //           [0 -sin(r)  cos(p)cos(r)]
  struct vec qrp = vadd(qr_dot, veltmul(lambda, vsub(qr, q)));
  struct vec omega_r = mkvec(
    qrp.x +                 -sinf(q.y)           *qrp.z,
            cosf(q.x)*qrp.y + cosf(q.y)*sinf(q.x)*qrp.z,
          - sinf(q.x)*qrp.y + cosf(q.y)*cosf(q.x)*qrp.z);

  // omegar_dot =   Z(q)^-1_dot*qr_dot         + Z(q)^-1*qr_ddot
  //              + Z(q)^-1_dot*lambda(qr - q) + Z(q)^-1*lambda*(qr_dot - q_dot)
  //            =   Z(q)^-1_dot (qr_dot + lambda (qr - q))
  //              + Z(q)^-1 (qr_ddot + lambda (qr_dot - q_dot))
  //            =   Z(q)^-1_dot qrp + Z(q)^-1 qrp_dot
  // For euler angles Z(q)^-1_dot is:
  //               [0  0            -cos(p)p_dot                        ]
  // Z(q)^-1_dot = [0 -sin(r)r_dot  -sin(p)sin(r)p_dot+cos(p)cos(r)r_dot]
  //               [0 -cos(r)r_dot  -cos(r)sin(p)p_dot-cos(p)sin(r)r_dot]
  struct vec qrp_dot = vadd(qr_ddot, veltmul(lambda, vsub(qr_dot, q_dot)));
  struct vec omega_r_dot = mkvec(
                             -cosf(q.y)             *q_dot.y                             *qrp.z + qrp_dot.x                     -sinf(q.y)           *qrp_dot.z,
    -sinf(q.x)*q_dot.x*qrp.y + (-sinf(q.y)*sinf(q.x)*q_dot.y+cosf(q.y)*cosf(q.x)*q_dot.x)*qrp.z +           cosf(q.x)*qrp_dot.y + cosf(q.y)*sinf(q.x)*qrp_dot.z,
    -cosf(q.x)*q_dot.x*qrp.y + (-cosf(q.x)*sinf(q.y)*q_dot.y-cosf(q.y)*sinf(q.x)*q_dot.x)*qrp.z +          -sinf(q.x)*qrp_dot.y + cosf(q.y)*cosf(q.x)*qrp_dot.z);

  // compute moments (note there is a type on the paper in equation 71)
  // u = J omega_r_dot - (J omega) x omega_r - K(omega - omega_r)
  struct vec u = vsub2(veltmul(J, omega_r_dot), vcross(veltmul(J, omega), omega_r), veltmul(K, vsub(omega, omega_r)));

  // On CF2, thrust is mapped 65536 <==> 60 grams
  float linAcc = 0.0; // vertical acceleration m/s^2
  if (setpoint->mode.z == modeDisable) {
    linAcc = setpoint->thrust / 65536.0f * 22.2f;
  }

  control->controlMode = controlModeForceTorque;
  control->thrustSI = g_vehicleMass * linAcc;
  control->torque[0] = u.x;
  control->torque[1] = u.y;
  control->torque[2] = u.z;
}

PARAM_GROUP_START(ctrlSJC)
PARAM_ADD(PARAM_FLOAT, k1, &K.x)
PARAM_ADD(PARAM_FLOAT, k2, &K.y)
PARAM_ADD(PARAM_FLOAT, k3, &K.z)
PARAM_ADD(PARAM_FLOAT, l1, &lambda.x)
PARAM_ADD(PARAM_FLOAT, l2, &lambda.y)
PARAM_ADD(PARAM_FLOAT, l3, &lambda.z)
PARAM_GROUP_STOP(ctrlSJC)

LOG_GROUP_START(ctrlSJC)
LOG_ADD(LOG_FLOAT, momentRoll, &moment.x)
LOG_ADD(LOG_FLOAT, momentPitch, &moment.y)
LOG_ADD(LOG_FLOAT, momentYaw, &moment.z)
LOG_GROUP_STOP(ctrlSJC)