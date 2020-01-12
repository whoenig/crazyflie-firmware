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
This controller is based on the following publication:

Taeyoung Lee, Melvin Leok, and N. Harris McClamroch
Geometric Tracking Control of a Quadrotor UAV on SE(3)
CDC 2010

* Difference to Mellinger:
  * Different angular velocity error
  * Higher-order terms in attitude controller

TODO:
  * Switch position controller
  * consider Omega_d dot (currently assumes this is zero)

*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_lee.h"

#include "usec_time.h"

#include "FreeRTOS.h"
#include "task.h"

#define GRAVITY_MAGNITUDE (9.81f)

static float g_vehicleMass = 0.033; // TODO: should be CF global for other modules

// Inertia matrix (diagonal matrix), see
// System Identification of the Crazyflie 2.0 Nano Quadrocopter
// BA theses, Julian Foerster, ETHZ
// https://polybox.ethz.ch/index.php/s/20dde63ee00ffe7085964393a55a91c7
static struct vec J = {16.571710e-6, 16.655602e-6, 29.261652e-6}; // kg m^2

// Position PID
static struct vec Kpos_P = {6, 6, 6}; // Kp in paper
static float Kpos_P_limit = 100;
static struct vec Kpos_D = {4, 4, 4}; // Kv in paper
static float Kpos_D_limit = 100;
static struct vec Kpos_I = {0, 0, 0}; // not in paper
static float Kpos_I_limit = 2;
static struct vec i_error_pos;

// Attitude PID
static struct vec KR = {10, 10, 10};
static struct vec Komega = {0.0005, 0.0005, 0.001};

// Logging variables
static struct vec rpy;
static struct vec rpy_des;
static struct vec qr;
static struct vec omega;
static struct vec omega_r;
static struct vec u;

static uint32_t ticks;

static inline struct vec vclampscl(struct vec value, float min, float max) {
  return mkvec(
    clamp(value.x, min, max),
    clamp(value.y, min, max),
    clamp(value.z, min, max));
}

void controllerLeeReset(void)
{
  i_error_pos = vzero();
}

void controllerLeeInit(void)
{
  controllerLeeReset();
}

bool controllerLeeTest(void)
{
  return true;
}

void controllerLee(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{


  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  uint64_t startTime = usecTimestamp();

  float dt = (float)(1.0f/ATTITUDE_RATE);

  // Address inconsistency in firmware where we need to compute our own desired yaw angle
  // Rate-controlled YAW is moving YAW angle setpoint
  float desiredYaw = 0; //rad
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = radians(state->attitude.yaw + setpoint->attitudeRate.yaw * dt);
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = radians(setpoint->attitude.yaw);
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    rpy_des = quat2rpy(setpoint_quat);
    desiredYaw = rpy_des.z;
  }

  // qr: Desired/reference angles in rad
  // struct vec qr;

  // Position controller
  if (   setpoint->mode.x == modeAbs
      || setpoint->mode.y == modeAbs
      || setpoint->mode.z == modeAbs) {

    struct vec pos_d = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec vel_d = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
    struct vec acc_d = mkvec(setpoint->acceleration.x, setpoint->acceleration.y, setpoint->acceleration.z + GRAVITY_MAGNITUDE);
    struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
    struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

    // errors
    struct vec pos_e = vclampscl(vsub(pos_d, statePos), -Kpos_P_limit, Kpos_P_limit);
    struct vec vel_e = vclampscl(vsub(vel_d, stateVel), -Kpos_D_limit, Kpos_D_limit);
    i_error_pos = vclampscl(vadd(i_error_pos, vscl(dt, pos_e)), -Kpos_I_limit, Kpos_I_limit);

    struct vec F_d = vscl(g_vehicleMass, vadd4(
      acc_d,
      veltmul(Kpos_D, vel_e),
      veltmul(Kpos_P, pos_e),
      veltmul(Kpos_I, i_error_pos)));

    control->thrustSI = vmag(F_d);
    // Reset the accumulated error while on the ground
    if (control->thrustSI < 0.01f) {
      controllerLeeReset();
    }

    // Use current yaw instead of desired yaw for roll/pitch
    float yaw = radians(state->attitude.yaw);
    qr = mkvec(
      asinf((F_d.x * sinf(yaw) - F_d.y * cosf(yaw)) / control->thrustSI),
      atanf((F_d.x * cosf(yaw) + F_d.y * sinf(yaw)) / F_d.z),
      desiredYaw);
  } else {
    if (setpoint->mode.z == modeDisable) {
      if (setpoint->thrust < 1000) {
          control->controlMode = controlModeForceTorque;
          control->thrustSI = 0;
          control->torque[0] = 0;
          control->torque[1] = 0;
          control->torque[2] = 0;
          controllerLeeReset();
          return;
      }
    }
    // On CF2, thrust is mapped 65536 <==> 4 * 12 grams
    const float max_thrust = 70.0f / 1000.0f * 9.81f; // N
    control->thrustSI = setpoint->thrust / UINT16_MAX * max_thrust;

    qr = mkvec(
      radians(setpoint->attitude.roll),
      -radians(setpoint->attitude.pitch), // This is in the legacy coordinate system where pitch is inverted
      desiredYaw);
  }

  // Attitude controller

  // current rotation [R]
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);

  rpy = quat2rpy(q);

  // desired rotation [Rdes]
  struct quat q_des = rpy2quat(qr);
  struct mat33 R_des = quat2rotmat(q_des);

  rpy_des = quat2rpy(q_des);

  // rotation error
  struct mat33 eRM = msub(mmul(mtranspose(R_des), R), mmul(mtranspose(R), R_des));

  struct vec eR = vscl(0.5f, mkvec(eRM.m[2][1], eRM.m[0][2], eRM.m[1][0]));

  // angular velocity
  omega = mkvec(
    radians(sensors->gyro.x),
    radians(sensors->gyro.y),
    radians(sensors->gyro.z));

  struct vec omega_des = mkvec(
    radians(setpoint->attitudeRate.roll),
    radians(setpoint->attitudeRate.pitch),
    radians(setpoint->attitudeRate.yaw));

  omega_r = mvmul(mmul(mtranspose(R), R_des), omega_des);

  struct vec omega_error = vsub(omega, omega_r);

  // compute moments
  // M = -kR eR - kw ew + w x Jw - J(w x wr)
  u = vadd4(
    vneg(veltmul(KR, eR)),
    vneg(veltmul(Komega, omega_error)),
    vcross(omega, veltmul(J, omega)),
    vneg( veltmul(J, vcross(omega, omega_r)))
    );

  // if (enableNN > 1) {
  //   u = vsub(u, tau_a);
  // }

  control->controlMode = controlModeForceTorque;
  control->torque[0] = u.x;
  control->torque[1] = u.y;
  control->torque[2] = u.z;

  ticks = usecTimestamp() - startTime;
}

PARAM_GROUP_START(ctrlLee)
// Attitude P
PARAM_ADD(PARAM_FLOAT, KR_x, &KR.x)
PARAM_ADD(PARAM_FLOAT, KR_y, &KR.y)
PARAM_ADD(PARAM_FLOAT, KR_z, &KR.z)
// Attitude D
PARAM_ADD(PARAM_FLOAT, Kw_x, &Komega.x)
PARAM_ADD(PARAM_FLOAT, Kw_y, &Komega.y)
PARAM_ADD(PARAM_FLOAT, Kw_z, &Komega.z)

// J
PARAM_ADD(PARAM_FLOAT, J_x, &J.x)
PARAM_ADD(PARAM_FLOAT, J_y, &J.y)
PARAM_ADD(PARAM_FLOAT, J_z, &J.z)

// Position P
PARAM_ADD(PARAM_FLOAT, Kpos_Px, &Kpos_P.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Py, &Kpos_P.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Pz, &Kpos_P.z)
PARAM_ADD(PARAM_FLOAT, Kpos_P_limit, &Kpos_P_limit)
// Position D
PARAM_ADD(PARAM_FLOAT, Kpos_Dx, &Kpos_D.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Dy, &Kpos_D.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Dz, &Kpos_D.z)
PARAM_ADD(PARAM_FLOAT, Kpos_D_limit, &Kpos_D_limit)
// Position I
PARAM_ADD(PARAM_FLOAT, Kpos_Ix, &Kpos_I.x)
PARAM_ADD(PARAM_FLOAT, Kpos_Iy, &Kpos_I.y)
PARAM_ADD(PARAM_FLOAT, Kpos_Iz, &Kpos_I.z)
PARAM_ADD(PARAM_FLOAT, Kpos_I_limit, &Kpos_I_limit)
PARAM_GROUP_STOP(ctrlLee)


LOG_GROUP_START(ctrlLee)
LOG_ADD(LOG_FLOAT, torquex, &u.x)
LOG_ADD(LOG_FLOAT, torquey, &u.y)
LOG_ADD(LOG_FLOAT, torquez, &u.z)

// current angles
LOG_ADD(LOG_FLOAT, rpyx, &rpy.x)
LOG_ADD(LOG_FLOAT, rpyy, &rpy.y)
LOG_ADD(LOG_FLOAT, rpyz, &rpy.z)

// desired angles
LOG_ADD(LOG_FLOAT, rpydx, &rpy_des.x)
LOG_ADD(LOG_FLOAT, rpydy, &rpy_des.y)
LOG_ADD(LOG_FLOAT, rpydz, &rpy_des.z)

// errors
LOG_ADD(LOG_FLOAT, i_error_posx, &i_error_pos.x)
LOG_ADD(LOG_FLOAT, i_error_posy, &i_error_pos.y)
LOG_ADD(LOG_FLOAT, i_error_posz, &i_error_pos.z)

// omega
LOG_ADD(LOG_FLOAT, omegax, &omega.x)
LOG_ADD(LOG_FLOAT, omegay, &omega.y)
LOG_ADD(LOG_FLOAT, omegaz, &omega.z)

// omega_r
LOG_ADD(LOG_FLOAT, omegarx, &omega_r.x)
LOG_ADD(LOG_FLOAT, omegary, &omega_r.y)
LOG_ADD(LOG_FLOAT, omegarz, &omega_r.z)

LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlLee)
