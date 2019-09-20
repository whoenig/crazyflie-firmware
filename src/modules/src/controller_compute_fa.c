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

#include <math.h>
#include <string.h>

#include "param.h"
#include "log.h"
#include "math3d.h"


#include "rho_2.h"
#include "phi_2.h"

#include "usec_time.h"
#include "crtp_localization_service.h"
#include "configblock.h"

#include "FreeRTOS.h"
#include "task.h"

static struct vec Fa;
static uint32_t ticks;
static uint8_t enableNN = 0;

static float scaleHack = 1.0f;

static float phiInput[6];
static float phiSummedOutput[40];
static uint8_t my_id;

static struct vec lastPos;
static struct vec lastVel;

#ifdef REALTIME_VERSION
extern float motorForce[4];
#endif

// 1 neighbor: 1000us
// 2 neighbors: 1600us
// 3 neighbors: 2100 us

// private functions
static void controllerComputeFaTask(void * prm);

void controllerComputeFaInit(void)
{
  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;

  lastPos = vzero();
  lastVel = vzero();

  #ifndef REALTIME_VERSION
  //Start the Fa task
  xTaskCreate(controllerComputeFaTask, COMPUTE_FA_TASK_NAME,
              COMPUTE_FA_TASK_STACKSIZE, NULL, COMPUTE_FA_TASK_PRI, NULL);
  #endif
}

void controllerComputeFa(const state_t *state, struct vec* F_d)
{
  if (enableNN > 0) {
    lastPos = mkvec(state->position.x, state->position.y, state->position.z);
    lastVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

#ifdef REALTIME_VERSION
    // realtime version
    const float mass = 0.034;
    struct vec a = vscl(9.81f, mkvec(state->acc.x, state->acc.y, state->acc.z + 1.0f));
    struct vec fu = mkvec(0, 0, motorForce[0] + motorForce[1] + motorForce[2] + motorForce[3]);

    struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    struct mat33 R = quat2rotmat(q);

    Fa = vsub(vscl(mass, a), mvmul(R, fu));

    // convert to g
    Fa = vscl(1000.0f / 9.81f, Fa);
#endif

    if (enableNN > 1) {

      // Apply Fa (convert Fa to N first)
      // TODO: CRAZY HACK
      *F_d = vsub(*F_d, vscl(9.81f / 1000.0f * scaleHack, Fa));
    }

  }
}

static void recomputeFa(void)
{
  uint64_t startTime = usecTimestamp();

  if (enableNN > 0) {

    // For each agent in the neighborhood, evaluate the rho network and sum the result
    // over all evaluations

    // zero entries
    memset(phiSummedOutput, 0, sizeof(phiSummedOutput));

    // evaluate rho for each nearby neighbor
    uint8_t numNeighbors = 0;
    // struct allCfState* myState = locSrvGetState(my_id);
    for (int id = MIN_CF_ID; id < MAX_CF_ID; ++id) {

      struct allCfState* otherState = locSrvGetState(id);

      // ignore myself and agents that have not received an update for 500ms
      if (id != my_id
          && xTaskGetTickCount() - otherState->timestamp < 500) {

        struct vec dpos = vsub(otherState->pos, lastPos);
        // if (fabsf(dpos.x) <= 0.5f && fabsf(dpos.y) <= 0.5f && dpos.z >= -1.0f && dpos.z <= 1.0f) {
        if (true) {
          struct vec dvel = vsub(otherState->vel, lastVel);
   
          // evaluate NN
          phiInput[0] = dpos.x;
          phiInput[1] = dpos.y;
          phiInput[2] = dpos.z;
          phiInput[3] = dvel.x;
          phiInput[4] = dvel.y;
          phiInput[5] = dvel.z;
          const float* phiOutput = nn_phi_2(phiInput);

          // sum result
          for (int i = 0; i < 40; ++i) {
            phiSummedOutput[i] += phiOutput[i];
          }
          ++numNeighbors;
        }
      }
    }

    // evaluate rho network
    if (numNeighbors > 0) {
      const float* rhoOutput = nn_rho_2(phiSummedOutput);
      Fa = mkvec(0, 0, rhoOutput[0]);
    } else {
      Fa = vzero();
    }
  } else {
    Fa = vzero();
  }

  ticks = usecTimestamp() - startTime;
}

void controllerComputeFaTask(void * prm)
{
  uint32_t lastWakeTime = xTaskGetTickCount();
  // execute at 100 Hz (which is roughly our position update)
  while(1) {
    recomputeFa();
    vTaskDelayUntil(&lastWakeTime, F2T(100));
  }
}

PARAM_GROUP_START(ctrlFa)
PARAM_ADD(PARAM_UINT8, enableNN, &enableNN)
// PARAM_ADD(PARAM_FLOAT, scaleHack, &scaleHack)

PARAM_GROUP_STOP(ctrlFa)

LOG_GROUP_START(ctrlFa)

LOG_ADD(LOG_FLOAT, Fax, &Fa.x)
LOG_ADD(LOG_FLOAT, Fay, &Fa.y)
LOG_ADD(LOG_FLOAT, Faz, &Fa.z)

LOG_ADD(LOG_UINT32, ticks, &ticks)

LOG_GROUP_STOP(ctrlFa)
