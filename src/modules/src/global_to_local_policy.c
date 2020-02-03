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

#include "usec_time.h"
#include "crtp_localization_service.h"
#include "configblock.h"

#include "FreeRTOS.h"
#include "task.h"

#include "nn.h"

static uint32_t ticks;
static uint8_t enableNN = 0;
static float scale = 1.0;
static uint8_t my_id;

static struct vec last_goal;
static struct vec vel_desired;

struct obstacle {
  float x;
  float y;
  uint8_t enabled;
};
#define MAX_OBSTACLES (3)
static struct obstacle obstacles[MAX_OBSTACLES];

// Timing
// 1 neighbor: 800us

// private functions
static void globalToLocalPolicyTask(void * prm);

void globalToLocalPolicyInit(void)
{
  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;

  for (int i = 0; i < MAX_OBSTACLES; ++i) {
    obstacles[i].enabled = 0;
  }

  //Start the Fa task
  xTaskCreate(globalToLocalPolicyTask, G2L_POLICY_TASK_NAME,
              G2L_POLICY_TASK_STACKSIZE, NULL, G2L_POLICY_TASK_PRI, NULL);
}

void globalToLocalPolicyGet(const struct vec* goal, struct vec* vel)
{
  if (enableNN > 0) {
    last_goal = *goal;
    *vel = vel_desired;
  }
}

static void recompute(void)
{
  uint64_t startTime = usecTimestamp();

  if (enableNN > 0) {

    // get my current position
    struct vec pos = locSrvGetStateByCfId(my_id)->pos;

    // start NN evaluation
    nn_reset();

    // add all neighbors
    for (uint8_t idx = 0; ; ++idx) {

      struct allCfState* otherState = locSrvGetStateByIdx(idx);
      if (!otherState) {
        break;
      }

      // ignore myself and agents that have not received an update for 500ms
      if (   otherState->id != my_id
          && otherState->id != 0
          && xTaskGetTickCount() - otherState->timestamp < 500) {

        struct vec dpos = vsub(otherState->pos, pos);

        float input[2] = {dpos.x, dpos.y};
        nn_add_neighbor(input);
      }
    }

    // add all obstacles
    for (int i = 0; i < MAX_OBSTACLES; ++i) {
      if (obstacles[i].enabled) {
        struct vec dpos = vsub(mkvec(obstacles[i].x,obstacles[i].y,0), pos);
        float input[2] = {dpos.x, dpos.y};
        nn_add_obstacle(input);
      }
    }

    struct vec dpos = vsub(last_goal, pos);
    float input[2] = {dpos.x, dpos.y};
    const float* vel = nn_eval(input);

    vel_desired.x = scale * vel[0];
    vel_desired.y = scale * vel[1];
    vel_desired.z = 0;
  } else {
    vel_desired = vzero();
  }

  ticks = usecTimestamp() - startTime;
}

void globalToLocalPolicyTask(void * prm)
{
  uint32_t lastWakeTime = xTaskGetTickCount();
  // execute at 20 Hz to match simulation
  while(1) {
    recompute();
    vTaskDelayUntil(&lastWakeTime, F2T(20));
  }
}

PARAM_GROUP_START(g2lp)
PARAM_ADD(PARAM_UINT8, enableNN, &enableNN)
PARAM_ADD(PARAM_FLOAT, scale, &scale)

PARAM_ADD(PARAM_FLOAT, obs0x, &obstacles[0].x)
PARAM_ADD(PARAM_FLOAT, obs0y, &obstacles[0].y)
PARAM_ADD(PARAM_UINT8, obs0en, &obstacles[0].enabled)

PARAM_ADD(PARAM_FLOAT, obs1x, &obstacles[1].x)
PARAM_ADD(PARAM_FLOAT, obs1y, &obstacles[1].y)
PARAM_ADD(PARAM_UINT8, obs1en, &obstacles[1].enabled)

PARAM_ADD(PARAM_FLOAT, obs2x, &obstacles[2].x)
PARAM_ADD(PARAM_FLOAT, obs2y, &obstacles[2].y)
PARAM_ADD(PARAM_UINT8, obs2en, &obstacles[2].enabled)

PARAM_GROUP_STOP(g2lp)

LOG_GROUP_START(g2lp)
LOG_ADD(LOG_FLOAT, vx, &vel_desired.x)
LOG_ADD(LOG_FLOAT, vy, &vel_desired.y)
LOG_ADD(LOG_UINT32, ticks, &ticks)
LOG_GROUP_STOP(g2lp)
