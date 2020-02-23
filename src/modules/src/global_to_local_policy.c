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
#include <stdlib.h>

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
static uint8_t num_neighbors = 6;
static uint8_t my_id;

static struct vec last_goal;
static struct vec last_pos;
static struct vec last_vel;
static struct vec acc_desired;

struct obstacle {
  float x;
  float y;
  uint8_t enabled;
};
#define MAX_OBSTACLES (4)
static struct obstacle obstacles[MAX_OBSTACLES];

#define MAX_NEIGHBORS_ARRAY (20)
struct dist_idx_pair {
  float dist;
  uint8_t idx;
};
static struct dist_idx_pair dist_idx_array[MAX_NEIGHBORS_ARRAY];

// Timing
// 0 neighbor: 2.6ms
// 1 neighbor: 3.4ms
// 2 neighbor: 4.2ms
// 3 neighbor: 5.0ms


// private functions
static void globalToLocalPolicyTask(void * prm);
static int compare(const void* a, const void* b);

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

void globalToLocalPolicyGet(const struct vec* pos, const struct vec* vel, const struct vec* goal, struct vec* acc)
{
  if (enableNN > 0) {
    last_pos = *pos;
    last_vel = *vel;
    last_goal = *goal;
    *acc = acc_desired;
  }
}

static void recompute(void)
{
  uint64_t startTime = usecTimestamp();

  if (enableNN > 0) {

    // start NN evaluation
    nn_reset();

    // find all neighbors within Rsense
    float Rsense = nn_get_Rsense();
    uint8_t num_neighbors_found = 0;
    for (uint8_t idx = 0; ; ++idx) {

      struct allCfState* otherState = locSrvGetStateByIdx(idx);
      if (!otherState) {
        break;
      }

      // ignore myself and agents that have not received an update for 500ms
      if (   otherState->id != my_id
          && otherState->id != 0
          && xTaskGetTickCount() - otherState->timestamp < 500) {

        struct vec dpos = vsub(otherState->pos, last_pos);
        float dist = sqrtf(dpos.x * dpos.x + dpos.y * dpos.y);
        if (dist < Rsense) {
          dist_idx_array[num_neighbors_found].dist = dist;
          dist_idx_array[num_neighbors_found].idx = idx;
          num_neighbors_found++;
          if (num_neighbors_found >= MAX_NEIGHBORS_ARRAY) {
            break;
          }
        }
      }
    }

    // sort neighbors
    qsort(dist_idx_array, num_neighbors_found, sizeof(struct dist_idx_pair), &compare);

    // add only closest neighbors
    for (uint8_t idx = 0; idx < num_neighbors && idx < num_neighbors_found; ++idx) {
      struct allCfState* otherState = locSrvGetStateByIdx(dist_idx_array[idx].idx);
      struct vec dpos = vsub(otherState->pos, last_pos);
      struct vec dvel = vsub(otherState->vel, last_vel);
      float input[4] = {dpos.x, dpos.y, dvel.x, dvel.y};
      nn_add_neighbor(input);
    }

    // add all obstacles within Rsense
    // TODO: technically we should sort here, too, but currently MAX_OBSTACLES < max_neighbors
    for (int i = 0; i < MAX_OBSTACLES; ++i) {
      if (obstacles[i].enabled) {
        struct vec dpos = vsub(mkvec(obstacles[i].x,obstacles[i].y,0), last_pos);
        float dist = sqrtf(dpos.x * dpos.x + dpos.y * dpos.y);
        if (dist < Rsense) {
          float input[4] = {dpos.x, dpos.y, last_vel.x, last_vel.y};
          nn_add_obstacle(input);
        }
      }
    }

    // compute and set normalized goal
    struct vec dpos = vsub(last_goal, last_pos);
    float dist = sqrtf(dpos.x * dpos.x + dpos.y * dpos.y);
    if (dist > 0 && dist > Rsense) {
      dpos = vscl(Rsense / dist, dpos);
    }
    float input[4] = {dpos.x, dpos.y, -last_vel.x, -last_vel.y};
    const float* acc = nn_eval(input);

    acc_desired.x = scale * acc[0];
    acc_desired.y = scale * acc[1];
    acc_desired.z = 0;
  } else {
    acc_desired = vzero();
  }

  ticks = usecTimestamp() - startTime;
}

void globalToLocalPolicyTask(void * prm)
{
  uint32_t lastWakeTime = xTaskGetTickCount();
  // execute at 40 Hz to match simulation
  while(1) {
    recompute();
    vTaskDelayUntil(&lastWakeTime, F2T(40));
  }
}

static int compare(const void* a, const void* b)
{
  const struct dist_idx_pair* a_typed = (const struct dist_idx_pair*)a;
  const struct dist_idx_pair* b_typed = (const struct dist_idx_pair*)b;

  if (a_typed->dist < b_typed->dist) return -1;
  if (a_typed->dist > b_typed->dist) return 1;
  return 0;
}


PARAM_GROUP_START(g2lp)
PARAM_ADD(PARAM_UINT8, enableNN, &enableNN)
PARAM_ADD(PARAM_FLOAT, scale, &scale)
PARAM_ADD(PARAM_UINT8, num_neighbors, &num_neighbors)

PARAM_ADD(PARAM_FLOAT, obs0x, &obstacles[0].x)
PARAM_ADD(PARAM_FLOAT, obs0y, &obstacles[0].y)
PARAM_ADD(PARAM_UINT8, obs0en, &obstacles[0].enabled)

PARAM_ADD(PARAM_FLOAT, obs1x, &obstacles[1].x)
PARAM_ADD(PARAM_FLOAT, obs1y, &obstacles[1].y)
PARAM_ADD(PARAM_UINT8, obs1en, &obstacles[1].enabled)

PARAM_ADD(PARAM_FLOAT, obs2x, &obstacles[2].x)
PARAM_ADD(PARAM_FLOAT, obs2y, &obstacles[2].y)
PARAM_ADD(PARAM_UINT8, obs2en, &obstacles[2].enabled)

PARAM_ADD(PARAM_FLOAT, obs3x, &obstacles[3].x)
PARAM_ADD(PARAM_FLOAT, obs3y, &obstacles[3].y)
PARAM_ADD(PARAM_UINT8, obs3en, &obstacles[3].enabled)

PARAM_GROUP_STOP(g2lp)

LOG_GROUP_START(g2lp)
LOG_ADD(LOG_FLOAT, ax, &acc_desired.x)
LOG_ADD(LOG_FLOAT, ay, &acc_desired.y)
LOG_ADD(LOG_UINT32, ticks, &ticks)
LOG_GROUP_STOP(g2lp)
