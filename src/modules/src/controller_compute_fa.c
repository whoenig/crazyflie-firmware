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

#include "param.h"
#include "log.h"
#include "math3d.h"

#define THREE_AGENT_NN

#ifdef THREE_AGENT_NN
  #include "three_dim6_sn_1.h"
#else
  #include "dim6_sn_6.h"
  #include "tau_dim6_sn_1.h"
#endif

#include "usec_time.h"
#include "crtp_localization_service.h"
#include "configblock.h"

#include "FreeRTOS.h"
#include "task.h"

static struct vec Fa;
static struct vec tau_a;
static uint8_t enableNN = 0;

#ifdef THREE_AGENT_NN
  static net_outputs nnOutput;
  static float nnInput[6];
#else
  static net_outputs control_n;
  static net_outputs2 control_n2;
  static float input[6];
#endif


static uint8_t my_id;

void controllerComputeFaInit(void)
{
  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
}

void controllerComputeFa(const state_t *state, struct vec* F_d)
{
  // Compute Fa
  // inputs: x2 - x1, y2 - y1, z2 - z1
  // valid range: -0.15 -- 0.15; -0.2 -- 0.2; 0.2 -- 0.7

  Fa = vzero();
  tau_a = vzero();

  if (enableNN > 0) {

    #ifdef THREE_AGENT_NN

    nnInput[0] = 0.2;
    nnInput[1] = 0.6;
    nnInput[2] = 0.7;
    nnInput[3] = 0.2;
    nnInput[4] = 0.6;
    nnInput[5] = 0.7;
    int idx = 0;

    for (int id = 0; id < MAX_CF_ID; ++id) {
      // dx: 0.2, dy: 0.6, dz: 0---0.7

      // ignore myself and agents that have not received an update for 500ms
      if (id != my_id
          && xTaskGetTickCount() - all_states[id].timestamp < 500) {

        struct vec dpos = vsub(all_states[id].pos, all_states[my_id].pos);
        if (fabsf(dpos.x) <= 0.2f && fabsf(dpos.y) <= 0.6f && dpos.z >= 0.0f && dpos.z <= 0.7f) {
          nnInput[idx++] = dpos.x;
          nnInput[idx++] = dpos.y;
          nnInput[idx++] = dpos.z;

          if (idx > 5) {
            break;
          }
        }
      }
    }

    if (idx > 0) {
      network(&nnOutput, nnInput);
      Fa = mkvec(0, 0, nnOutput.out_0);
    }
    #else
    // find most important neighbor
    float lowestZ = 10;
    for (int id = 0; id < MAX_CF_ID; ++id) {

      // ignore myself and agents that have not received an update for 500ms
      if (id != my_id
          && xTaskGetTickCount() - all_states[id].timestamp < 500) {

        struct vec dpos = vsub(all_states[id].pos, all_states[my_id].pos);
        if (fabsf(dpos.x) <= 0.2f && fabsf(dpos.y) <= 0.2f && dpos.z >= 0.0f && dpos.z <= 0.7f) {
          if (dpos.z < lowestZ) {
            lowestZ = dpos.z;

            input[0] = dpos.x;
            input[1] = dpos.y;
            input[2] = dpos.z;
            input[3] = state->attitude.roll / 10;
            input[4] = state->attitude.pitch / 10;
            input[5] = state->attitude.yaw / 10;
          }
        }
      }
    }

    // if there was a neighbor, evaluate the NN to compuate Fa
    if (lowestZ < 10) {
      network(&control_n, input);
      Fa = mkvec(control_n.out_0, control_n.out_1, control_n.out_2);

      network2(&control_n2, input);
      tau_a = vdiv(mkvec(control_n.out_0, control_n.out_1, control_n.out_2), 1000.0f);
    }
    #endif

    if (enableNN > 1) {
      // Apply Fa (convert Fa to N first)
      *F_d = vsub(*F_d, vscl(9.81f / 1000.0f, Fa));
    }
  }
}


PARAM_GROUP_START(ctrlFa)
PARAM_ADD(PARAM_UINT8, enableNN, &enableNN)
PARAM_GROUP_STOP(ctrlFa)

LOG_GROUP_START(ctrlFa)

LOG_ADD(LOG_FLOAT, Fax, &Fa.x)
LOG_ADD(LOG_FLOAT, Fay, &Fa.y)
LOG_ADD(LOG_FLOAT, Faz, &Fa.z)

LOG_ADD(LOG_FLOAT, tauax, &tau_a.x)
LOG_ADD(LOG_FLOAT, tauay, &tau_a.y)
LOG_ADD(LOG_FLOAT, tauaz, &tau_a.z)

LOG_GROUP_STOP(ctrlFa)
