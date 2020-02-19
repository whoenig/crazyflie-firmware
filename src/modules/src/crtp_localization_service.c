/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "crtp_localization_service.h"
#include "log.h"
#include "param.h"

#include "stabilizer_types.h"
#include "stabilizer.h"
#include "configblock.h"

#include "locodeck.h"

#include "estimator.h"
#include "quatcompress.h"

#define NBR_OF_RANGES_IN_PACKET   5
#define DEFAULT_EMERGENCY_STOP_TIMEOUT (1 * RATE_MAIN_LOOP)

typedef enum
{
  EXT_POSITION        = 0,
  GENERIC_TYPE        = 1,
  EXT_POSITION_PACKED = 2,
} locsrvChannels_t;

typedef struct
{
  uint8_t type;
  struct
  {
    uint8_t id;
    float range;
  } __attribute__((packed)) ranges[NBR_OF_RANGES_IN_PACKET];
} __attribute__((packed)) rangePacket;

// up to 4 items per CRTP packet
typedef struct {
  uint8_t id; // last 8 bit of the Crazyflie address
  int16_t x; // mm
  int16_t y; // mm
  int16_t z; // mm
} __attribute__((packed)) extPositionPackedItem;

// up to 2 items per CRTP packet
typedef struct {
  uint8_t id; // last 8 bit of the Crazyflie address
  int16_t x; // mm
  int16_t y; // mm
  int16_t z; // mm
  uint32_t quat; // compressed quaternion, see quatcompress.h
} __attribute__((packed)) extPosePackedItem;

// Struct for logging position information
static positionMeasurement_t ext_pos;
// Struct for logging pose information
static poseMeasurement_t ext_pose;

static CRTPPacket pkRange;
static uint8_t rangeIndex;
static bool enableRangeStreamFloat = false;
static float extPosStdDev = 0.01;
static float extQuatStdDev = 4.5e-3;
static bool isInit = false;
static uint8_t my_id;
static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;

  // compressed quaternion, see quatcompress.h
  int32_t quat;

  // lower 16 bit of timestamp (in ms)
  uint16_t time;
} stateCompressed;

static uint32_t time;

// Keeping track of neighbors
static float alpha = 0.8;
static float max_v = 0.5; // m/s

#define NUM_MAX_NEIGHBORS 20
static struct allCfState all_states[NUM_MAX_NEIGHBORS];

static void locSrvCrtpCB(CRTPPacket* pk);
static void extPositionHandler(CRTPPacket* pk);
static void genericLocHandle(CRTPPacket* pk);
static void extPositionPackedHandler(CRTPPacket* pk);

void locSrvInit()
{
  if (isInit) {
    return;
  }

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;

  memset(all_states, 0, sizeof(all_states));

  crtpRegisterPortCB(CRTP_PORT_LOCALIZATION, locSrvCrtpCB);
  isInit = true;
}

static void locSrvCrtpCB(CRTPPacket* pk)
{
  switch (pk->channel)
  {
    case EXT_POSITION:
      extPositionHandler(pk);
      break;
    case GENERIC_TYPE:
      genericLocHandle(pk);
      break;
    case EXT_POSITION_PACKED:
      extPositionPackedHandler(pk);
      break;
    default:
      break;
  }
}

static void extPositionHandler(CRTPPacket* pk)
{
  const struct CrtpExtPosition* data = (const struct CrtpExtPosition*)pk->data;

  ext_pos.x = data->x;
  ext_pos.y = data->y;
  ext_pos.z = data->z;
  ext_pos.stdDev = extPosStdDev;
  estimatorEnqueuePosition(&ext_pos);

  stateCompressed.x = ext_pos.x * 1000.0f;
  stateCompressed.y = ext_pos.y * 1000.0f;
  stateCompressed.z = ext_pos.z * 1000.0f;
  stateCompressed.quat = 0;
  stateCompressed.time = xTaskGetTickCount();
}

static void genericLocHandle(CRTPPacket* pk)
{
  uint8_t type = pk->data[0];
  if (pk->size < 1) return;

  if (type == LPS_SHORT_LPP_PACKET && pk->size >= 2) {
    bool success = lpsSendLppShort(pk->data[1], &pk->data[2], pk->size-2);

    pk->port = CRTP_PORT_LOCALIZATION;
    pk->channel = GENERIC_TYPE;
    pk->size = 3;
    pk->data[2] = success?1:0;
    crtpSendPacket(pk);
  } else if (type == EMERGENCY_STOP) {
    stabilizerSetEmergencyStop();
  } else if (type == EMERGENCY_STOP_WATCHDOG) {
    stabilizerSetEmergencyStopTimeout(DEFAULT_EMERGENCY_STOP_TIMEOUT);
  } else if (type == EXT_POSE) {
    const struct CrtpExtPose* data = (const struct CrtpExtPose*)&pk->data[1];
    ext_pose.x = data->x;
    ext_pose.y = data->y;
    ext_pose.z = data->z;
    ext_pose.quat.x = data->qx;
    ext_pose.quat.y = data->qy;
    ext_pose.quat.z = data->qz;
    ext_pose.quat.w = data->qw;
    ext_pose.stdDevPos = extPosStdDev;
    ext_pose.stdDevQuat = extQuatStdDev;
    estimatorEnqueuePose(&ext_pose);

    stateCompressed.x = ext_pose.x * 1000.0f;
    stateCompressed.y = ext_pose.y * 1000.0f;
    stateCompressed.z = ext_pose.z * 1000.0f;
    float const q[4] = {
      ext_pose.quat.x,
      ext_pose.quat.y,
      ext_pose.quat.z,
      ext_pose.quat.w};
    stateCompressed.quat = quatcompress(q);
    stateCompressed.time = xTaskGetTickCount();
  } else if (type == EXT_POSE_PACKED) {
    uint8_t numItems = (pk->size - 1) / sizeof(extPosePackedItem);
    for (uint8_t i = 0; i < numItems; ++i) {
      const extPosePackedItem* item = (const extPosePackedItem*)&pk->data[1 + i * sizeof(extPosePackedItem)];
      if (item->id == my_id) {
        ext_pose.x = item->x / 1000.0f;
        ext_pose.y = item->y / 1000.0f;
        ext_pose.z = item->z / 1000.0f;
        quatdecompress(item->quat, (float *)&ext_pose.quat.q0);
        ext_pose.stdDevPos = extPosStdDev;
        ext_pose.stdDevQuat = extQuatStdDev;
        estimatorEnqueuePose(&ext_pose);

        stateCompressed.x = item->x;
        stateCompressed.y = item->y;
        stateCompressed.z = item->z;
        stateCompressed.quat = item->quat;
        stateCompressed.time = xTaskGetTickCount();
      }
      struct allCfState* state = locSrvGetStateByCfId(item->id);
      if (state)
      {
        struct vec pos = vdiv(mkvec(item->x, item->y, item->z), 1000.0f);
        uint64_t lastTime = state->timestamp;
        if (lastTime != 0) {
          float dt = (xTaskGetTickCount() - lastTime) / 1000.0f;
          if (dt > 0) {
            struct vec vel = vdiv(vsub(pos, state->pos), dt);

            vel = vclampabs(vel, vrepeat(max_v));
            state->vel = vadd(vscl(alpha, vel), vscl(1-alpha, state->vel));
            // state->vel = vel;
          }
        }
        state->pos = pos;
        state->timestamp = xTaskGetTickCount();
      }
    }
  }
}

static void extPositionPackedHandler(CRTPPacket* pk)
{
  uint8_t numItems = pk->size / sizeof(extPositionPackedItem);
  for (uint8_t i = 0; i < numItems; ++i) {
    const extPositionPackedItem* item = (const extPositionPackedItem*)&pk->data[i * sizeof(extPositionPackedItem)];
    if (item->id == my_id) {
      ext_pos.x = item->x / 1000.0f;
      ext_pos.y = item->y / 1000.0f;
      ext_pos.z = item->z / 1000.0f;
      ext_pos.stdDev = extPosStdDev;
      estimatorEnqueuePosition(&ext_pos);
      stateCompressed.x = item->x;
      stateCompressed.y = item->y;
      stateCompressed.z = item->z;
      stateCompressed.quat = 0;
      time = xTaskGetTickCount();
      stateCompressed.time = xTaskGetTickCount();
    }
    struct allCfState* state = locSrvGetStateByCfId(item->id);
    if (state)
    {
      struct vec pos = vdiv(mkvec(item->x, item->y, item->z), 1000.0f);
      uint64_t lastTime = state->timestamp;
      if (lastTime != 0) {
        float dt = (xTaskGetTickCount() - lastTime) / 1000.0f;
        if (dt > 0) {
          struct vec vel = vdiv(vsub(pos, state->pos), dt);

          vel = vclampabs(vel, vrepeat(max_v));
          state->vel = vadd(vscl(alpha, vel), vscl(1-alpha, state->vel));
          // state->vel = vel;
        }
      }
      state->pos = pos;
      state->timestamp = xTaskGetTickCount();
    }
  }
}

void locSrvSendPacket(locsrv_t type, uint8_t *data, uint8_t length)
{
  CRTPPacket pk;

  ASSERT(length < CRTP_MAX_DATA_SIZE);

  pk.port = CRTP_PORT_LOCALIZATION;
  pk.channel = GENERIC_TYPE;
  memcpy(pk.data, data, length);
  crtpSendPacket(&pk);
}

void locSrvSendRangeFloat(uint8_t id, float range)
{
  rangePacket *rp = (rangePacket *)pkRange.data;

  ASSERT(rangeIndex <= NBR_OF_RANGES_IN_PACKET);

  if (enableRangeStreamFloat)
  {
    rp->ranges[rangeIndex].id = id;
    rp->ranges[rangeIndex].range = range;
    rangeIndex++;

    if (rangeIndex >= 5)
    {
      rp->type = RANGE_STREAM_FLOAT;
      pkRange.port = CRTP_PORT_LOCALIZATION;
      pkRange.channel = GENERIC_TYPE;
      pkRange.size = sizeof(rangePacket);
      crtpSendPacket(&pkRange);
      rangeIndex = 0;
    }
  }
}

struct allCfState* locSrvGetStateByCfId(uint8_t cfid)
{
  for (uint8_t i = 0; i < NUM_MAX_NEIGHBORS; ++i) {
    if (all_states[i].id == 0 || all_states[i].id == cfid) {
      all_states[i].id = cfid;
      return &all_states[i];
    }
  }
  return NULL;
}

struct allCfState* locSrvGetStateByIdx(uint8_t idx)
{
  if (idx < NUM_MAX_NEIGHBORS) {
    return &all_states[idx];
  }
  return NULL;
}

LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)


LOG_GROUP_START(locSrvZ)
  LOG_ADD(LOG_INT16, x, &stateCompressed.x)                 // position - mm
  LOG_ADD(LOG_INT16, y, &stateCompressed.y)
  LOG_ADD(LOG_INT16, z, &stateCompressed.z)

  LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)           // compressed quaternion, see quatcompress.h

  LOG_ADD(LOG_UINT16, tick, &stateCompressed.time)  // time when data was received last (ms/ticks)
LOG_GROUP_STOP(locSrvZ)

PARAM_GROUP_START(locSrv)
  PARAM_ADD(PARAM_UINT8, enRangeStreamFP32, &enableRangeStreamFloat)
  PARAM_ADD(PARAM_FLOAT, extPosStdDev, &extPosStdDev)
  PARAM_ADD(PARAM_FLOAT, extQuatStdDev, &extQuatStdDev)
  
  PARAM_ADD(PARAM_FLOAT, alpha, &alpha)
  PARAM_ADD(PARAM_FLOAT, max_v, &max_v)
PARAM_GROUP_STOP(locSrv)
