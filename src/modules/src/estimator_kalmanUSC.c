/**
 * EKF author:
 * James Preiss
 * University of Southern California
 *
 */

#include "FreeRTOS.h"
#include "task.h"

#include "ekf.h"
// #include "mathconstants.h"
#include "stabilizer_types.h"
#include "sensors.h"
#include "param.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

#define GRAV (9.81f)


// EKF implementation uses double-buffered approach
static struct ekf ekfa;
static struct ekf ekfb;
static struct ekf *ekf_front = &ekfa;
static struct ekf *ekf_back = &ekfb;
static void ekf_flip()
{
	struct ekf *ekf_temp = ekf_front;
	ekf_front = ekf_back;
	ekf_back = ekf_temp;
}
static bool initialized = false;
static bool resetEstimation = false;
static bool useFakeVel = true; // estimate velocity from the mocap and fuse it in the filter
static struct vec initialPos;

static struct vec lastPos;
static uint64_t lastTime;

// TODO change EKF funcs to take struct vec,
// then make a function that wraps positionExternalBringupGetLastData

#define ATTITUDE_UPDATE_RATE RATE_500_HZ
#define ATTITUDE_UPDATE_DT (1.0/ATTITUDE_UPDATE_RATE)

static struct vec3_s ekf2vec(struct vec v)
{
	struct vec3_s v3s = { .x = v.x, .y = v.y, .z = v.z };
	return v3s;
}

static xQueueHandle measurementsQueue;
#define MEASUREMENTS_QUEUE_LENGTH (10)

enum measurementType_e
{
	measurementPosition,
	measurementPose,
};

struct measurement
{
	enum measurementType_e type;
	union {
		positionMeasurement_t position;
		poseMeasurement_t pose;
	};
};

static struct vec estimateVelocity(struct vec pos)
{
	struct vec result;
	if (lastTime != 0)
	{
		float dt = (xTaskGetTickCount() - lastTime) / 1000.0f;
		dt = fmax(dt, 0.005);
		result = vdiv(vsub(pos, lastPos), dt);
	} else {
		result = vzero();
	}
	lastPos = pos;
	lastTime = xTaskGetTickCount();
	return result;
}

// public functions

void estimatorKalmanUSCInit(void)
{
	// Initialize to 0 so gyro integration still works without Vicon
	float init[] = {0, 0, 0, 1};
	ekf_init(ekf_back, init, init, init);

	measurementsQueue = xQueueCreate(MEASUREMENTS_QUEUE_LENGTH, sizeof(struct measurement));

	initialized = true;
}

bool estimatorKalmanUSCTest(void)
{
	// Nothing to test...
	return initialized;
}

void estimatorKalmanUSC(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
	// In this new design the state estimator had to get sensor data...
	sensorsReadAcc(&sensors->acc);
	sensorsReadGyro(&sensors->gyro);
	sensorsReadMag(&sensors->mag);

	// rate limit
	if (!RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
		return;
	}

	if (resetEstimation) {
		float init[] = {0, 0, 0, 1};
		ekf_init(ekf_back, &initialPos.x, init, init);
		lastTime = 0;
		lastPos = vzero();
		xQueueReset(measurementsQueue);
		resetEstimation = false;
	}

	float acc[3] = {sensors->acc.x * GRAV, sensors->acc.y * GRAV, sensors->acc.z * GRAV};
	float gyro[3] = {radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z)};
	ekf_imu(ekf_back, ekf_front, acc, gyro, ATTITUDE_UPDATE_DT);
	ekf_flip();

	struct measurement m;
	while (pdTRUE == xQueueReceive(measurementsQueue, &m, 0)) {
		switch(m.type) {
			case measurementPosition:
				if (useFakeVel) {
					struct vec v = estimateVelocity(vloadf(m.position.pos));
					ekf_position_and_vel(ekf_back, ekf_front, m.position.pos, &v.x);
				} else {
					ekf_position(ekf_back, ekf_front, m.position.pos);
				}
				break;
			case measurementPose:
				{
					if (useFakeVel) {
						struct vec v = estimateVelocity(vloadf(m.pose.pos));
						ekf_pose_and_vel(ekf_back, ekf_front, m.pose.pos, &v.x, &m.pose.quat.x);
					} else {
						ekf_pose(ekf_back, ekf_front, m.pose.pos, &m.pose.quat.x);
					}
				}
				break;
		}
		ekf_flip();
	}

	state->position = ekf2vec(ekf_back->pos);
	state->velocity = ekf2vec(ekf_back->vel);
	state->acc = ekf2vec(vscl(1.0f / GRAV, ekf_back->acc));

	struct vec rpy = quat2rpy(ekf_back->quat);
	state->attitude.roll = degrees(rpy.x);
	state->attitude.pitch = -degrees(rpy.y);
	state->attitude.yaw = degrees(rpy.z);

	state->attitudeQuaternion.x = ekf_back->quat.x;
	state->attitudeQuaternion.y = ekf_back->quat.y;
	state->attitudeQuaternion.z = ekf_back->quat.z;
	state->attitudeQuaternion.w = ekf_back->quat.w;

	// state->attitudeRate.roll = gyro[0];
	// state->attitudeRate.pitch = -gyro[1];
	// state->attitudeRate.yaw = gyro[2];
}

static bool stateEstimatorUSCEnqueueExternalMeasurement(void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(measurementsQueue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(measurementsQueue, measurement, 0);
  }
  return (result==pdTRUE);
}

bool estimatorKalmanUSCEnqueuePosition(const positionMeasurement_t *pos)
{
	struct measurement m = {
		.type = measurementPosition,
		.position = *pos
	};
	return stateEstimatorUSCEnqueueExternalMeasurement((void *)&m);
}

bool estimatorKalmanUSCEnqueuePose(const poseMeasurement_t *pose)
{
	struct measurement m = {
		.type = measurementPose,
		.pose = *pose
	};
	return stateEstimatorUSCEnqueueExternalMeasurement((void *)&m);
}

PARAM_GROUP_START(kalmanUSC)
  PARAM_ADD(PARAM_UINT8, useFakeVel, &useFakeVel)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_FLOAT, initialX, &initialPos.x)
  PARAM_ADD(PARAM_FLOAT, initialY, &initialPos.y)
  PARAM_ADD(PARAM_FLOAT, initialZ, &initialPos.z)
PARAM_GROUP_STOP(kalmanUSC)