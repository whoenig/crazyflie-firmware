#ifndef __ESTIMATOR_KALMAN_USC_H__
#define __ESTIMATOR_KALMAN_USC_H__

#include <stdint.h>
#include "stabilizer_types.h"

void estimatorKalmanUSCInit(void);
bool estimatorKalmanUSCTest(void);
void estimatorKalmanUSC(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);

/**
 * The filter supports the incorporation of additional sensors into the state estimate via the following functions:
 */
bool estimatorKalmanUSCEnqueuePosition(const positionMeasurement_t *pos);
bool estimatorKalmanUSCEnqueuePose(const poseMeasurement_t *pose);

#endif // __ESTIMATOR_KALMAN_USC_H__