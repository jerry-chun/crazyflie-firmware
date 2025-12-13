#ifndef PASSIVE_SENSING_H
#define PASSIVE_SENSING_H

#include "stabilizer_types.h"

/**
 * Output struct for passive sensing.
 * All values are in the body frame.
 */
typedef struct {
  Axis3f force;      // estimated external force [N] (body frame)
  Axis3f torque;     // external torque [N·m] (not implemented)
  float accMag;      // ||acc|| [G]
  float gyroEnergy;  // ||gyro|| [deg/s]
} passiveSensingOutput_t;

void passiveSensingInit(void);

void passiveSensingUpdate(const sensorData_t* sensorData,
                          const state_t* state,
                          const control_t* control,
                          passiveSensingOutput_t* out,
                          const float dt);

#endif // PASSIVE_SENSING_H
