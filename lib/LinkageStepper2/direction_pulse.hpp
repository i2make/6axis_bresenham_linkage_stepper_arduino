
#ifndef STEPPER_MOTOR_LIBRARY3_DIRECTION_PULSE_HPP
#define STEPPER_MOTOR_LIBRARY3_DIRECTION_PULSE_HPP


#include "StepperDefine.hpp"
#define DELAY_BETWEEN_PULSE         3

/// X axis 1 stepping pulse generation
void xPulse();

/// X axis direction selection
void xDirection(int dir);

/// Y axis 1 stepping pulse generation
void yPulse();

/// Y axis direction selection
void yDirection(int dir);

/// Z axis 1 stepping pulse generation
void zPulse();

/// Z axis direction selection
void zDirection(int dir);

/// A axis 1 stepping pulse generation
void aPulse();

/// A axis direction selection
void aDirection(int dir);

/// B axis 1 stepping pulse generation
void bPulse();

/// B axis direction selection
void bDirection(int dir);

/// C axis 1 stepping pulse generation
void cPulse();

/// C axis direction selection
void cDirection(int dir);


#endif //STEPPER_MOTOR_LIBRARY3_DIRECTION_PULSE_HPP
