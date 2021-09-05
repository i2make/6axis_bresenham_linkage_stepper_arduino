
#ifndef STEPPER_MOTOR_STEPPERMOTOR_HPP
#define STEPPER_MOTOR_STEPPERMOTOR_HPP

#include "StepperDefine.hpp"

struct Motor
{
    Motor(void(* direction)(int), void(* pulse)());

    void (* direction)(int);    // direction function
    void (* pulse)();           // pulse function

    long currentPosition;                    // current position
    long targetPosition;                 // target position
    long dy;                    // delta y = current position - newPosition
    long absDy;                 // abs(dy)

    short dirValue;               // position add dir (dir = 1 or -1)
    long over;                  // bresenham line's parameter
};

#endif //STEPPER_MOTOR_STEPPERMOTOR_HPP
