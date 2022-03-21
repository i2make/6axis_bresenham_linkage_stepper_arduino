
#ifndef STEPPER_MOTOR_STEPPERMOTOR_HPP
#define STEPPER_MOTOR_STEPPERMOTOR_HPP

#include "StepperDefine.hpp"
#include "direction_pulse.hpp"

struct Motor
{
    Motor(void(* direction)(int), void(* pulse)()); // constructor

    void (* direction)(int);    // direction function
    void (* pulse)();           // pulse function

    long currentPosition;       // current position (현재 절대 위치 )
    long targetPosition;        // target position (가야할 절대 위치)
    long dy;                    // delta y = current position - newPosition
    long absDy;                 // abs(dy)

    short dirValue;             // position add dir (dir = 1 or -1) (방향에 따라 값을 증감)
    long over;                  // using bresenham line's parameter
};

#endif //STEPPER_MOTOR_STEPPERMOTOR_HPP
