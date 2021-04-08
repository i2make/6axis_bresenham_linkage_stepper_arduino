
#ifndef STEPPER_MOTOR_STEPPERMOTOR_HPP
#define STEPPER_MOTOR_STEPPERMOTOR_HPP

#include <StepperDefine.hpp>

struct Motor
{
    Motor(void(* direction)(int), void(* pulse)());

    void (* direction)(int);    // direction function
    void (* pulse)();           // pulse function

    long dx;                    // dx
    long newDx;                 // target position
    long dy;                    // delta y = position - newPosition
    long absDy;                 // abs(dy)
    int dirValue;               // position add dir (dir = 1 or -1)
    long over;                  // bresenham line's parameter

    float delayC0;              // initial delay value
    float delayValue;           // current delay value
    long delayN;                // number of acceleration step
};

#endif //STEPPER_MOTOR_STEPPERMOTOR_HPP
