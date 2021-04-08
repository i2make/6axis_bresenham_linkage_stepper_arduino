
#ifndef STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
#define STEPPER_MOTOR_STEPPERMOTORWORLD_HPP

#include <StepperMotor.hpp>

struct World
{
    World(Motor* _motor);

    Motor* motor[MAX_AXIS];     // number of motor

    bool addMotor(Motor* _motor);

    unsigned short setMaxLongDy();         // find maxDy in the motors

    void bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2);

    void generatePulse();       // one pulse generate

    void setMotorDirection();   // set direction of motors

    void setDelay();

    void moving(long _x, long _y, long _z, long _a, long _b, long _c);

    // variable
    unsigned short motorIndex;
    unsigned short maxDyMotor;
    volatile unsigned int accelNstep;
    unsigned long decelNstep;

    long xPosition; // x axis position
    long yPosition; // y axis position
    long zPosition; // z axis position
    long aPosition; // a axis position
    long bPosition; // b axis position
    long cPosition; // c axis position

};



#endif //STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
