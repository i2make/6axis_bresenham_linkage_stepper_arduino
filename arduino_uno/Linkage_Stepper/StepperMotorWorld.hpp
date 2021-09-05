#ifndef STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
#define STEPPER_MOTOR_STEPPERMOTORWORLD_HPP

#include "StepperMotor.hpp"

// moving flag define
#define pause_msk                       (1u << 0u)
#define resume_msk                      (1u << 1u)
#define reachedHalfStep_msk             (1u << 2u)
#define movementDone_msk                (1u << 3u)
#define acceleration_msk                (1u << 4u)
#define constantSpeed_msk               (1u << 5u)
#define deceleration_msk                (1u << 6u)
#define reachedMaxSpeed_msk             (1u << 7u)


struct World
{
// method ///////////////////////////////////////////////////////////
public:

    World(Motor* _motor);                   // constructor

    Motor* motor[MAX_AXIS];                 // number of motor

    bool addMotor(Motor* _motor);           // adding motors

    void generatePulse();       // one pulse generate

    void setMotorDirection();   // set direction of motors

    float setDelay();           // setting delay for N step

    float setDelay2();          // testing

    void setSpeed(float speed);

    void
    moving(long _x, long _y, long _z, long _a, long _b, long _c,
           void(*)());

    void pauseMoving();         // pause method

    void resumeMoving();        // resume method

    bool movingDone();          // moving done ?

private:

    void setMaxLongDy();         // find maxDy in the motors

    void bresenham(long _x2, long _y2, long _z2, long _a2, long _b2,
                   long _c2);


    void calculateAccel(unsigned long counter) {
        delayValue = delayValue -
                     (2 * delayValue) / (4 * (float) counter + 1);
    }

    void calculateDecel(unsigned long counter) {
        delayValue = (delayValue * float(4 * counter + 1)) /
                     float(4 * counter + 1 - 2);
    }


// variable ////////////////////////////////////////////////////////
public:
    unsigned short motorIndex;          // added motor number
    unsigned short maxDyMotor;          // max long distance moving motor

    unsigned long targetCounter;
    unsigned long stepCounter;          // Number of steps to move
    unsigned long elapsedCounter;

    unsigned int accelNstep;            // n step of acceleration
    unsigned int decelNstep;            // n step of deceleration

    // not used setDelay2
    unsigned int resumeAccelNstep;      // using resume acceleration
    unsigned int pauseDecelNstep;       // using pause deceleration

    volatile float delayValue;          // current delay value

    // 0000 1000    : movementDone
    // 0000 0100    : resumeReady
    // 0000 0010    : resume
    // 0000 0001    : pause
    unsigned short movingFlag;           // flag for moving status

private:
    float minDelayValue;          // max speed
};

#endif //STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
