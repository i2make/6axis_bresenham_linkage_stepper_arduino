#ifndef STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
#define STEPPER_MOTOR_STEPPERMOTORWORLD_HPP

#include <StepperMotor.hpp>

struct World
{
    // method
    World(Motor* _motor);                   // constructor

    Motor* motor[MAX_AXIS];                 // number of motor

    bool addMotor(Motor* _motor);           // adding motors

    void setMaxLongDy();         // find maxDy in the motors

    void bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2);

    void generatePulse();       // one pulse generate

    void setMotorDirection();   // set direction of motors

    float setDelay();           // setting delay for N step

    void moving(long _x, long _y, long _z, long _a, long _b, long _c, void(*)());

    void moving(long _x, long _y, long _z, long _a, long _b, long _c);

    void pauseMoving();         // pause method

    void resumeMoving();        // resume method

    // variable
    unsigned short motorIndex;          // added motor number
    unsigned short maxDyMotor;          // max long distance moving motor
    unsigned long  currentDelay         // setting delay value for current speed

    unsigned long stepCounter;          // Number of steps to move
    unsigned int accelNstep;            // n step of acceleration
    unsigned int decelNstep;            // n step of deceleration
    unsigned int resumeAccelNstep;      // using resume acceleration
    unsigned int pauseDecelNstep;       // using pause deceleration
    volatile float delayValue;          // current delay value

    volatile bool movementDone;         // complete move
    volatile bool pause;                // pause flag
    volatile bool resume;               // resume flag
    volatile bool resumeReady;          // resume after pause

    // 0000 1000    : movementDone
    // 0000 0100    : resumeReady
    // 0000 0010    : resume
    // 0000 0001    : pause
    bool movingFlag;                    // flag for moving status


};

extern TM1638QYF module;
extern long xx1;
extern long xx2;
extern unsigned int mode;


#endif //STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
