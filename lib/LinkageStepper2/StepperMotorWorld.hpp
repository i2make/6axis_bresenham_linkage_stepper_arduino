#ifndef STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
#define STEPPER_MOTOR_STEPPERMOTORWORLD_HPP

#include "StepperMotor.hpp"

// moving flag define
#define pause_msk                       (1u << 0u)
#define resume_msk                      (1u << 1u)
#define changeSpeed_msk                 (1u << 2u)
#define movementDone_msk                (1u << 3u)
#define acceleration_msk                (1u << 4u)
#define constantSpeed_msk               (1u << 5u)
#define deceleration_msk                (1u << 6u)
#define reachedMaxSpeed_msk             (1u << 7u)

#define ACCELERATING                (movingFlag & acceleration_msk)
#define CONSTANT_SPEED              (movingFlag & constantSpeed_msk)
#define DECELERATING                (movingFlag & deceleration_msk)
#define NOT_ACCELERATING            !(movingFlag & acceleration_msk)
#define NOT_CONSTANT_SPEED          !(movingFlag & constantSpeed_msk)
#define NOT_DECELERATING            !(movingFlag & deceleration_msk)

#define CLEAR_ACCELERATION_FLAG     movingFlag &= ~acceleration_msk
#define SET_ACCELERATION_FLAG       movingFlag |= acceleration_msk

#define CLEAR_CONSTANT_SPEED_FLAG   movingFlag &= ~constantSpeed_msk
#define SET_CONSTANT_SPEED_FLAG     movingFlag |= constantSpeed_msk

#define CLEAR_DECELERATION_FLAG     movingFlag &= ~deceleration_msk
#define SET_DECELERATION_FLAG       movingFlag |= deceleration_msk

#define CLEAR_RESUME_FLAG           movingFlag &= ~resume_msk

#define CLEAR_MOVING_DONE_FLAG      movingFlag &= ~movementDone_msk
#define SET_MOVING_DONE_FLAG        movingFlag |= movementDone_msk

#define BUTTON_PRESS_PAUSE          movingFlag & pause_msk
#define BUTTON_PRESS_RESUME         movingFlag & resume_msk

struct World {
/// method ///////////////////////////////////////////////////////////
public:

    explicit World(Motor *_motor);          // constructor

    Motor *motor[MAX_AXIS];                 // number of motor

    bool addMotor(Motor *_motor);           // adding motors

    void generatePulse();                   // one pulse generate

    void setMotorDirection();               // set direction of motors

    float setDelay2();                      // setting delay for N step

    void setSpeed(float speed);

    float getSpeed();

    void moving(long _x, long _y, long _z, long _a, long _b, long _c, void(*)());

    void pauseMoving();                     // pause method

    void resumeMoving();                    // resume method

    void changeSpeed();

    float readSpeedController();

    bool movingDone() const;                      // moving done?

private:

    void setMaxLongDy();         // find maxDy in the motors

    void bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2);

    void calculateAccel(unsigned long counter) {
        delayValue = delayValue - (2 * delayValue) / (4 * (float) counter + 1);
    }

    void calculateDecel(unsigned long counter) {
        delayValue = (delayValue * float(4 * counter + 1)) / float(4 * counter + 1 - 2);
    }

//
//
//           ________________________________________
//          /|                                      |\
//         / |                                      | \
//        /  |                                      |  \
//       /   |                                      |   \
//      /    |                                      |    \
//     /     |                                      |     \
//    /      |                                      |      \
//   /       |                                      |       \
// -------------------------------------------------------------------
//  |<------>| <-- accelNstep        decelNstep --> |<------>|
//  |--------------- totalMovedCounter --------------------->|
//  |--------------- stepCounter -------------->             |
//
// totalMovedCounter is the total number of moving steps
// stepCounter restarts from 0 when resume is executed after pause

// +-------------- Accelerating ----------------->
//
//   +---------+        +-------+      +-----+   +--+  +--+  +--+
//   |         |        |       |      |     |   |  |  |  |  |  |
//   |         |        |       |      |     |   |  |  |  |  |  |
//   |         |        |       |      |     |   |  |  |  |  |  |
// --+         +--------+       +------+     +---+  +--+  +--+  +--
//             |<------>|                                 |<>|
//                  ^                                      ^
//                  |                                      |
//              DELAY_C0                               MIN_DELAY
//

/// variable ////////////////////////////////////////////////////////
public:
    unsigned short motorIndex;          // added motor number
    unsigned short maxDyMotor;          // max long distance moving motor

    unsigned long totalMovedCounter;    // Total number of steps moved (총 이동 스텝 수)
    unsigned long stepCounter;          // number of moving steps to calculate accelNstep, decelNstep, elapsedCounter
    unsigned long elapsedCounter;       // summing elapsed step

    unsigned int accelNstep;            // n step of acceleration
    unsigned int decelNstep;            // n step of deceleration

    volatile float delayValue;          // current delay value
    volatile float delayValuePercent;   // current speed controlled delay value

    // 1000 0000    :
    // 0100 0000    : deceleration_msk
    // 0010 0000    : constantSpeed_msk
    // 0001 0000    : acceleration_msk
    // 0000 1000    : movementDone
    // 0000 0100    : changeSpeed
    // 0000 0010    : resume
    // 0000 0001    : pause
    unsigned short movingFlag;           // flag for moving status
    unsigned short speedFlag;           // flag for speed percentage
    float speedPercent;

private:
    float minDelayValue;          // max speed
};

#ifdef USING_TM1638QYF
extern TM1638QYF module;
extern unsigned int mode;
#endif

#endif //STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
