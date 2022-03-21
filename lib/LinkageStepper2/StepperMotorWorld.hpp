#ifndef STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
#define STEPPER_MOTOR_STEPPERMOTORWORLD_HPP

#include "StepperMotor.hpp"

// moving flag define
#define pause_msk                               (1u << 0u)
#define resume_msk                              (1u << 1u)
#define changeSpeed_msk                         (1u << 2u)
#define movementDone_msk                        (1u << 3u)
#define acceleration_msk                        (1u << 4u)
#define constantSpeed_msk                       (1u << 5u)
#define deceleration_msk                        (1u << 6u)
#define first_speed_control_sensor_read_msk     (1u << 7u)

#define ACCELERATING                (movingFlag & acceleration_msk)
#define CONSTANT_SPEED              (movingFlag & constantSpeed_msk)
#define DECELERATING                (movingFlag & deceleration_msk)
#define CHANGE_SPEED                (movingFlag & changeSpeed_msk)
#define MOVE_DONE_FLAG              (movingFlag & movementDone_msk)

#define NOT_ACCELERATING            !(movingFlag & acceleration_msk)
#define NOT_CONSTANT_SPEED          !(movingFlag & constantSpeed_msk)
#define NOT_DECELERATING            !(movingFlag & deceleration_msk)

// speed control
#define FIRST_SPEED_CONTROL_SENSOR_READING          movingFlag &   first_speed_control_sensor_read_msk
#define SET_FIRST_SPEED_CONTROL_SENSOR_READ_FLAG    movingFlag |=  first_speed_control_sensor_read_msk
#define CLEAR_FIRST_SPEED_CONTROL_SENSOR_READ_FLAG  movingFlag &= ~first_speed_control_sensor_read_msk

// flag
#define CLEAR_CHANGE_SPEED_FLAG     movingFlag &= ~changeSpeed_msk
#define SET_CHANGE_SPEED_FLAG       movingFlag |=  changeSpeed_msk

#define CLEAR_ACCELERATION_FLAG     movingFlag &= ~acceleration_msk
#define SET_ACCELERATION_FLAG       movingFlag |=  acceleration_msk

#define CLEAR_CONSTANT_SPEED_FLAG   movingFlag &= ~constantSpeed_msk
#define SET_CONSTANT_SPEED_FLAG     movingFlag |=  constantSpeed_msk

#define CLEAR_DECELERATION_FLAG     movingFlag &= ~deceleration_msk
#define SET_DECELERATION_FLAG       movingFlag |=  deceleration_msk

#define CLEAR_RESUME_FLAG           movingFlag &= ~resume_msk
#define SET_RESUME_FLAG             movingFlag |=  resume_msk

#define CLEAR_PAUSE_FLAG            movingFlag &= ~pause_msk
#define SET_PAUSE_FLAG              movingFlag |=  pause_msk

#define CLEAR_MOVING_DONE_FLAG      movingFlag &= ~movementDone_msk
#define SET_MOVING_DONE_FLAG        movingFlag |=  movementDone_msk

// pause & resume
#define BUTTON_PRESS_PAUSE          movingFlag & pause_msk
#define BUTTON_PRESS_RESUME         movingFlag & resume_msk

// moving status
#define STOP_STATUS                         NOT_ACCELERATING && NOT_CONSTANT_SPEED && NOT_DECELERATING
#define ACCELERATING_STATUS                     ACCELERATING && NOT_CONSTANT_SPEED && NOT_DECELERATING
#define CONSTANT_SPEED_STATUS               NOT_ACCELERATING &&     CONSTANT_SPEED && NOT_DECELERATING
#define DECELERATING_STATUS                 NOT_ACCELERATING && NOT_CONSTANT_SPEED &&     DECELERATING
#define CHANGE_SPEED_DECELERATING_STATUS    NOT_ACCELERATING &&     CONSTANT_SPEED &&     DECELERATING

// setting status
#define SET_STOP_STATUS                         CLEAR_ACCELERATION_FLAG; CLEAR_CONSTANT_SPEED_FLAG; CLEAR_DECELERATION_FLAG;
#define SET_ACCELERATING_STATUS                   SET_ACCELERATION_FLAG; CLEAR_CONSTANT_SPEED_FLAG; CLEAR_DECELERATION_FLAG;
#define SET_CONSTANT_SPEED_STATUS               CLEAR_ACCELERATION_FLAG;   SET_CONSTANT_SPEED_FLAG; CLEAR_DECELERATION_FLAG;
#define SET_DECELERATING_STATUS                 CLEAR_ACCELERATION_FLAG; CLEAR_CONSTANT_SPEED_FLAG;   SET_DECELERATION_FLAG;
#define SET_CHANGE_SPEED_DECELERATING_STATUS    CLEAR_ACCELERATION_FLAG;   SET_CONSTANT_SPEED_FLAG;   SET_DECELERATION_FLAG;

class InputIO {
public:
    unsigned short previousSpeedPercent;    // previous controlled speed percent

    void readSpeedController();

    void firstReadSpeedController();

    void readPauseButton();

    void readResumeButton();
};

struct World {
/// method ///////////////////////////////////////////////////////////
public:

    explicit World();          // constructor

    Motor *motor[MAX_AXIS];                 // number of motor

    InputIO *inputIo;                       // input (speed control)

    void generatePulse();                   // one pulse generate

    void setMotorDirection();               // set direction of motors

    float setDelay2();                      // setting delay for N step

    // moving overloaded method
    void moving(long _x, long _y, long _z, long _a, long _b, long _c, void(*)());

    void moving(long _x, long _y, long _z, long _a, long _b, long _c);

    void moving(long _x, long _y, long _z, long _a, long _b);

    void moving(long _x, long _y, long _z, long _a);

    void moving(long _x, long _y, long _z);

    void moving(long _x, long _y);

    void moving(long _x);


    void pauseMoving();                     // pause method

    void resumeMoving();                    // resume method

    void changeSpeed();

    bool movingDone() const;                      // moving done?

    void calculateAccel(unsigned long counter) {
        delayValue = delayValue - (2 * delayValue) / (4 * (float) counter + 1);
    }

    void calculateDecel(unsigned long counter) {
        delayValue = (delayValue * float(4 * counter + 1)) / float(4 * counter + 1 - 2);
    }

private:

    void setMaxLongDy();         // find maxDy in the motors

    void bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2);

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
//  |<-- elapsedCounter -->|-- stepCounter --->              |
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
private:
    unsigned short maxDyMotor;          // max long distance moving motor

public:
    unsigned long totalMovedCounter;    // Total number of steps moved (총 이동 스텝 수)
    unsigned long stepCounter;          // number of moving steps to calculate delayValue, accelNstep, decelNstep, elapsedCounter
    unsigned long elapsedCounter;       // summing elapsed step

    unsigned int accelNstep;            // Number of steps used for acceleration (가속에 사용된 스텝 수)
    unsigned int decelNstep;            // Number of steps to be used for deceleration (감속에 사용될 스텝 수)
    unsigned int previousDecelNstep;    // Speed changed decelNstep

    float delayValue;                   // current delay value (현재 스텝과 스텝 사이의 값)
    float minDelayValue;                // max speed
    float previousMinDelayValue;        // previous max speed

    // 1000 0000    : first_speed_control_sensor_read_msk
    // 0100 0000    : deceleration_msk
    // 0010 0000    : constantSpeed_msk
    // 0001 0000    : acceleration_msk
    // 0000 1000    : movementDone_msk
    // 0000 0100    : changeSpeed_msk
    // 0000 0010    : resume_msk
    // 0000 0001    : pause_msk
    unsigned short movingFlag;              // flag for moving status
};



#ifdef USING_TM1638QYF
extern TM1638QYF module;
extern unsigned int mode;
#endif

extern void display();
extern World world;

#endif //STEPPER_MOTOR_STEPPERMOTORWORLD_HPP
