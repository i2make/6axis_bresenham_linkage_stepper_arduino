//
// Created by KJH on 2021-03-19.
//

#include <StepperMotorWorld.hpp>

///////////////////////////////////////////////////
/// constructor
///////////////////////////////////////////////////
World::World(Motor *_motor) {
    motor[0] = _motor;
    motorIndex = 1;
    minDelayValue = MIN_DELAY;
    movingFlag = 0;
}

///////////////////////////////////////////////////
/// find long moving motor
///////////////////////////////////////////////////
void World::setMaxLongDy() {
    // 모든 motor에서 maxLongdx를 찾아냄
    unsigned short tempMaxLongDy = 0;
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        if (motor[i]->absDy > tempMaxLongDy) {
            tempMaxLongDy = motor[i]->absDy;
            maxDyMotor = i;
        }
    }
}

void
World::bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2) {

    ////////////////////////////////////////////////////////
    /// initialize
    ////////////////////////////////////////////////////////
    switch (MAX_AXIS) {
        case 6:
            motor[5]->targetPosition = _c2;
        case 5:
            motor[4]->targetPosition = _b2;
        case 4:
            motor[3]->targetPosition = _a2;
        case 3:
            motor[2]->targetPosition = _z2;
        case 2:
            motor[1]->targetPosition = _y2;
        case 1:
            motor[0]->targetPosition = _x2;
    }

    // setting parameter
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        motor[i]->dy = motor[i]->targetPosition -
                       motor[i]->currentPosition;           // dy 계산
        motor[i]->dirValue = motor[i]->dy > 0 ? 1 : -1;     // dir 설정
        motor[i]->absDy = abs(motor[i]->dy);            // dy 절대값
    } // for

    setMaxLongDy(); // fine max dy-value motor (최대값 모터를 찾음)

    // bresenham parameter "over" setting
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        motor[i]->over = motor[maxDyMotor]->absDy / 2;
    }

    setMotorDirection();    // set motor direction

    TIMER1_INTERRUPTS_ON    // generating pulse

} // bresenham

void World::generatePulse() {

    // Increase or decrease the dir of maxDyMotor (maxDyMotor의 dir을 증감 (기준 모터))
    motor[maxDyMotor]->currentPosition += motor[maxDyMotor]->dirValue;

    // generate pulse for maxDyMotor
    motor[maxDyMotor]->pulse();

    // Repeat for the number of motors (모터 수 만큼 반복)
    for (unsigned short w = 0; w < MAX_AXIS; w++) {

        if (w != maxDyMotor) {          // skip max dy motor (최대 dy 모터는 스킵)

            if (motor[w]->dy != 0) {

                // over value update
                motor[w]->over += motor[w]->absDy;

                if (motor[w]->over >= motor[maxDyMotor]->absDy) {

                    motor[w]->over -= motor[maxDyMotor]->absDy;
                    motor[w]->currentPosition += motor[w]->dirValue;

                    // generate pulse for other motor's dy
                    motor[w]->pulse();
                } // if
            }
        } // if
    } // for(w
}

void World::setMotorDirection() {
    // set motor direction
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        motor[i]->direction(motor[i]->dirValue);
    }
}

///////////////////////////////////////////////////////////////////////
/// Setting delay between steps (스텝간의 delay를 설정)
///////////////////////////////////////////////////////////////////////
float World::setDelay2() {

    ///////////////////////////////////////////////////////////////////
    /// pause (일시 정지)
    ///////////////////////////////////////////////////////////////////
    if (BUTTON_PRESS_PAUSE) {           // pause flag on ?

        // executed when the pause button is pressed and started
        // pause 버튼을 누른 상태로 시작하면 이 조건문이 실행됨
        if (NOT_ACCELERATING &&         // not accelerating (가속 아님)
            NOT_CONSTANT_SPEED &&       // not constant speed (등속 아님)
            NOT_DECELERATING) {         // not decelerating (감속 아님)

            TIMER1_INTERRUPTS_OFF
        }

        // executed when the pause button is pressed during acceleration
        // 가속 중 pause 버튼을 누르면 이 조건문이 실행됨
        if (ACCELERATING &&             // accelerating (가속중)
            NOT_CONSTANT_SPEED &&       // not constant speed (등속 아님)
            NOT_DECELERATING) {         // not decelerating (감속 아님)

            accelNstep = stepCounter;   // 현재 진행된 스텝 수 -> 가속에 사용된 스텝 수
            decelNstep = stepCounter;   // 현재 진행된 스텝 수 -> 감속에 사용될 스텝 수
        }

        // setting deceleration flag
        CLEAR_ACCELERATION_FLAG;         // set not accelerating (가속 아님 설정)
        CLEAR_CONSTANT_SPEED_FLAG;       // set not constant speed (등속 아님 설정)
        SET_DECELERATION_FLAG;           // set decelerating (감속중 설정)
    }

    ///////////////////////////////////////////////////////////////////
    /// resume (재개)
    ///////////////////////////////////////////////////////////////////
    if (BUTTON_PRESS_RESUME) {

        elapsedCounter += stepCounter;
        stepCounter = 0;
        accelNstep = 0;
        delayValue = DELAY_C0;
        CLEAR_RESUME_FLAG;

        // setting starting flag
        CLEAR_ACCELERATION_FLAG;     // clear not acceleration (가속 아님 설정)
        CLEAR_CONSTANT_SPEED_FLAG;   // clear not constant speed (등속 아님 설정)
        CLEAR_DECELERATION_FLAG;     // clear not deceleration (감속 아님 설정)

    }

    ///////////////////////////////////////////////////////////////////
    /// stop -> acceleration (정지 -> 가속 시작)
    ///////////////////////////////////////////////////////////////////
    if (NOT_ACCELERATING &&     // not accelerating (가속 아님)
        NOT_CONSTANT_SPEED &&   // not constant speed (등속 아님)
        NOT_DECELERATING) {     // not decelerating (감속 아님)

        totalMovedCounter++;        //
        stepCounter++;          // add moved step

        SET_ACCELERATION_FLAG;       // acceleration flag on
        return DELAY_C0;        // initial delay value
    }

    ///////////////////////////////////////////////////////////////////
    /// during acceleration (가속중)
    ///////////////////////////////////////////////////////////////////
    if (ACCELERATING &&         // accelerating (가속중)
        NOT_CONSTANT_SPEED &&   // not constant speed (등속 아님)
        NOT_DECELERATING) {     // not decelerating (감속 아님)

        calculateAccel(stepCounter);    // Calculate the delay value at the current position
        totalMovedCounter++;        //
        stepCounter++;          // add moved step

        // exit condition: reached max speed (최대 속도에 도달) ///////////////
        if (delayValue < minDelayValue) {

            delayValue = minDelayValue;     // fix max speed
            accelNstep = stepCounter;       // Number of steps used for acceleration (가속에 사용된 스텝 수)
            decelNstep = stepCounter;       // Number of steps to use for deceleration (감속에 사용할 스텝 수)
            CLEAR_ACCELERATION_FLAG;        // 가속 아님
            SET_CONSTANT_SPEED_FLAG;        // 등속
            CLEAR_DECELERATION_FLAG;        // 감속 아님
            return delayValue;      // delay value between steps
        }

        // exit condition: Decelerate before reaching maximum speed (최대 속도에 도달 전 감속) ////////
        //      ---------------   <-- max speed (constant speed)
        //     /               \
        //    /                 \
        //   /\ <-- decelerating \
        //  /  \                  \
        // /    \                  \
        // ------+---------------------
        // ----->|
        //
        if (stepCounter >
            (motor[maxDyMotor]->absDy - elapsedCounter) / 2) {

            accelNstep = stepCounter;
            decelNstep = stepCounter;
            movingFlag &= ~acceleration_msk;        // 가속 아님
            movingFlag &= ~constantSpeed_msk;       // 등속 아님
            movingFlag |= deceleration_msk;         // 감속
            return delayValue;
        }
        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// constant speed (등속)
    ///////////////////////////////////////////////////////////////////
    if (NOT_ACCELERATING &&     // not accelerating (가속 아님)
        CONSTANT_SPEED &&       // constant speed (등속중)
        NOT_DECELERATING) {     // not decelerating (감속 아님)

        totalMovedCounter++;        //
        stepCounter++;          // add moved step

        // stop condition: targetCounter의 감속
        if (totalMovedCounter > motor[maxDyMotor]->absDy - accelNstep) {
            CLEAR_ACCELERATION_FLAG;    // 가속 아님
            CLEAR_CONSTANT_SPEED_FLAG;  // 등속 아님
            SET_DECELERATION_FLAG;      // 감속중
        }
        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// decelerating (감속중)
    ///////////////////////////////////////////////////////////////////
    if (NOT_ACCELERATING &&     // not acceleration (가속 아님)
        NOT_CONSTANT_SPEED &&   // not constant speed (등속 아님)
        DECELERATING) {         // deceleration (감속)

        decelNstep--;           // 감속용 스텝 수를 줄임
        totalMovedCounter++;        //
        stepCounter++;          // add moved step

        // stop condition: reached target position (원하는 위치에 도달하면 정지)
        if (totalMovedCounter == motor[maxDyMotor]->absDy) {
            CLEAR_ACCELERATION_FLAG;    // not acceleration (가속 아님)
            CLEAR_CONSTANT_SPEED_FLAG;  // not constant speed (등속 아님)
            CLEAR_DECELERATION_FLAG;    // not deceleration (감속 아님)
            SET_MOVING_DONE_FLAG;       // movementDone flag on
            return delayValue;
        } // stop condition

        // stop condition: pause (일시 정지로 감속)
        if (decelNstep == 0) {
            TIMER1_INTERRUPTS_OFF
            return delayValue;
        }

        calculateDecel(decelNstep);

        return delayValue;
    }
}

void World::moving(long _x, long _y, long _z, long _a, long _b, long _c, void(func)()) {

    movingFlag &= ~movementDone_msk;        // start moving
    totalMovedCounter = 0;                      // number of total step
    stepCounter = 0;                        // restart counter for resume
    elapsedCounter = 0;                     // summing elapsed step
    accelNstep = 0;
    delayValue = DELAY_C0;

    bresenham(_x, _y, _z, _a, _b, _c); // start moving

    while (!movingDone()) {
        func();
    }
    func();
    TIMER1_INTERRUPTS_OFF
}

bool World::addMotor(Motor *_motor) {
    motor[motorIndex] = _motor;
    motorIndex++;
    return motorIndex <= MAX_AXIS;
}

void World::pauseMoving() {
    movingFlag &= ~resume_msk;          // resume flag off
    movingFlag |= pause_msk;            // pause flag on
}

void World::resumeMoving() {
    if (movingFlag & pause_msk) {
        TIMER1_INTERRUPTS_ON
        movingFlag &= ~pause_msk;       // pause flag off
        movingFlag |= resume_msk;       // resume flag on
    }
}

void World::changeSpeed() {

}

float World::readSpeedController() {
    speedPercent = int(analogRead(SPEED_CONTROL) / 100.f) / 10.f;
    return speedPercent;
}

bool World::movingDone() const {
    return movingFlag & movementDone_msk;
}

void World::setSpeed(float speed) {
    if (speed > MIN_DELAY)
        minDelayValue = speed;
    else
        minDelayValue = MIN_DELAY;
}

float World::getSpeed() {
    return minDelayValue;
}
