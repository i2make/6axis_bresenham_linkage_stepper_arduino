
#include <StepperMotorWorld.hpp>

///////////////////////////////////////////////////
/// constructor
///////////////////////////////////////////////////
World::World() {
    minDelayValue = MIN_DELAY;
    movingFlag = 0;

    // create instance
    switch (MAX_AXIS) {
        case 1:
            motor[0] = new Motor(xDirection, xPulse);
            break;
        case 2:
            motor[0] = new Motor(xDirection, xPulse);
            motor[1] = new Motor(yDirection, yPulse);
            break;
        case 3:
            motor[0] = new Motor(xDirection, xPulse);
            motor[1] = new Motor(yDirection, yPulse);
            motor[2] = new Motor(zDirection, zPulse);
            break;
        case 4:
            motor[0] = new Motor(xDirection, xPulse);
            motor[1] = new Motor(yDirection, yPulse);
            motor[2] = new Motor(zDirection, zPulse);
            motor[3] = new Motor(aDirection, aPulse);
            break;
        case 5:
            motor[0] = new Motor(xDirection, xPulse);
            motor[1] = new Motor(yDirection, yPulse);
            motor[2] = new Motor(zDirection, zPulse);
            motor[3] = new Motor(aDirection, aPulse);
            motor[4] = new Motor(bDirection, bPulse);
            break;
        case 6:
            motor[0] = new Motor(xDirection, xPulse);
            motor[1] = new Motor(yDirection, yPulse);
            motor[2] = new Motor(zDirection, zPulse);
            motor[3] = new Motor(aDirection, aPulse);
            motor[4] = new Motor(bDirection, bPulse);
            motor[5] = new Motor(cDirection, cPulse);
    }

    inputIo = new InputIO();
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

void World::bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2) {

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
        motor[i]->dy = motor[i]->targetPosition - motor[i]->currentPosition;           // dy 계산
        motor[i]->dirValue = motor[i]->dy > 0 ? 1 : -1;     // dir 설정
        motor[i]->absDy = abs(motor[i]->dy);            // dy 절대값
    } // for

    setMaxLongDy(); // find max dy-value motor (최대 dy 값 모터를 찾음)

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

                // TODO: >= or > 수정 확인
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
        if ((STOP_STATUS) && (FIRST_SPEED_CONTROL_SENSOR_READING)) {

            CLEAR_FIRST_SPEED_CONTROL_SENSOR_READ_FLAG;
            TIMER1_INTERRUPTS_OFF
        }

        // executed when the pause button is pressed during acceleration
        // 가속 중 pause 버튼을 누르면 이 조건문이 실행됨
        if (ACCELERATING_STATUS) {

            accelNstep = stepCounter;   // 현재 진행된 스텝 수 -> 가속에 사용된 스텝 수
            decelNstep = stepCounter;   // 현재 진행된 스텝 수 -> 감속에 사용될 스텝 수
            previousDecelNstep = stepCounter;
        }

        // when constant speed, decelerating
        // setting deceleration status flag
        SET_DECELERATING_STATUS     // Change from accelerating status to decelerating status.
    }

    ///////////////////////////////////////////////////////////////////
    /// resume (재개)
    ///////////////////////////////////////////////////////////////////
    if (BUTTON_PRESS_RESUME) {
        elapsedCounter += stepCounter;  // summing stepCounter
        stepCounter = 0;                // restart stepCounter
        accelNstep = 0;
        delayValue = DELAY_C0;
        CLEAR_RESUME_FLAG;

        // setting stop status flag
        SET_STOP_STATUS
    }

    ///////////////////////////////////////////////////////////////////
    /// change speed (속도 변경)
    ///////////////////////////////////////////////////////////////////
    if (CHANGE_SPEED) {
        if (CONSTANT_SPEED) { // 등속일 때만 속도 변경

            //
            //                -------------> accelerating
            //               /
            //      ________/
            //     /       |\
            //    /        | \
            //   /         |  -------------> decelerating
            //  /          |
            // ------------+----------------------------------
            //             ^
            //         current accelNstep

            if (previousMinDelayValue >= minDelayValue) {
                SET_ACCELERATING_STATUS     // increase velocity (속도 증가)
            } else {
                previousDecelNstep = accelNstep;
                SET_CHANGE_SPEED_DECELERATING_STATUS // decrease velocity (속도 감소)
            }
            stepCounter = accelNstep; // 이전 accelNstep부터 다시 계산하기 위해 stepCounter 설정
            CLEAR_CHANGE_SPEED_FLAG;    // end of speed control
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////
    /// stop -> acceleration (정지 -> 가속 시작)
    ///////////////////////////////////////////////////////////////////
    if (STOP_STATUS) {

        totalMovedCounter++;        // total moved step counter
        stepCounter++;              // add moved step

        SET_ACCELERATION_FLAG;      // acceleration flag on
        return DELAY_C0;            // initial delay value
    }

    ///////////////////////////////////////////////////////////////////
    /// during acceleration (가속중)
    ///////////////////////////////////////////////////////////////////
    if (ACCELERATING_STATUS) {

        calculateAccel(stepCounter);    // Calculate the delay value at the current position

        totalMovedCounter++;        // add total moved step
        stepCounter++;              // add stepCounter for calculate accelNstep, decelNstep, elapsedCounter

        // exit condition: reached max speed (최대 속도에 도달) ///////////////
        if (delayValue < minDelayValue) {

            delayValue = minDelayValue;     // fix max speed
            accelNstep = stepCounter;       // Number of steps used for acceleration (가속에 사용된 스텝 수)
            decelNstep = stepCounter;       // Number of steps to use for deceleration (감속에 사용할 스텝 수)
            previousDecelNstep = stepCounter;

            SET_CONSTANT_SPEED_STATUS       // Change from accelerating status to constant speed status.

            return delayValue;      // delay value between steps
        }

        // exit condition: Decelerate before reaching maximum speed (최대 속도에 도달 전 감속) ///////
        //      ---------------   <-- max speed (constant speed)
        //     /               \
        //    /                 \
        //   /\ <-- decelerating \
        //  /  \                  \
        // /    \                  \
        // ------+---------------------
        // ----->|
        //
        // (motor[maxDyMotor]->absDy - elapsedCounter) is remaining step
        if (stepCounter >
            (motor[maxDyMotor]->absDy - elapsedCounter) / 2) {

            accelNstep = stepCounter;
            decelNstep = stepCounter;
            previousDecelNstep = stepCounter;

            SET_DECELERATING_STATUS     // Change from constant speed status to decelerating status.

            return delayValue;
        }
        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// constant speed (등속)
    ///////////////////////////////////////////////////////////////////
    if (CONSTANT_SPEED_STATUS) {

        totalMovedCounter++;    // add total moved step counter
        previousMinDelayValue = minDelayValue;

        // stop condition: starting deceleration (감속 시작)
        if (totalMovedCounter > motor[maxDyMotor]->absDy - accelNstep) {
            SET_DECELERATING_STATUS     // Change from constant speed status to decelerating status.
        }
        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// decelerating (감속중)
    ///////////////////////////////////////////////////////////////////
    if (DECELERATING_STATUS) {

        //calculateDecel(decelNstep);

        decelNstep--;           // decelerating counter (감속용 스텝 수를 줄임)
        totalMovedCounter++;    // add counter

        // stop condition: reached target position (원하는 위치에 도달하면 정지)
        if (totalMovedCounter == motor[maxDyMotor]->absDy) {
            SET_STOP_STATUS             // set stop status
            SET_MOVING_DONE_FLAG;       // movementDone flag on
            return 0;
        } // stop condition

        // stop condition: pause button pressed (일시 정지 버튼을 누를 때 감속)
        if (decelNstep == 0) {
            TIMER1_INTERRUPTS_OFF   // stop moving
            return 0;
        }

        calculateDecel(decelNstep);

        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// change speed decelerating (속도 조절 감속중)
    ///////////////////////////////////////////////////////////////////
    if (CHANGE_SPEED_DECELERATING_STATUS) {
        // TODO: 속도 조절후에 delayValue가 조금씩 변동하는 것 수정
        calculateDecel(decelNstep);

        decelNstep--;           // decelerating counter (감속용 스텝 수를 줄임)
        totalMovedCounter++;    // add total moved counter

        if (delayValue > minDelayValue) {
            accelNstep = (accelNstep - (previousDecelNstep - decelNstep)) + 1; // Number of steps used for acceleration (가속에 사용된 스텝 수)
            decelNstep = accelNstep;       // Number of steps to use for deceleration (정지 감속에 사용할 스텝 수를 update)
            delayValue = minDelayValue;
            SET_CONSTANT_SPEED_STATUS       // change constant speed status (등속으로 전환)
        }

        return delayValue;
    }

    return 0; // never reached
} // setDelay2

void World::moving(long _x, long _y, long _z, long _a, long _b, long _c, void(func)()) {

#ifdef ENABLE_SPEED_CONTROL
    inputIo->firstReadSpeedController();
#endif

    SET_STOP_STATUS
    CLEAR_CHANGE_SPEED_FLAG;                    // clear change speed flag
    CLEAR_MOVING_DONE_FLAG;                     // start moving
    totalMovedCounter = 0;                      // number of total step
    stepCounter = 0;                            // restart counter for resume
    elapsedCounter = 0;                         // summing elapsed step
    accelNstep = 0;
    delayValue = DELAY_C0;

    bresenham(_x, _y, _z, _a, _b, _c); // start moving

    while (!movingDone()) {
        func();
    }
    func();
    TIMER1_INTERRUPTS_OFF
}

void World::moving(long _x, long _y, long _z, long _a, long _b, long _c) {
    moving(_x, _y, _z, _a, _b, _c, display);
}

void World::moving(long _x, long _y, long _z, long _a, long _b) {
    moving(_x, _y, _z, _a, _b, 0, display);
}

void World::moving(long _x, long _y, long _z, long _a) {
    moving(_x, _y, _z, _a, 0, 0, display);
}

void World::moving(long _x, long _y, long _z) {
    moving(_x, _y, _z, 0, 0, 0, display);
}

void World::moving(long _x, long _y) {
    moving(_x, _y, 0, 0, 0, 0, display);
}

void World::moving(long _x) {
    moving(_x, 0, 0, 0, 0, 0, display);
}

void World::pauseMoving() {
    CLEAR_RESUME_FLAG;          // resume flag off
    SET_PAUSE_FLAG;             // pause flag on
}

void World::resumeMoving() {
    if (BUTTON_PRESS_PAUSE) {
        TIMER1_INTERRUPTS_ON
        CLEAR_PAUSE_FLAG;       // pause flag off
        SET_RESUME_FLAG;        // resume flag on
    }
}

void World::changeSpeed() {
    TIMER1_INTERRUPTS_ON
    SET_CHANGE_SPEED_FLAG;
}

bool World::movingDone() const {
    return MOVE_DONE_FLAG;
}

////////////////////////////////////////////////////////////////////////
/// InputIO
////////////////////////////////////////////////////////////////////////

void InputIO::readSpeedController() {
    int sensorRead = int(analogRead(SPEED_CONTROL) / 10.f);
    switch (sensorRead) {
        case 99 ... 102:
            if (previousSpeedPercent != 100) {
                previousSpeedPercent = 100;
                world.minDelayValue = MIN_DELAY;
                world.changeSpeed();
            }
            break;
        case 89 ... 91:
            if (previousSpeedPercent != 90) {
                previousSpeedPercent = 90;
                world.minDelayValue = DELAY_90;
                world.changeSpeed();
            }
            break;
        case 79 ... 81:
            if (previousSpeedPercent != 80) {
                previousSpeedPercent = 80;
                world.minDelayValue = DELAY_80;
                world.changeSpeed();
            }
            break;
        case 69 ... 71:
            if (previousSpeedPercent != 70) {
                previousSpeedPercent = 70;
                world.minDelayValue = DELAY_70;
                world.changeSpeed();
            }
            break;
        case 59 ... 61:
            if (previousSpeedPercent != 60) {
                previousSpeedPercent = 60;
                world.minDelayValue = DELAY_60;
                world.changeSpeed();
            }
            break;
        case 49 ... 51:
            if (previousSpeedPercent != 50) {
                previousSpeedPercent = 50;
                world.minDelayValue = DELAY_50;
                world.changeSpeed();
            }
            break;
        case 39 ... 41:
            if (previousSpeedPercent != 40) {
                previousSpeedPercent = 40;
                world.minDelayValue = DELAY_40;
                world.changeSpeed();
            }
            break;
        case 29 ... 31:
            if (previousSpeedPercent != 30) {
                previousSpeedPercent = 30;
                world.minDelayValue = DELAY_30;
                world.changeSpeed();
            }
            break;
        case 19 ... 21:
            if (previousSpeedPercent != 20) {
                previousSpeedPercent = 20;
                world.minDelayValue = DELAY_20;
                world.changeSpeed();
            }
            break;
        case 9 ... 11:
            if (previousSpeedPercent != 10) {
                if (previousSpeedPercent == 0) {
                    previousSpeedPercent = 10;
                    world.minDelayValue = DELAY_10;
                    world.resumeMoving();
                } else {
                    previousSpeedPercent = 10;
                    world.minDelayValue = DELAY_10;
                    world.changeSpeed();
                }
            }
            break;
        case 0 ... 2:
            previousSpeedPercent = 0;
            world.minDelayValue = DELAY_C0;
            world.pauseMoving();
            break;
    }
}

void InputIO::firstReadSpeedController() {
    int sensorRead = int(analogRead(SPEED_CONTROL) / 10.f);
    switch (sensorRead) {
        case 96 ... 103:
            previousSpeedPercent = 100;
            world.minDelayValue = MIN_DELAY;
            break;
        case 86 ... 95:
            previousSpeedPercent = 90;
            world.minDelayValue = DELAY_90;
            break;
        case 76 ... 85:
            previousSpeedPercent = 80;
            world.minDelayValue = DELAY_80;
            break;
        case 66 ... 75:
            previousSpeedPercent = 70;
            world.minDelayValue = DELAY_70;
            break;
        case 56 ... 65:
            previousSpeedPercent = 60;
            world.minDelayValue = DELAY_60;
            break;
        case 46 ... 55:
            previousSpeedPercent = 50;
            world.minDelayValue = DELAY_50;
            break;
        case 36 ... 45:
            previousSpeedPercent = 40;
            world.minDelayValue = DELAY_40;
            break;
        case 26 ... 35:
            previousSpeedPercent = 30;
            world.minDelayValue = DELAY_30;
            break;
        case 16 ... 25:
            previousSpeedPercent = 20;
            world.minDelayValue = DELAY_20;
            break;
        case 6 ... 15:
            previousSpeedPercent = 10;
            world.minDelayValue = DELAY_10;
            break;
        case 0 ... 5:
            previousSpeedPercent = 0;
            world.minDelayValue = DELAY_C0;
            break;
    }
    world.SET_FIRST_SPEED_CONTROL_SENSOR_READ_FLAG;
}

void InputIO::readPauseButton() {
    if (!digitalRead(PAUSE_BUTTON)) {
        world.pauseMoving();
#ifdef SERIAL_OUTPUT
        Serial.println("PAUSE...");
#endif
    }
}

void InputIO::readResumeButton() {
    if (!digitalRead(RESUME_BUTTON)) {
        world.resumeMoving();
#ifdef SERIAL_OUTPUT
        Serial.println("RESUME...");
#endif
    }
}
