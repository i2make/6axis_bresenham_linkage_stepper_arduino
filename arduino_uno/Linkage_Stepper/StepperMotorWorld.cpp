//
// Created by KJH on 2021-03-19.
//

#include "StepperMotorWorld.hpp"

///////////////////////////////////////////////////
/// constructor
///////////////////////////////////////////////////
World::World(Motor* _motor) {
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
World::bresenham(long _x2, long _y2, long _z2, long _a2, long _b2,
                 long _c2) {

    ////////////////////////////////////////////////////////
    /// initialaze
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

    setMaxLongDy(); // fine max dy-value motor 최대값 모터를 찾음

    // bresenham parameter "over" setting
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        motor[i]->over = motor[maxDyMotor]->absDy / 2;
    }

    setMotorDirection();    // set motor direction

    TIMER1_INTERRUPTS_ON

} // bresenham

void World::generatePulse() {

    // maxDyMotor의 dir 증감 (기준 모터)
    motor[maxDyMotor]->currentPosition += motor[maxDyMotor]->dirValue;

    // generate pulse for maxDyMotor
    motor[maxDyMotor]->pulse();

    // 모터 수 만큼 반복
    for (unsigned short w = 0; w < MAX_AXIS; w++) {

        if (w != maxDyMotor) {          // 최대 dy 모터는 스킵

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
/// testing setDelay2
///////////////////////////////////////////////////////////////////////
float World::setDelay2() {

    ///////////////////////////////////////////////////////////////////
    /// pause 일시 정지
    ///////////////////////////////////////////////////////////////////
    if (movingFlag & pause_msk) {           // pause flag on ?

        // pause 버튼을 누른 상태로 시작하면 이 조건문이 실행됨
        if ((movingFlag & acceleration_msk) == 0 &&     // 가속 아님
            (movingFlag & constantSpeed_msk) == 0 &&    // 등속 아님
            (movingFlag & deceleration_msk) == 0) {     // 감속 아님

            TIMER1_INTERRUPTS_OFF
        }

        // 가속 중 pause 버튼을 누르면 이 조건문이 실행됨
        if ((movingFlag & acceleration_msk) != 0 &&     // 가속
            (movingFlag & constantSpeed_msk) == 0 &&    // 등속 아님
            (movingFlag & deceleration_msk) == 0) {     // 감속 아님

            accelNstep = stepCounter;
            decelNstep = stepCounter;
        }

        // setting deceleration flag
        movingFlag &= ~acceleration_msk;        // 가속 아님
        movingFlag &= ~constantSpeed_msk;       // 등속 아님
        movingFlag |= deceleration_msk;         // 감속
    }
    ///////////////////////////////////////////////////////////////////
    /// resume 재개
    ///////////////////////////////////////////////////////////////////
    if (movingFlag & resume_msk) {

        elapsedCounter += stepCounter;
        stepCounter = 0;
        accelNstep = 0;
        delayValue = DELAY_C0;
        movingFlag &= ~resume_msk;

        // setting starting flag
        movingFlag &= ~acceleration_msk;    // 가속 아님
        movingFlag &= ~constantSpeed_msk;   // 등속 아님
        movingFlag &= ~deceleration_msk;    // 감속 아님

    }

    ///////////////////////////////////////////////////////////////////
    /// start acceleration 가속 시작
    ///////////////////////////////////////////////////////////////////
    if ((movingFlag & acceleration_msk) == 0 &&     // 가속 아님
        (movingFlag & constantSpeed_msk) == 0 &&    // 등속 아님
        (movingFlag & deceleration_msk) == 0) {     // 감속 아님

        digitalWrite(13, LOW);
        targetCounter++;
        stepCounter++;

        movingFlag |= acceleration_msk;     // acceleration flag on
        return DELAY_C0;                    // initial delay value
    }

    ///////////////////////////////////////////////////////////////////
    /// during acceleration 가속 중
    ///////////////////////////////////////////////////////////////////
    if ((movingFlag & acceleration_msk) != 0 &&     // 가속
        (movingFlag & constantSpeed_msk) == 0 &&    // 등속 아님
        (movingFlag & deceleration_msk) == 0) {     // 감속 아님

        calculateAccel(stepCounter);
        targetCounter++;
        stepCounter++;

        // exit condition: reached max speed 최대 속도에 도달 ///////////////
        if (delayValue < minDelayValue) {

            delayValue = minDelayValue;     // fix max speed
            accelNstep = stepCounter;
            decelNstep = stepCounter;
            pauseDecelNstep = stepCounter;
            movingFlag &= ~acceleration_msk;        // 가속 아님
            movingFlag |= constantSpeed_msk;        // 등속
            movingFlag &= ~deceleration_msk;        // 감속 아님
            return delayValue;
        }

        // TODO: test
        // exit condition: reached half step 최대 속도에 도달 전 감속 ////////
        if (stepCounter >
            (motor[maxDyMotor]->absDy - elapsedCounter) / 2) {

            accelNstep = stepCounter;
            decelNstep = stepCounter;
            pauseDecelNstep = stepCounter;
            movingFlag &= ~acceleration_msk;        // 가속 아님
            movingFlag &= ~constantSpeed_msk;       // 등속 아님
            movingFlag |= deceleration_msk;         // 감속
            return delayValue;
        }
        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// constant speed 등속
    ///////////////////////////////////////////////////////////////////
    if ((movingFlag & acceleration_msk) == 0 &&     // 가속 아님
        (movingFlag & constantSpeed_msk) != 0 &&    // 등속
        (movingFlag & deceleration_msk) == 0) {     // 감속 아님

        targetCounter++;
        stepCounter++;

        // TODO: if statement , accelNstep
        // stop condition: targetCounter의 감속
        if (targetCounter > motor[maxDyMotor]->absDy - accelNstep) {
            movingFlag &= ~acceleration_msk;    // 가속 아님
            movingFlag &= ~constantSpeed_msk;   // 등속 아님
            movingFlag |= deceleration_msk;     // 감속
        }
        return delayValue;
    }

    ///////////////////////////////////////////////////////////////////
    /// deceleration 감속
    ///////////////////////////////////////////////////////////////////
    if ((movingFlag & acceleration_msk) == 0 &&     // 가속 아님
        (movingFlag & constantSpeed_msk) == 0 &&    // 등속 아님
        (movingFlag & deceleration_msk) != 0) {     // 감속

        decelNstep--;
        targetCounter++;
        stepCounter++;

        // stop condition: end 끝에 도달 정지
        if (targetCounter == motor[maxDyMotor]->absDy) {
            movingFlag &= ~acceleration_msk;    // 가속 아님
            movingFlag &= ~constantSpeed_msk;   // 등속 아님
            movingFlag &= ~deceleration_msk;    // 감속 아님
            movingFlag |= movementDone_msk;     // movementDone flag on
            return delayValue;
        } // stop condition

        // stop condition: pause 일시 정지로 감속
        if (decelNstep == 0) {
            TIMER1_INTERRUPTS_OFF
            return delayValue;
        }

        calculateDecel(decelNstep);

        return delayValue;
    }

}

/////////////////////////////////////////////////////////////////////////////
/// 가속을 시작 : delayValue 계산
/////////////////////////////////////////////////////////////////////////////

/// start acceleration ?
/// during acceleration?
/// yes: acceleration   ///  no: deceleration?
/// yes: deceleration
///  no: constant speed moving
float World::setDelay() {

    // 최대 속도에 도달하거나
    // 움직일 거리가 짧아서 가속이 끝나기 전에 감속이 시작되면
    // acelNstep 값과 decelNstep 값을 설정한다




    ////////////////////////////////////////////////////////////////////////////
    /// start acceleration 가속 시작
    ////////////////////////////////////////////////////////////////////////////
    if (stepCounter == 0) {
        stepCounter++;
        return DELAY_C0;                    // initial delay value
    }


    ////////////////////////////////////////////////////////////////////////////
    /// During acceleration 가속 중
    ////////////////////////////////////////////////////////////////////////////
    if (accelNstep == 0) {



        // calculate delay value for n step
        // delayValue - (2 * delayValue) / (4 * (float) stepCounter + 1);
        calculateAccel(stepCounter);

        stepCounter++;

        // acceleration exit condition 최대 속도에 도달하면
        if (delayValue < minDelayValue) {
            delayValue = minDelayValue;     // fix max speed
            accelNstep = stepCounter;
            decelNstep = stepCounter;
            pauseDecelNstep = stepCounter;
        }

        // acceleration exit condition 가속이 끝나기 전에 감속을 시작하면
        if (stepCounter > motor[maxDyMotor]->absDy / 2) {
            accelNstep = stepCounter;
            decelNstep = stepCounter;
            pauseDecelNstep = stepCounter;
        }
        return delayValue;
    }

    else {
        ////////////////////////////////////////////////////////////////////////
        /// starting deceleration 감속
        ////////////////////////////////////////////////////////////////////////
        if (stepCounter > motor[maxDyMotor]->absDy - accelNstep) {
            decelNstep--;
            stepCounter++;

            // stop condition
            if (stepCounter == motor[maxDyMotor]->absDy) {
                movingFlag |= movementDone_msk;     // movementDone flag on
                return delayValue;
            } // stop condition

            // calculate delay value for n step
            calculateDecel(decelNstep);

            return delayValue;
        } //감속


        else {
            ////////////////////////////////////////////////////////////////////////
            /// constant speed 등속
            ////////////////////////////////////////////////////////////////////////
            stepCounter++;

//////////////////////////////////////////////////////////////////////////////
            // pause 일시 정지
            if (movingFlag & pause_msk) {           // pause flag on ?

                if (pauseDecelNstep == 0) {
                    TIMER1_INTERRUPTS_OFF
                    //movingFlag &= ~pause_msk;       // pause flag off
                    //movingFlag |= resumeReady_msk;  // resumeReady flag on
                    pauseDecelNstep = decelNstep;   // set deceleration n step
                }

                // calculate delay value for n step
                // (delayValue * float(4 * decelNstep + 1)) /
                //                             float(4 * decelNstep + 1 - 2);
                calculateDecel(decelNstep);

                pauseDecelNstep--;
            }

////////////////////////////////////////////////////////////////////////////////
            // resume 재개
            if (movingFlag & resume_msk) {

                // start acceleration 가속 시작
                if (resumeAccelNstep == 0) {
                    resumeAccelNstep++;
                    delayValue = DELAY_C0;
                    return DELAY_C0;
                }
                // delayValue - (2 * delayValue) / (4 * (float) resumeAccelNstep + 1);
                calculateAccel(resumeAccelNstep);

                // reached max speed 최대 속도에 도달
                if (delayValue < minDelayValue) {
                    delayValue = minDelayValue;     // min
                    movingFlag &= ~resume_msk;
                    //movingFlag &= ~resumeReady_msk;
                    resumeAccelNstep = 0;

                    return delayValue;
                }
                resumeAccelNstep++;
            }

            return delayValue;
        } //else 등속
    }
}


void World::moving(long _x, long _y, long _z, long _a, long _b, long _c,
                   void(func)()) {

    movingFlag &= ~movementDone_msk;        // start moving
    targetCounter = 0;                      // number of total step
    stepCounter = 0;                        // restart counter for resume
    elapsedCounter = 0;                     // summing elapsed step
    accelNstep = 0;
    delayValue = DELAY_C0;

    bresenham(_x, _y, _z, _a, _b, _c);

    while (!movingDone()) {
        func();
    }
    TIMER1_INTERRUPTS_OFF
}

bool World::addMotor(Motor* _motor) {
    motor[motorIndex] = _motor;
    motorIndex++;
    return motorIndex <= MAX_AXIS;
}

void World::pauseMoving() {
    movingFlag &= ~resume_msk;          // resume flag off
    movingFlag |= pause_msk;            // pause flag on
}

void World::resumeMoving() {
    // activate after pause
    //if (movingFlag & resumeReady_msk) {
    //digitalWrite(13, HIGH);
    if (movingFlag & pause_msk) {
        TIMER1_INTERRUPTS_ON
        resumeAccelNstep = 0;       // reset resume acceleration step counter
        movingFlag &= ~pause_msk;       // pause flag off
        movingFlag |= resume_msk;       // resume flag on
    }
}

bool World::movingDone() {
    return movingFlag & movementDone_msk;
}

void World::setSpeed(float speed) {
    if (speed > MIN_DELAY)
        minDelayValue = speed;
    else
        minDelayValue = MIN_DELAY;
}
