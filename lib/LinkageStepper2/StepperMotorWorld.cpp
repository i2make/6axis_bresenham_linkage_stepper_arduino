//
// Created by KJH on 2021-03-19.
//

#include <StepperMotorWorld.hpp>

///////////////////////////////////////////////////
/// constructor
///////////////////////////////////////////////////
World::World(Motor* _motor) {
    motor[0] = _motor;
    motorIndex = 1;
    //resumeReady = false;
    movingFlag &= ~resumeReady_msk;
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

    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        motor[i]->dy = motor[i]->targetPosition - motor[i]->currentPosition;      // dy 계산
        motor[i]->dirValue = motor[i]->dy > 0 ? 1 : -1;     // dir 설정
        motor[i]->absDy = abs(motor[i]->dy);            // dy 절대값
    } // for

    //////////////////////////////////////////////////////////
    ///
    //////////////////////////////////////////////////////////
    setMaxLongDy(); // fine max dy value motor 최대값 모터를 찾음

    // bresenham parameter "over" setting
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        motor[i]->over = motor[maxDyMotor]->absDy / 2;
    }

    setMotorDirection();    // set motor direction

    TIMER1_INTERRUPTS_ON

} // bresenham

void World::generatePulse() {

//    if (stepCounter == motor[maxDyMotor]->absDy) {
//        // maxDyMotor의 dir 증감 (기준 모터)
//        motor[maxDyMotor]->currentPosition += motor[maxDyMotor]->dirValue;
//
//        // generate pulse for maxDyMotor
//        motor[maxDyMotor]->pulse();
//        return;
//    }

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



/////////////////////////////////////////////////////////////////////////////
/// 가속을 시작 : delayValue 계산
/////////////////////////////////////////////////////////////////////////////
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
    // During acceleration 가속 중
    ////////////////////////////////////////////////////////////////////////////
    if (accelNstep == 0) {

        // calculate delay value for n step
        delayValue =
                delayValue - (2 * delayValue) / (4 * (float) stepCounter + 1);
        stepCounter++;

        // 최대 속도에 도달하면
        if (delayValue < MIN_DELAY) {
            delayValue = MIN_DELAY;
            accelNstep = stepCounter;
            decelNstep = stepCounter;
            pauseDecelNstep = stepCounter;
        }
        // 가속이 끝나기 전에 감속을 시작하면
        if (stepCounter > motor[maxDyMotor]->absDy / 2) {
            accelNstep = stepCounter;
            decelNstep = stepCounter;
            pauseDecelNstep = stepCounter;
        }

        return delayValue;
    }

    else {
        ////////////////////////////////////////////////////////////////////////
        // starting deceleration 감속
        ////////////////////////////////////////////////////////////////////////
        if (stepCounter > motor[maxDyMotor]->absDy - accelNstep) {
            decelNstep--;
            stepCounter++;

            // stop condition
            if (stepCounter == motor[maxDyMotor]->absDy) {
                //movementDone = true;
                movingFlag |= movementDone_msk;
                return delayValue;
            }

            // calculate delay value for n step
            delayValue = (delayValue * float(4 * decelNstep + 1)) /
                         float(4 * decelNstep + 1 - 2);

            return delayValue;
        } //if


        else {
            ////////////////////////////////////////////////////////////////////////
            // 등속
            ////////////////////////////////////////////////////////////////////////
            stepCounter++;

            // pause 일시 정지
            // if (pause){
            if (movingFlag & pause_msk) {           // pause flag on ?
                if (pauseDecelNstep == 0) {
                    TIMER1_INTERRUPTS_OFF
                    //pause = false;
                    movingFlag &= ~pause_msk;       // pause flag off
                    //resumeReady = true;
                    movingFlag |= resumeReady_msk;
                    pauseDecelNstep = decelNstep;

                    //return delayValue;
                }

                // calculate delay value for n step
                delayValue = (delayValue * float(4 * decelNstep + 1)) /
                             float(4 * decelNstep + 1 - 2);

                pauseDecelNstep--;
            }
////////////////////////////////////////////////////////////////////////////////
            // resume 재개
            //if (resume) {
            if (movingFlag & resume_msk) {
                // start acceleration
                if (resumeAccelNstep == 0) {
                    resumeAccelNstep++;
                    delayValue = DELAY_C0;
                    return DELAY_C0;
                }

                delayValue = delayValue - (2 * delayValue) /
                                          (4 * (float) resumeAccelNstep + 1);

                // reached max speed
                if (delayValue < MIN_DELAY) {
                    delayValue = MIN_DELAY;
                    //resume = false;
                    movingFlag &= ~resume_msk;
                    //resumeReady = false;
                    movingFlag &= ~resumeReady_msk;
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
    //movementDone = false;
    movingFlag &= ~movementDone_msk;
    stepCounter = 0;
    accelNstep = 0;
    delayValue = DELAY_C0;

    bresenham(_x, _y, _z, _a, _b, _c);

    while (!movingDone()) {
        func();
    }
    TIMER1_INTERRUPTS_OFF
}

void World::moving(long _x, long _y, long _z, long _a, long _b, long _c) {
    //movementDone = false;
    movingFlag &= ~movementDone_msk;
    stepCounter = 0;
    accelNstep = 0;
    delayValue = DELAY_C0;

    bresenham(_x, _y, _z, _a, _b, _c);

    while (!movingDone());
    TIMER1_INTERRUPTS_OFF
}

bool World::addMotor(Motor* _motor) {
    motor[motorIndex] = _motor;
    motorIndex++;
    return motorIndex <= MAX_AXIS;
}

void World::pauseMoving() {
    //pause = true;                   // activate pause
    movingFlag |= pause_msk;
}

void World::resumeMoving() {
    //if (resumeReady) {              // activate after pause
    if (movingFlag & resumeReady_msk) {
        TIMER1_INTERRUPTS_ON
        resumeAccelNstep = 0;       // reset resume acceleration step counter
        //resume = true;              // activate resume
        movingFlag |= resume_msk;
    }
}

bool World::movingDone() {
    return movingFlag & movementDone_msk;
}
