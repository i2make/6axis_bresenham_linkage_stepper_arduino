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
}

unsigned short World::setMaxLongDy() {
    unsigned short maxDyMotor;
    // 모든 motor에서 maxLongdx를 찾아냄
    unsigned short tempMaxLongDy = 0;
    for (unsigned short i = 0; i < MAX_AXIS; i++) {
        if (motor[i]->absDy > tempMaxLongDy) {
            tempMaxLongDy = motor[i]->absDy;
            maxDyMotor = i;
        }
    }
    return maxDyMotor;
}

void World::bresenham(long _x2, long _y2, long _z2, long _a2, long _b2, long _c2) {

    ////////////////////////////////////////////////////////
    /// initialaze
    ////////////////////////////////////////////////////////
    switch (MAX_AXIS) {
        case 6:
            motor[5]->newDx = _c2;
        case 5:
            motor[4]->newDx = _b2;
        case 4:
            motor[3]->newDx = _a2;
        case 3:
            motor[2]->newDx = _z2;
        case 2:
            motor[1]->newDx = _y2;
        case 1:
            motor[0]->newDx = _x2;
        default:
            break;
    }

    for (int i = 0; i < MAX_AXIS; i++) {
        // dy 계산
        motor[i]->dy = motor[i]->newDx - motor[i]->dx;

        motor[i]->dirValue = motor[i]->dy > 0 ? 1 : -1;  // dir 설정
        motor[i]->absDy = abs(motor[i]->dy);        // dy 절대값
    } // for

    maxDyMotor = setMaxLongDy(); // dy 최대값 모터를 찾음

    long& maxDy = motor[maxDyMotor]->absDy; // 모터 중 최대 absDy

    // over setting
    for (int i = 0; i < MAX_AXIS; i++) {
        motor[i]->over = maxDy / 2;
    }

    setMotorDirection();    // set motor direction

    //////////////////////////////////////////////////////////////
    /// 최대 dy 모터를 기준으로 반복
    //////////////////////////////////////////////////////////////

    // maxDyMotor(긴축)만큼 펄스 생성
    for (int i = 0; i < maxDy; i++) {

        setDelay(); // set for maxDyMotor's delayN counter "delayValue"
        //Serial.print("delayValue= ");
        //Serial.println(motor[maxDyMotor]->delayValue);
        delayMicroseconds(motor[maxDyMotor]->delayValue);

        generatePulse();

    } // for(i
} // bresenham

void World::generatePulse() {

    // maxDyMotor의 dir 증감 (기준 모터)
    motor[maxDyMotor]->dx += motor[maxDyMotor]->dirValue;

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
                    motor[w]->dx += motor[w]->dirValue;

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

void World::setDelay() {

    // create reference
    float& d = motor[maxDyMotor]->delayValue;
    long& n = motor[maxDyMotor]->delayN;

    ///////////////////////////////////////////
    /// 가속을 시작 : delayValue 계산
    ///////////////////////////////////////////

    /// 가속중 이면

    // 최대 가속도에 도달하거나
    // 거리가 짧아서 가속이 끝나기 전에 감속이 시작되면
    // sccelNstep 값이 set되고 이 루프 안으로 진입하지 않고
    // else 부분만 실행된다
    if (accelNstep == 0 && n > 0) {
        d = d - (2 * d) / (4 * n + 1);  // calculate delay value for n step

//        if (n % 100 == 0) {
//            Serial.println(d);
//        }

        // 가속이 끝나기 전에 감속이 시작
        if (n > motor[maxDyMotor]->absDy / 2) {
            accelNstep = n;
            decelNstep = n;
//            Serial.print("absDy / 2 = ");
//            Serial.println(d);
        }

        // 최대 가속도까지 증가하면
        if (d < MIN_DELAY) {
//            Serial.print("MIN_DELAY = ");
//            Serial.println(d);
            d = MIN_DELAY;
            accelNstep = n;
            decelNstep = n;
        }
    }
    else {
        // 감속을 시작
        if (n >= motor[maxDyMotor]->absDy - accelNstep) {
            decelNstep--;
            d = (d * (4 * decelNstep + 1)) / (4 * decelNstep + 1 - 2);
            //Serial.println(d);
        }
    }
    //Serial.println(d);
    n++;
}

void World::moving(long _x, long _y, long _z, long _a, long _b, long _c) {
    motor[maxDyMotor]->delayN = 0;
    accelNstep = 0;
    motor[maxDyMotor]->delayValue = motor[maxDyMotor]->delayC0;

    bresenham(_x, _y, _z, _a, _b, _c);
}

bool World::addMotor(Motor* _motor) {
    motor[motorIndex] = _motor;
    motorIndex++;
    return motorIndex <= MAX_AXIS;
}
