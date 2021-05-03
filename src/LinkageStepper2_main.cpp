//
// Created by KJH on 2021-03-19.
//

#include "direction_pulse.cpp"
#include <StepperMotorWorld.hpp>

void display();

TM1638QYF module(14, 15, 16);
unsigned int mode;
unsigned int previousButtons;
unsigned long startTime;
unsigned long runningSecs;
unsigned int testCounter;
unsigned int ttt = 0;
long xx1 = 8888;
long xx2 = 8888;

Motor X_Axis(xDirection, xPulse);
Motor Y_Axis(yDirection, yPulse);
Motor Z_Axis(zDirection, zPulse);
Motor A_Axis(aDirection, aPulse);
Motor B_Axis(bDirection, bPulse);
Motor C_Axis(cDirection, cPulse);

World world(&X_Axis);

unsigned long elapTime() {
    unsigned long temp = (micros() - startTime);
    startTime = micros();
    return temp;
}

void update(TM1638QYF* _module, word* _mode) {
    word buttons = _module->getButtons();

    //runningSecs = elapTime();

    // button pressed - change mode
    if (buttons != 0 && buttons != previousButtons) {
        previousButtons = buttons;
        *_mode = buttons >> 1;
    }


    switch (*_mode) {
        case 0: //S1
            //_module->setDisplayToDecNumber(world.stepCounter, 0);
            //_module->setDisplayToDecNumber(OCR1A, 0);
            //_module->setDisplayToDecNumber(TCNT1, 0);
            //_module->setDisplayToDecNumber(ttt, 0);
            _module->setDisplayToDecNumber(world.motor[0]->currentPosition, 0);
            //_module->setDisplayToDecNumber(world.motor[0]->absDy, 0);
            break;
        case 1: //S2
            _module->setDisplayToDecNumber(world.motor[1]->currentPosition, 0);
            break;
        case 2: //S3
            _module->setDisplayToDecNumber(world.motor[2]->currentPosition, 0);
            break;
        case 4: //S4
            _module->setDisplayToDecNumber(world.motor[3]->currentPosition, 0);
            break;
        case 8: //S5
            _module->setDisplayToDecNumber(world.motor[4]->currentPosition, 0);
            break;
        case 16: //S6
            _module->setDisplayToDecNumber(world.motor[5]->currentPosition, 0);
            break;
        case 32: //S7
            _module->setDisplayToDecNumber(xx1, 0);
            break;
        case 64: //S8
            _module->setDisplayToDecNumber(xx2, 0);
            break;
        case 128: //S9
            _module->setDisplayToString("S09", 0);
            break;
        case 256: //S10
            _module->setDisplayToString("S10", 0);
            break;
        case 512: //S11
            _module->setDisplayToString("S11", 0);
            break;
        case 1024: //S12
            _module->setDisplayToString("S12", 0);
            break;
        case 2048: //S13
            // 등속 이동 중에 pause
            world.pauseMoving();
            *_mode = 0;
            break;

        case 4096: //S14
            // pause 해제
            world.resumeMoving();
            *_mode = 0;
            break;

        case 8192: //S15
            if (runningSecs % 2 == 0) {
                _module->setDisplayToString("TM1638QY", 1);
            }
            else {
                _module->setDisplayToString(String("LIBRARY "), 1);
            }
            break;
        case 16384: //S16
            char s[9];
            sprintf(s, "Secs %03d", runningSecs % 999);
            _module->setDisplayToString(s, 1 << 1);
            break;
        default:;
    }
} //update

/// display function
void display() {
    update(&module, &mode);
}

/////////////////////////////////////////////////////////////////////////////
/// ISR
/// clk/64: 21 ~ 23 tick
/// clk/ 8: 200 tick
/// ISR에 진입할 때 TCNT는 0이 된다
////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect) {

//    if (!world.movementDone) {
//        //runningSecs = elapTime();
//        OCR1A = world.setDelay();   // delay between steps
//        //TCNT1 = 0;
//        world.generatePulse();      // generate pulse
//    }

    //if (!world.movementDone) {
    if (!world.movingDone()) {

        //OCR1A = 65000;
        TCNT1 = DELAY_C0;           // for regular interval

        OCR1A = world.setDelay();   // delay between steps
        TCNT1 = 0;
        world.generatePulse();      // generate pulse
    }
}

//////////////////////////////////////////////////////////////////////////
void setup() {
    module.setupDisplay(true, 7);
    startTime = micros();
    mode = 0;

    world.addMotor(&Y_Axis);
    world.addMotor(&Z_Axis);
    world.addMotor(&A_Axis);
    world.addMotor(&B_Axis);
    world.addMotor(&C_Axis);

    // error checking
    if (world.motorIndex != MAX_AXIS) {
        module.setDisplayToString("Error", 0);
        while (true);
    }

    //Serial.begin(9600);

    // pinmode setting
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT);
    pinMode(Z_DIR_PIN, OUTPUT);
    pinMode(Z_STEP_PIN, OUTPUT);
    pinMode(A_DIR_PIN, OUTPUT);
    pinMode(A_STEP_PIN, OUTPUT);
    pinMode(B_DIR_PIN, OUTPUT);
    pinMode(B_STEP_PIN, OUTPUT);
    pinMode(C_DIR_PIN, OUTPUT);
    pinMode(C_STEP_PIN, OUTPUT);

    // timer 1 setting
    cli();
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1B |= (1 << WGM12);                   // CTC mode OCR1A(TOP)
    //TCCR1B |= ((1 << CS12) | (0 << CS11) | (1 << CS10));    // clk/1024 prescaler
    TCCR1B |= ((0 << CS12) | (1 << CS11) | (1 << CS10));    // clk/64 prescaler
    //TCCR1B |= ((0 << CS12) | (1 << CS11) | (0 << CS10));    // clk/8  prescaler
    sei();

}

void loop() {
    long position = 32000;

//    world.moving(position, 0,0,0,0,0, display);
//    display();
//    delay(1000);
//    world.moving(position, position,0,0,0,0, display);
//    display();
//    delay(1000);
//    world.moving(position, position, position,0,0,0, display);
//    display();
//    delay(1000);
//    world.moving(position, position, position, position,0,0, display);
//    display();
//    delay(1000);
//    world.moving(position, position, position, position, position,position, display);
//    display();
//    delay(1000);
//    world.moving(0,0,0,0,0,0, display);
//    display();
//    delay(1000);

    world.moving(16000,8000,4800,3200,1600,800, display);
    display();
    delay(1000);

    world.moving(32000,10000,9600,6400,3200,1600, display);
    display();
    delay(1000);
//
//    world.moving(0,0,0,0,0,0, display);
//    display();
//    delay(1000);
//
//    world.moving(3200,10000,9600,6400,3200,1600, display);
//    display();
//    delay(1000);
//
//    world.moving(3600,10000,9600,6400,3200,1600, display);
//    display();
//    delay(1000);

    while (true) {
        digitalWrite(13, HIGH);
        display();
        //module.setDisplayToDecNumber(TCNT1, 0);
        //module.setDisplayToDecNumber(ttt, 0);
    }
}