//
// Created by KJH on 2021-03-19.
//

#include "direction_pulse.cpp"
#include <StepperMotorWorld.hpp>

Motor X_Axis(xDirection, xPulse);
Motor Y_Axis(yDirection, yPulse);
Motor Z_Axis(zDirection, zPulse);
Motor A_Axis(aDirection, aPulse);
Motor B_Axis(bDirection, bPulse);
Motor C_Axis(cDirection, cPulse);

World world(&X_Axis);

ISR(TIMER1_COMPA_vect) {

}

void setup() {
    world.addMotor(&Y_Axis);
    world.addMotor(&Z_Axis);
    world.addMotor(&A_Axis);
    world.addMotor(&B_Axis);
    //world.addMotor(&C_Axis);

    // error checking
    if (world.motorIndex != MAX_AXIS) {
        pinMode(13, OUTPUT);
        while (1) {
            digitalWrite(13, HIGH);
            delay(1000);
            digitalWrite(13, LOW);
            delay(1000);
        }
    }

    //Serial.begin(9600);
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

    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = 1000;                             // compare value
    TCCR1B |= (1 << WGM12);                   // CTC mode OCR1A(TOP)
    TCCR1B |= ((1 << CS11) | (1 << CS10));    // clk/64 prescaler
    sei();

}

void loop() {
    world.moving(1600, 0, 0, 0, 0, 0);
    world.moving(1600, 1600, 0, 0, 0, 0);
    world.moving(1600, 1600, 1600, 0, 0, 0);
    world.moving(1600, 1600, 1600, 1600, 0, 0);
    world.moving(1600, 1600, 1600, 1600, 1600, 0);

    delay(500);
    world.moving(0, 0, 0, 0, 0, 0);

    delay(500);
    world.moving(1600, 3200, 4800, 6400, 8000, 0);

    delay(500);
    world.moving(0, 0, 0, 0, 0, 0);

    delay(500);
    world.moving(8000, 6400, 4800, 3200, 1600, 0);

    delay(500);
    world.moving(0, 0, 0, 0, 0, 0);

    while (1);
}