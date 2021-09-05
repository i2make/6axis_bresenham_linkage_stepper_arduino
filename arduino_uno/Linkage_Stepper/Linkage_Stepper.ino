//
// Created by KJH on 2021-03-19.
//

#include "direction_pulse.hpp"
#include "StepperMotorWorld.hpp"

void display();

Motor X_Axis(xDirection, xPulse);
Motor Y_Axis(yDirection, yPulse);
Motor Z_Axis(zDirection, zPulse);
Motor A_Axis(aDirection, aPulse);
Motor B_Axis(bDirection, bPulse);
Motor C_Axis(cDirection, cPulse);

World world(&X_Axis);


/// display function
void display() {
    ;
}

/////////////////////////////////////////////////////////////////////////////
/// ISR
/// clk/64: 21 ~ 23 tick
/// clk/ 8: 200 tick
/// ISR에 진입할 때 TCNT는 0이 된다
////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect) {

    if (!world.movingDone()) {

        TCNT1 = DELAY_C0;           // for regular interval

        //OCR1A = world.setDelay();   // setting delay between steps
        OCR1A = world.setDelay2();   // setting delay between steps
        TCNT1 = 0;
        world.generatePulse();      // generate pulse
    }
}

//////////////////////////////////////////////////////////////////////////
void setup() {
    
    Serial.begin(9600);
    
    world.addMotor(&Y_Axis);
    world.addMotor(&Z_Axis);
    world.addMotor(&A_Axis);
    world.addMotor(&B_Axis);
    //world.addMotor(&C_Axis);

    // error checking
    if (world.motorIndex != MAX_AXIS) {
        Serial.print("Error");
        while (true);
    }

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

    TCCR1B |= (1 << WGM12);                                 // CTC mode OCR1A(TOP)
    TCCR1B |= ((0 << CS12) | (1 << CS11) | (1 << CS10));    // clk/64 prescaler
    sei();

}

void loop() {

    // moving
    world.setSpeed(20.0f);
    world.moving(16000,8000,4800,3200,1600,800, display);
    display();
    delay(1000);

    // moving
    world.setSpeed(20.0f);
    world.moving(32000,10000,9600,6400,3200,1600, display);
    display();

    while (true) {
        ;
    }
}
