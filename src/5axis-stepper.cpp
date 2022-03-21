
#include "direction_pulse.hpp"
#include "StepperMotorWorld.hpp"

#ifdef USING_TM1638QYF

void update(TM1638QYF *_module, word *_mode);

TM1638QYF module(14, 15, 16);
unsigned int mode;
unsigned int previousButtons;
#endif

//unsigned long testingElapsedTime = 65535;
//unsigned long previousElapsedTime;

/// World
World world;

/////////////////////////////////////////////////////////////////////////////
/// ISR
/// clk/64: 32 tick
///
/// TCNT becomes 0 when entering ISR (ISR에 진입할 때 TCNT는 0이 된다)
////////////////////////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect) {
    // TODO:
    TCNT1 = DELAY_C0;
    OCR1A = world.setDelay2(); // min 6 tick
    world.generatePulse(); // min 4 tick
    TCNT1 = 9;

//    OCR1A = world.setDelay2();
//    world.generatePulse();      // generate pulse
}

//////////////////////////////////////////////////////////////////////////

void setup() {
#ifdef SERIAL_OUTPUT
    Serial.begin(115200);
#endif

#ifdef USING_TM1638QYF
    /// TM1638QYF
    module.setupDisplay(true, 7);
    mode = 0;
#endif

    /// pinmode setting
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

    pinMode(PAUSE_BUTTON, INPUT_PULLUP);
    pinMode(RESUME_BUTTON, INPUT_PULLUP);

    /// timer 1 setting
    cli();
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1B |= (1 << WGM12);                   // CTC mode OCR1A(TOP)
    TCCR1B |= ((0 << CS12) | (1 << CS11) | (1 << CS10));    // clk/64 prescaler
    sei();
}

////////////////////////////////////////////////////////////////////////////

void loop() {
    world.moving(6400, 0, 0, 0, 0);
    delay(100);
    world.moving(6400, 6400, 0, 0, 0);
    delay(100);
    world.moving(6400, 6400, 6400, 0, 0);
    delay(100);
    world.moving(6400, 6400, 6400, 6400, 0);
    delay(100);
    world.moving(6400, 6400, 6400, 6400, 6400, 6400);
    delay(100);
    world.moving(0, 0, 0, 0, 0);
    delay(100);

    world.moving(3200, 6400, 9600, 16000, 32000);
    world.moving(0, 0, 0, 0, 0, 0);

    while (true) ;
}

///////////////////////////////////////////////////////////////////////////////

/// display function
void display() {

#ifdef SERIAL_OUTPUT
    String displayPosition = "*";
    int ascii = 88;
    for (unsigned int i = 0; i < MAX_AXIS; i++) {
        if (i > 2) { ascii = 62; }
        displayPosition.concat(" | ");
        displayPosition.concat(char(ascii + i));
        displayPosition.concat("=");
        displayPosition.concat(world.motor[i]->currentPosition);
    }
    Serial.println(displayPosition);
#endif

#ifdef PAUSE_RESUME
    world.inputIo->readPauseButton();
    world.inputIo->readResumeButton();
#endif

    /// Speed control
#ifdef ENABLE_SPEED_CONTROL
    world.inputIo->readSpeedController();
#endif

#ifdef USING_TM1638QYF
    update(&module, &mode);
#endif
}  //display

/// using TM1638QYF board
#ifdef USING_TM1638QYF

void update(TM1638QYF *_module, word *_mode) {
    word buttons = _module->getButtons();

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
            //_module->setDisplayToDecNumber(world.motor[0]->absDy, 0);
            //_module->setDisplayToString(String(testingElapsedTime));
            //_module->setDisplayToString(String(world.accelNstep));
            //_module->setDisplayToString(String(world.minDelayValue));
            //_module->setDisplayToString(String(world.previousSpeedPercent));
            //_module->setDisplayToString(String(world.previousMinDelayValue));
            //_module->setDisplayToString(String(world.delayValue));
            //_module->setDisplayToString(String(world.calculatedDelayValue[9]));
            //_module->setDisplayToString(String(world.speedPercent));
            _module->setDisplayToDecNumber(world.motor[4]->currentPosition, 0);
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
            break;
        case 64: //S8
            break;
        case 128: //S9
            break;
        case 256: //S10
            break;
        case 512: //S11
            break;
        case 1024: //S12
            break;
        case 2048: //S13
            // pause
            world.pauseMoving();
            *_mode = 0;
            break;
        case 4096: //S14
            // resume
            world.resumeMoving();
            *_mode = 0;
            break;
        case 8192: //S15
            break;
        case 16384: //S16
            break;
        default:;
    }
} //update
#endif
