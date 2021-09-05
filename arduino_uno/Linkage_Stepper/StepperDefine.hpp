
#ifndef STEPPER_MOTOR_STEPPERDEFINE_HPP
#define STEPPER_MOTOR_STEPPERDEFINE_HPP

#include <Arduino.h>

#define MAX_AXIS            5u      // number of axis

// setting 1: 8000, 200.0f
#define DELAY_C0    600             // maximum delay between step and step
#define MIN_DELAY   20.0f           // minimum delay between step and step

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1u << (unsigned)OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1u << (unsigned)OCIE1A);

/////////////////////////
// calculation example using rorational axis
/////////////////////////

// #define X_PPR               (long)2000 // pulse per rotation for stepper motor
// #define Y_PPR               (long)2000 // pulse per rotation for stepper motor
// #define X_GEAR_RATIO        (long)100 // A axis gear ratio
// #define Y_GEAR_RATIO        (long)50 // C axis gear ration

// #define X_SPD               float(A_PPR * A_GEAR_RATIO / 360) // steps per dgree
// #define Y_SPD               float(C_PPR * C_GEAR_RATIO / 360) // steps per dgree

/////////////////////////////////

#define X_DIR_PIN          2   // PD2(arduino digital pin 2)
#define X_STEP_PIN         3   // PD3(arduino digital pin 3)

#define Y_DIR_PIN          4   // PD4(arduino digital pin 4)
#define Y_STEP_PIN         5   // PD5(arduino digital pin 5)

#define Z_DIR_PIN          6   // PD6(arduino digital pin 6)
#define Z_STEP_PIN         7   // PD7(arduino digital pin 7)

#define A_DIR_PIN          8   // PB0(arduino digital pin 8)
#define A_STEP_PIN         9   // PB1(arduino digital pin 9)

#define B_DIR_PIN          10   // PB2(arduino digital pin 10)
#define B_STEP_PIN         11   // PB3(arduino digital pin 11)

#define C_DIR_PIN          12   // PB4(arduino digital pin 12)
#define C_STEP_PIN         13   // PB5(arduino digital pin 13)

/////////////////////////////////////////////////////////////////////

#define X_PULSE_HIGH             PORTD |=  0b00001000; // PD3 pin HIGH (arduino digital pin 3)
#define X_PULSE_LOW              PORTD &= ~0b00001000; // PD3 pin LOW  (arduino digital pin 3)

#define Y_PULSE_HIGH             PORTD |=  0b00100000; // PD5 pin HIGH (arduino digital pin 5)
#define Y_PULSE_LOW              PORTD &= ~0b00100000; // PD5 pin LOW  (arduino digital pin 5)

#define Z_PULSE_HIGH             PORTD |=  0b10000000; // PD7 pin HIGH (arduino digital pin 7)
#define Z_PULSE_LOW              PORTD &= ~0b10000000; // PD7 pin LOW  (arduino digital pin 7)

#define A_PULSE_HIGH             PORTB |=  0b00000010; // PB1 pin HIGH (arduino digital pin 9)
#define A_PULSE_LOW              PORTB &= ~0b00000010; // PB1 pin LOW  (arduino digital pin 9)

#define B_PULSE_HIGH             PORTB |=  0b00001000; // PB3 pin HIGH (arduino digital pin 11)
#define B_PULSE_LOW              PORTB &= ~0b00001000; // PB3 pin LOW  (arduino digital pin 11)

#define C_PULSE_HIGH             PORTB |=  0b00100000; // PB5 pin HIGH (arduino digital pin 13)
#define C_PULSE_LOW              PORTB &= ~0b00100000; // PB5 pin LOW  (arduino digital pin 13)

#endif //STEPPER_MOTOR_STEPPERDEFINE_HPP