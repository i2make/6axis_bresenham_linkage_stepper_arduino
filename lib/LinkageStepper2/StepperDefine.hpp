
#ifndef STEPPER_MOTOR_STEPPERDEFINE_HPP
#define STEPPER_MOTOR_STEPPERDEFINE_HPP

#define USING_TM1638QYF

//#define SERIAL_OUTPUT

//#define PAUSE_RESUME

//#define SERIAL_INPUT

#define ENABLE_SPEED_CONTROL

#include <Arduino.h>

#ifdef USING_TM1638QYF
#include <TM1638.h>
#include <TM1638QYF.h>
#endif

#define MAX_AXIS            5u          // number of axis (1 ~ 6)

//      : MIN_DELAY : interval :      ms
// -------------------------------------
// 100% :      32.0 :        0 : 2705 ms
//  90% :      37.0 :        5 : 2735 ms
//  80% :      42.0 :        5 : 2712 ms
//  70% :      49.5 :        7 : 2713 ms
//  60% :      58.0 :        9 : 2703 ms
//  50% :      71.0 :       13 : 2708 ms
//  40% :      90.0 :       19 : 2701 ms
//  30% :     122.0 :       32 : 2700 ms
//  20% :     187.0 :       65 : 2713 ms
//  10% :     378.0 :      191 : 2703 ms
#define MIN_DELAY           32.0f       // minimum delay between steps (현 최대 속도 32)
#define DELAY_90            37.0f
#define DELAY_80            42.0f
#define DELAY_70            49.0f
#define DELAY_60            58.0f
#define DELAY_50            71.0f
#define DELAY_40            90.0f
#define DELAY_30           122.0f
#define DELAY_20           187.0f
#define DELAY_10           378.0f
#define DELAY_C0          1600.0f       // maximum delay between steps

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1u << (unsigned)OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1u << (unsigned)OCIE1A);

////////////////////////////////////////////////////////////////////////////////////
/// example using rotational moving
///
/// #define X_PPR               (long)2000 // pulse per rotation for stepper motor
/// #define Y_PPR               (long)2000 // pulse per rotation for stepper motor
/// #define X_GEAR_RATIO        (long)100 // A axis gear ratio
/// #define Y_GEAR_RATIO        (long)50 // C axis gear ration
///
/// #define X_SPD               float(A_PPR * A_GEAR_RATIO / 360) // steps per dgree
/// #define Y_SPD               float(C_PPR * C_GEAR_RATIO / 360) // steps per dgree
///
/////////////////////////////////////////////////////////////////////////////////////

#define X_DIR_PIN            2  // PD2(audiono digital pin 2)
#define X_STEP_PIN           3  // PD3(audiono digital pin 3)

#define Y_DIR_PIN            4  // PD4(audiono digital pin 4)
#define Y_STEP_PIN           5  // PD5(audiono digital pin 5)

#define Z_DIR_PIN            6  // PD6(audiono digital pin 6)
#define Z_STEP_PIN           7  // PD7(audiono digital pin 7)

#define A_DIR_PIN            8  // PB0(audiono digital pin 8)
#define A_STEP_PIN           9  // PB1(audiono digital pin 9)

#define B_DIR_PIN           10  // PB2(audiono digital pin 10)
#define B_STEP_PIN          11  // PB3(audiono digital pin 11)

#define C_DIR_PIN           12  // PB4(audiono digital pin 12)
#define C_STEP_PIN          13  // PB5(audiono digital pin 13)

#define PAUSE_BUTTON        19  // pause button
#define RESUME_BUTTON       18  // resume button

#define SPEED_CONTROL       17  // analog input

/////////////////////////////////////////////////////////////////////

#define X_PULSE_HIGH             PORTD |=  0b00001000; // PD3 pin HIGH
#define X_PULSE_LOW              PORTD &= ~0b00001000; // PD3 pin LOW

#define Y_PULSE_HIGH             PORTD |=  0b00100000; // PD5 pin HIGH
#define Y_PULSE_LOW              PORTD &= ~0b00100000; // PD5 pin LOW

#define Z_PULSE_HIGH             PORTD |=  0b10000000; // PD7 pin HIGH
#define Z_PULSE_LOW              PORTD &= ~0b10000000; // PD7 pin LOW

#define A_PULSE_HIGH             PORTB |=  0b00000010; // PB1 pin HIGH
#define A_PULSE_LOW              PORTB &= ~0b00000010; // PB1 pin LOW

#define B_PULSE_HIGH             PORTB |=  0b00001000; // PB3 pin HIGH
#define B_PULSE_LOW              PORTB &= ~0b00001000; // PB3 pin LOW

#define C_PULSE_HIGH             PORTB |=  0b00100000; // PB5 pin HIGH
#define C_PULSE_LOW              PORTB &= ~0b00100000; // PB5 pin LOW

#endif //STEPPER_MOTOR_STEPPERDEFINE_HPP

