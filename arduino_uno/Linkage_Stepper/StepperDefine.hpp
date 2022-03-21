
#ifndef STEPPER_MOTOR_STEPPERDEFINE_HPP
#define STEPPER_MOTOR_STEPPERDEFINE_HPP

/////////////////////////////////////////////////////////////////////////////
/// configuration
/////////////////////////////////////////////////////////////////////////////

/// number of axes to be used
#define MAX_AXIS            5u          // number of axis (1u ~ 6u)

/// stepper max speed
//#define MAX_20_SPEED
#define MAX_23_SPEED
//#define MAX_32_SPEED

/// using TM1638QYF board
//#define USING_TM1638QYF         // Enable TM1638QYF board

/// serial print
#define SERIAL_OUTPUT           // Enable Serial Print status message

// TODO: pause resume과 speed control을 동시에 사용할 때 충돌 수정

/// pause & resume button
//#define PAUSE_RESUME          // Enable Pause & Resume button

/// potentiometer speed control
#define ENABLE_SPEED_CONTROL    // Enable speed control using potentiometer

//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>

#ifdef USING_TM1638QYF
#include <TM1638.h>
#include <TM1638QYF.h>
#endif

#ifdef ENABLE_SPEED_CONTROL
//////////////////////////////////////////////////////////////////////////////////
#ifdef MAX_20_SPEED
                                    //          : MIN_DELAY : interval :      ms : multi
                                    // ---------------------------------------------------
#define MIN_DELAY            20.f   // 10 turns :      20.0 :        0 : 1673 ms :  1.00
#define DELAY_90             22.f   //  9 turns :      22.0 :        2 : 1642 ms :  1.10
#define DELAY_80             25.f   //  8 turns :      25.0 :        3 : 1636 ms :  1.25
#define DELAY_70             29.f   //  7 turns :      29.0 :        4 : 1639 ms :  1.45
#define DELAY_60             34.f   //  6 turns :      34.0 :        5 : 1629 ms :  1.70
#define DELAY_50             42.f   //  5 turns :      42.0 :        8 : 1652 ms :  2.10
#define DELAY_40             52.f   //  4 turns :      52.0 :       10 : 1630 ms :  2.60
#define DELAY_30             70.f   //  3 turns :      70.0 :       18 : 1633 ms :  3.50
#define DELAY_20            105.f   //  2 turns :     105.0 :       35 : 1622 ms :  5.25
#define DELAY_10            213.f   //  1 turn  :     213.0 :      108 : 1633 ms : 10.65
#define DELAY_C0            350.f   // C0       :     350.0 :      137 :         : 15.00
#endif
///////////////////////////////////////////////////////////////////////////////////
#ifdef MAX_23_SPEED
                                    //          : MIN_DELAY : interval :      ms : multi
                                    // ---------------------------------------------------
#define MIN_DELAY            23.f   // 10 turns :      23.0 :        0 : 1825 ms :  1.00
#define DELAY_90             26.f   //  9 turns :      26.0 :        3 : 1838 ms :  1.10
#define DELAY_80             30.f   //  8 turns :      30.0 :        4 : 1860 ms :  1.25
#define DELAY_70             34.f   //  7 turns :      34.0 :        4 : 1839 ms :  1.45
#define DELAY_60             40.f   //  6 turns :      40.0 :        6 : 1840 ms
#define DELAY_50             48.f   //  5 turns :      48.0 :        8 : 1830 ms
#define DELAY_40             60.f   //  4 turns :      60.0 :       12 : 1823 ms
#define DELAY_30             81.f   //  3 turns :      81.0 :       21 : 1837 ms
#define DELAY_20            121.f   //  2 turns :     121.0 :       40 : 1823 ms
#define DELAY_10            245.f   //  1 turn  :     245.0 :      124 : 1833 ms
#define DELAY_C0            550.f   // C0       :     600.0 :      305 :
#endif
///////////////////////////////////////////////////////////////////////////////////
#ifdef MAX_32_SPEED
                                    //          : MIN_DELAY : interval :      ms : multi
                                    // --------------------------------------------------
#define MIN_DELAY            32.f   // 10 turns :      32.0 :        0 : 2705 ms :  1.00
#define DELAY_90             37.f   //  9 turns :      37.0 :        5 : 2735 ms :  1.16
#define DELAY_80             42.f   //  8 turns :      42.0 :        5 : 2712 ms :  1.31
#define DELAY_70             49.f   //  7 turns :      49.0 :        7 : 2713 ms :  1.53
#define DELAY_60             58.f   //  6 turns :      58.0 :        9 : 2703 ms :  1.81
#define DELAY_50             71.f   //  5 turns :      71.0 :       13 : 2708 ms :  2.22
#define DELAY_40             90.f   //  4 turns :      90.0 :       19 : 2701 ms :  2.81
#define DELAY_30            122.f   //  3 turns :     122.0 :       32 : 2700 ms :  3.81
#define DELAY_20            187.f   //  2 turns :     187.0 :       65 : 2713 ms :  5.84
#define DELAY_10            378.f   //  1 turn  :     378.0 :      191 : 2703 ms : 11.81
#define DELAY_C0           1600.f   // C0       :    1600.0 :          :         : 50.00
#endif
////////////////////////////////////////////////////////////////////////////////////
#endif

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
/// #define X_SPD               float(A_PPR * A_GEAR_RATIO / 360) // steps per degree
/// #define Y_SPD               float(C_PPR * C_GEAR_RATIO / 360) // steps per degree
///
/////////////////////////////////////////////////////////////////////////////////////

#define X_DIR_PIN            2  // PD2(arduino digital pin 2)
#define X_STEP_PIN           3  // PD3(arduino digital pin 3)

#define Y_DIR_PIN            4  // PD4(arduino digital pin 4)
#define Y_STEP_PIN           5  // PD5(arduino digital pin 5)

#define Z_DIR_PIN            6  // PD6(arduino digital pin 6)
#define Z_STEP_PIN           7  // PD7(arduino digital pin 7)

#define A_DIR_PIN            8  // PB0(arduino digital pin 8)
#define A_STEP_PIN           9  // PB1(arduino digital pin 9)

#define B_DIR_PIN           10  // PB2(arduino digital pin 10)
#define B_STEP_PIN          11  // PB3(arduino digital pin 11)

#define C_DIR_PIN           12  // PB4(arduino digital pin 12)
#define C_STEP_PIN          13  // PB5(arduino digital pin 13)

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
