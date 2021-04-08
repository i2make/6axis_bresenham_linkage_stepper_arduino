#ifndef STEPPER_MOTOR_KEYPAD4X4_HPP
#define STEPPER_MOTOR_KEYPAD4X4_HPP

#include <Arduino.h>

#define DEBOUNCE_TIME           20  //int debounceTime(20);

// ======================================================================

/// KeyPad pin setting
const int r[4] = {13, 12, 11, 10};
const int c[4] = {6, 7, 8, 9};


const uint8_t key[4][4] = {{'C', '/', '*', '<'},
                           {'7', '8', '9', '-'},
                           {'4', '5', '6', '+'},
                           {'1', '2', '3', '('}};

extern volatile uint8_t pressedKey;

// ====================================================================

void keypadInit();

uint8_t GetKey();

#endif //STEPPER_MOTOR_KEYPAD4X4_HPP
