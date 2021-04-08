#include <KeyPad4X4.hpp>


// ============================================================================

void keypadInit() {
    for (int i = 0; i < 4; ++i) {
        pinMode(r[i], INPUT_PULLUP);
        pinMode(c[i], OUTPUT);
        digitalWrite(c[i], HIGH);       // inactive
    }
}

uint8_t GetKey() {
    for (int j = 0; j < 4; ++j) {
        digitalWrite(c[j], LOW);            // one column active

        for (int i = 0; i < 4; ++i)
            if (!digitalRead(r[i])) {
                delay(DEBOUNCE_TIME);         //debounce
                while (!digitalRead(r[i]));   //wait for to be released
                pressedKey = key[i][j];
            }
        digitalWrite(c[j], HIGH);           // one column inactive
    }
    return pressedKey;
}

