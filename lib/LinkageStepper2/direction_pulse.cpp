
#include "StepperDefine.hpp"
#define DELAY_BETWEEN_PULSE         3

/// X axis 1 stepping pulse generation
void xPulse() {
    X_PULSE_HIGH
    delayMicroseconds(DELAY_BETWEEN_PULSE);
    X_PULSE_LOW
}

/// X axis direction selection
void xDirection(int dir) {
    digitalWrite(X_DIR_PIN, dir > 0 ? 1 : 0);
}

/// Y axis 1 stepping pulse generation
void yPulse() {
    Y_PULSE_HIGH
    delayMicroseconds(DELAY_BETWEEN_PULSE);
    Y_PULSE_LOW
}

/// Y axis direction selection
void yDirection(int dir) {
    digitalWrite(Y_DIR_PIN, dir > 0 ? 1 : 0);
}

/// Z axis 1 stepping pulse generation
void zPulse() {
    Z_PULSE_HIGH
    delayMicroseconds(DELAY_BETWEEN_PULSE);
    Z_PULSE_LOW
}

/// Z axis direction selection
void zDirection(int dir) {
    digitalWrite(Z_DIR_PIN, dir > 0 ? 1 : 0);
}

/// A axis 1 stepping pulse generation
void aPulse() {
    A_PULSE_HIGH
    delayMicroseconds(DELAY_BETWEEN_PULSE);
    A_PULSE_LOW
}

/// A axis direction selection
void aDirection(int dir) {
    digitalWrite(A_DIR_PIN, dir > 0 ? 1 : 0);
}

/// B axis 1 stepping pulse generation
void bPulse() {
    B_PULSE_HIGH
    delayMicroseconds(DELAY_BETWEEN_PULSE);
    B_PULSE_LOW
}

/// B axis direction selection
void bDirection(int dir) {
    digitalWrite(B_DIR_PIN, dir > 0 ? 1 : 0);
}

/// C axis 1 stepping pulse generation
void cPulse() {
    C_PULSE_HIGH
    delayMicroseconds(DELAY_BETWEEN_PULSE);
    C_PULSE_LOW
}

/// C axis direction selection
void cDirection(int dir) {
    digitalWrite(C_DIR_PIN, dir > 0 ? 1 : 0);
}

