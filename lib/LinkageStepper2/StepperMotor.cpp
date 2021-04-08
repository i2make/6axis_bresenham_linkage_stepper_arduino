
#include "StepperMotor.hpp"

Motor::Motor(void(* _direction)(int), void(* _pulse)()) {
    // setting function
    direction = _direction;
    pulse = _pulse;

    // setting variable
    delayC0 = DELAY_C0;
    delayValue = delayC0;
}