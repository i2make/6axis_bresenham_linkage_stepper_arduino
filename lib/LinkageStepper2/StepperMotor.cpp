
#include "StepperMotor.hpp"

Motor::Motor(void(* _direction)(int), void(* _pulse)()) {
    // setting function
    direction = _direction;
    pulse = _pulse;

    currentPosition = 0;                    // current position
    targetPosition = 0;                 // target position
    dy = 0;                    // delta y = current position - newPosition
    absDy = 0;                 // abs(dy)

    dirValue = 0;               // position add dir (dir = 1 or -1)
    over = 0;                   // using bresenham line algorithm
}