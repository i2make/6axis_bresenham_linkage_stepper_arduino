# 6axis_bresenham_linkage_stepper_arduino
testing arduino uno
pin 02 ~ 13
# setup

- file: StepperDefine.hpp
Set the number of axes to be used
number of axis: 1u ~ 6u
```cpp
 #define MAX_AXIS 5u // number of axis
 ```
- file: Linkage_Stepper.ino
Create the same number of objects as the number of axes
```cpp
Motor X_Axis(xDirection, xPulse);
Motor Y_Axis(yDirection, yPulse);
Motor Z_Axis(zDirection, zPulse);
Motor A_Axis(aDirection, aPulse);
Motor B_Axis(bDirection, bPulse);
//Motor C_Axis(cDirection, cPulse);
```
- file: Linkage_Stepper.ino
add to world
```cpp
void setup() {
    
    Serial.begin(9600);
    
    world.addMotor(&Y_Axis);
    world.addMotor(&Z_Axis);
    world.addMotor(&A_Axis);
    world.addMotor(&B_Axis);
    //world.addMotor(&C_Axis);
  ```
   end
