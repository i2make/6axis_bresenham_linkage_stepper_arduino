# 6axis_bresenham_linkage_stepper_arduino
testing arduino uno

# setup

- file: StepperDefine.hpp:  
Set the number of axes to be used  
number of axis: 1u ~ 6u
```cpp
/// number of axes to be used
#define MAX_AXIS            5u          // number of axis (1u ~ 6u)
```

- Pin define (line 108~)  
pin 02 ~ 13 (X Y Z A B C axis direction, pulse)  
pin 18 (pause button)  
pin 19 (resume button)  
pin 17 (speed control analog input)

- Set Stepper speed
```cpp
/// stepper max speed
//#define MAX_20_SPEED
#define MAX_23_SPEED
//#define MAX_32_SPEED
```

- file: Linkage_Stepper.ino
Create movement
```cpp
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
```

end
