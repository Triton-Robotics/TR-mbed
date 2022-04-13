Outputs the standard pulse widths (from 1ms to 2ms) for controlling most servos / brushless ESCs that use pwm to modulate the output command

# Class Constructor:

```cpp
PWMMotor(PinName pin, int defaultVal)
```

**pin** : the pin the motor is on, Any PWM capable output pin. This will be the pin you plug the actual data signal. This is usually the white or yellow pin, and definitely not the pin that is black or red

**defaultVal** : integer PWM command from **0 - 180** sent to the pin on startup. Useful for Brushless ESCs where they require an "intialization" command equivalent to setting the throttle to zero.

# Motor Movement

You can move a motor with this function:

`void set(int motorVal)`

This outputs PWM to the pin. motorVal can be a value from **0 - 180** and correlates from a pulse width of 1ms to 2ms.

# Example

 Intializes PWM pin D5 to control a PWM capable ESC. First sets an int 90 command on startup. Then waits 100ms and later sends a new command 

```cpp
#include "mbed.h"
#include "stdlib.h"
#include "../TR-mbed6/util/motor/pwmmotor.cpp"

PWMMotor brushlessmotor(D5,90);

int main(){
    osDelay(100);
    while (1) {
        brushlessmotor.set(10);
    }
}
```

# Important Note

Note that some servos/ESCs don't use a pulse-width from 1ms to 2ms (for some reason) even though the datasheet says it should. During Ming's testing, some servos required a value from -90 to 90 for their full range of motion or 0 - 230. Don't be afriad to go outside of the boundaries.
