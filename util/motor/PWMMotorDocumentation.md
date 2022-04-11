# pwmmotor.cpp (TEST FIRST @MINGWEI)

- Outputs the standard pulse widths (from 1ms to 2ms) for controlling most servos / brushless ESCs that use pwm to modulate the output command

# Class Constructor:

```cpp
PWMMotor(PinName pin, double defaultVal)
```

**pin** : the pin the motor is on, Any PWM capable output pin. This will be the pin you plug the actual data signal. This is usually the white pin, or the pin that isn't black and red.

**defaultVal** : PWM command from **0 f - .99 f** sent to the pin on startup. Useful for Brushless ESCs where they require an "intialization" command equivalent to setting the throttle to zero.

## Motor Movement

You can move a motor with this function:

`void set(double motorVal)`

This outputs PWM to the pin. motorVal can be a value from **0 f - 0.99 f** and correlates from a pulse width of 1ms to 2ms.

Note: The 'f' is needed at the end of every number in order to tell the compiler that you want to using this as a floating-point number (to ming's knowledge)

## Example

 Intializes PWM pin D5 to control a PWM capable ESC

```cpp
#include "mbed.h"
#include "stdlib.h"
#include "../TR-mbed6/util/motor/pwmmotor.cpp"

PWMMotor brushlessmotor(D5,0.1f);

int main(){
    while (1) {
        brushlessmotor.set(0.8);
    }
}
```
