# SerialCommunication.hpp

- Facilitates Computer to Nucleo communication through Mbed Studio's serial monitor. (Great for testing)
  
  - Because the "Enter" button does not do anything on the Serial Monitor, pressing '\' will act like the Enter button in our SerialComm class.
  
  - Prints out data as you type and only sends when you press backslash

- Reads incoming Serial Data from any RX capable port on the Nucleo 

## Constructor

```cpp
SerialCommunication(PinName TX, PinName RX, int baud)
```

**TX** Any TX Capable pin

**RX** Any RX Capable pin

**baud** Baudrate, usually set to 9600

For example you can use the Tx and Rx built into the mini-usb connecting to the pc.

`SerialCommunication Serial(USBTX, USBRX, 9600)`

## Functions

### bool update(char message[])

- Takes in a **char array**, which is where the message will be stored into if there is new data in the Serial line. 

- Returns **True** if there is new data

### int toNum(char message[])

- Converts message array to an int (if valid)

- Returns INT if possible to convert, NULL if not possible

## Example

- Make sure the Serial monitor is open. View -> Serial Monitor

- When you are done typing your message use the '\' as the enter button

```cpp
#include "mbed.h"
#include "../TR-mbed6/util/communications/SerialCommunication.hpp"

SerialCommunication Serial(USBTX, USBRX, 9600);

char mycoolmessage[64];

int main() {

  while (1) {

        if (Serial.update(mycoolmessage)) {
            printf("your new messsage: %s\n", mycoolmessage);

            if (Serial.toNum(mycoolmessage) != NULL) 
                printf("message as an int: %d\n", Serial.toNum(mycoolmessage));

        }

    }

}
```

# pwmmotor.cpp (TEST FIRST @ MINGWEI)

- Outputs the standard pulse widths (from 1ms to 2ms) for controlling most servos / brushless ESCs that use pwm to modulate the output command

## Constructor

```cpp
PWMMotor(PinName pin, double defaultVal)
```

**pin** Any PWM capable output pin 

**defaultVal** PWM command from **0 f - .99 f** sent to the pin on startup. Useful for Brushless ESCs where they require an "intialization" command equivalent to setting the throttle to zero.



## void set(double motorVal)

Outputs PWM to the pin. From **0 f - 0.99 f** and correlates from a pulse width of 1ms to 2ms.

Note: The 'f' is needed at the end of every number in order to tell the compiler that you want to using this as a floating-point number (to ming's knowledge)



## Example

- Intializes PWM pin D5 to control a PWM capable ESC

- Uses Serial communication library to allow user to input desired PWM value on the fly

```cpp
#include "mbed.h"
#include "stdlib.h"
#include "../TR-mbed6/util/communications/pwmmotor.cpp"
#include "../TR-mbed6/util/communications/SerialCommunication.hpp"

PWMMotor brushlessmotor(D5,0.1f);

// Create a BufferedSerial object with a default baud rate.
SerialCommunication Serial(USBTX, USBRX, 9600);
static char message[32];


int main()
{
    while (true) {

        if (Serial.update(message)) {
            brushlessmotor.set(atof(message)); 
            printf("success\n");
        }
        
    }
}

```
