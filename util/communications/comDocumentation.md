# SerialCommunication.hpp

- Facilitates Computer -> Nucleo communication through Mbed Studio's serial monitor. (Great for testing)
  
  - Because the "Enter" button does not do anything on the Serial Monitor, pressing '\' will act like the Enter button.
  
  - Prints out data as you type and only sends when you press backslash

- Reads incoming Serial Data from any RX capable port on the Nucleo 

## Constructor

```cpp
SerialCommunication(PinName TX, PinName RX, int baud)
```

**TX** Any TX Capable pin

**RX** Any RX Capable pin

**baud** Baudrate, usually set to 9600

## Functions

### bool update(char message[])

- Takes in a **char array**. This is where the message will be stored into if there is new data in the Serial line. 

- Returns **True** if there is new data

### int toNum(char message[])

- Converts message array to an int (if valid)

- Returns INT if possible to convert, NULL if not possible

## Example

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
