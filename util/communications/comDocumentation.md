# SerialCommunication.hpp

- Enables reading through any TX/RX Capable Pin

- Facilitates Computer to Nucleo communication through Mbed Studio's serial monitor. (Great for testing)
  
  - Because the "Enter" button does not do anything on mbed's serial monitor, pressing '\' will act like the Enter button in our SerialComm class.
  
  - Prints out data as you type and only sends the data to the nucleo when you press backslash

- Converts string to an int

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

### PCRead(char message[])

- Takes in a **char array**

- Handles the update() function automatically

- Returns true once backslash (\\\) is sent and updates the char message with text in the serial monitor 

#### update(char message[], int sizeOfMessage, int sleep_fordelay)

- Takes in a **char array**, which is where the message will be stored into if there is new data in the Serial line. 

- Pass in the **size of the message array** 

- Pass in a **sleep for delay** (ms). A general rule of thumb is sizeOfMessage + 5. Though increase this constant if sizeOfMessage > 50
  
  - This is very important so that you don't read parts of your message. You must wait a bit so that you can get all of the data

- Returns **True** if there is new data and automatically populates **message** with this data.

#### toNum(char message[])

- Converts message array to an int (if valid)

- Returns **INT** if possible to convert, -999 if not possible (Since 0 = NULL)

#### strCompare(char word1[], char word2[])

- Compares two strings

- Returns **True** if they are the same

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

        if (Serial.PCRead(mycoolmessage)) {
            printf("your new messsage: %s\n", mycoolmessage);

            if (Serial.toNum(mycoolmessage) != -999) 
                printf("message as an int: %d\n", Serial.toNum(mycoolmessage));

        }

    }

}
```

# 
