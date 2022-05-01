#ifndef SerialCommunication_hpp
#define SerialCommunication_hpp
#include "mbed.h"
#include <cctype>
#include <ratio>


class SerialCommunication : BufferedSerial {
    private:
        char comdata[6];
        int index = 0;  
    
    public:
        /**
        * @brief Construct a new Serial Communication Object
        * @param TX is TX pin or USBTX
        * @param RX is RX pin or USBRX 
        * @param baud Set baud rate (optional, default is 9600)
        */
        SerialCommunication(PinName TX, PinName RX, int baud) : BufferedSerial(TX, RX, baud) {

        }

        /**
        * @brief Check for new input into Serial monitor. 
        * @param message is a char array[] intialized in the main function of which the message will be filled into only when '\' is sent.
        * @return Returns True if new message
        */
        bool PCRead(char message[]) {
            if (comdata[0] == '\\') { // Clear previous message if backslash was hit
                    int i = 0;
                    while(message[i] != '\0') 
                        message[i++] = '\0';
                }

            if (update(comdata, 6, 1)) {
                int z = 0;
                if (comdata[0] == '\\') { // Return true once backslash is hit
                    index = 0;
                    printf("\n");
                    return true;
                } else {
                    while (comdata[z] != '\0') {
                        printf("%s", comdata);
                        fflush(stdout); // Immediately print out what it receieved
                        message[index] = comdata[z];
                        z++;
                        index++;
                    }
                }
            }
            return false;
        }

        /**
        * @brief Check for new data into Serial. 
        * @param message is a char array[] intialized in the main function and the new message will be stored into that array if there is a new message
        * @param sizeOfMessage size of the char array[] 
        * @param sleep_fordelay Used to calculate the delay which is ESSENTIAL for reading data correctly. As a general rule it should be sizeOfMessage + 10  
        * @return Returns True if new message
        */
        bool update(char message[], int sizeOfMessage, int sleep_fordelay) {
            if (readable()) {
                for (int i = 0; i < sizeOfMessage; i++)
                    message[i] = 0;

                ThisThread::sleep_for(std::chrono::milliseconds(sleep_fordelay));
                read(message, sizeOfMessage);
                return 1;
            }
            return 0;
        }

        /**
        * @brief Check whether message is a number. Works with negatives as well
        * @param message is a string of chars to convert
        * @return integer if it is a number, NULL if not convertable.
        */
        int toNum(char message[]) {
            int i = 0;
            if (message[i] == '-')
                i++;
            while(message[i] != '\0') {
                if (!isdigit(message[i]))
                    return NULL;
                i++;
            }
            return std::atoi(message);
        }

};
#endif