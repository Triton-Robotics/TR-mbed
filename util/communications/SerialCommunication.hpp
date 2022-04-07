#include "mbed.h"
#include <cctype>


class SerialCommunication : BufferedSerial {
    private:
        char comdata[6];
        int index = 0;  
    
    public:
        /**
        * @brief Construct a new Serial Communication Object
        * @param TX is TX pin or USBTX
        * @param RX is RX pin or USBRX 
        * @param baud Set baud rate (optional, default is 115200)
        */
        SerialCommunication(PinName TX, PinName RX, int baud) : BufferedSerial(TX, RX, baud) {

        }

        /**
        * @brief Check for new input into Serial monitor. 
        * @param message is a char array[] intialized in the main function of which the message will be filled into only when '\' is sent.
        * @return Returns True if new message
        */
        bool update(char *message) {
            if (readable()) {
                int z = 0;
                if (comdata[0] == '\\') { // Clear previous message
                    int i = 0;
                    while(message[i] != '\0') 
                        message[i++] = '\0';
                }

                for (int i = 0; i < sizeof(comdata); i++)
                    comdata[i] = 0;

                ThisThread::sleep_for(20ms);
                read(comdata, sizeof(comdata));

                if (comdata[0] == '\\') {
                    index = 0;
                    printf("\n");
                    return true;
                } else 
                    while (comdata[z] != '\0') {
                    printf("%s", comdata);
                    fflush(stdout); // Immediately print out what it receieved
                    message[index] = comdata[z];
                    z++;
                    index++;
                }
                return false;
            } 
            return false;
        }

        /**
        * @brief Check whether message is a number. Works with negatives as well
        * @param message is the message array
        * @return integer if it is a number, NULL if not convertable.
        */
        int isNum(char message[]) {
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
