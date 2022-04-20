#include "mbed.h"
#include "CANMsg.h"
#ifndef canHandler_hpp
#define canHandler_hpp
class CANHandler{
    private:
        CANMsg txMsg; //Message object reused to send messages to motors
        CANMsg rxMsg; //Message object reused to recieve messages from motors
        CAN can1;
        CAN can2;
        
        
    public:

        bool exists = false;
        // Declaring CanHandler, can1, and can2
        CANHandler(PinName can1Rx, PinName can1Tx, PinName can2Rx, PinName can2Tx):
            can1(can1Rx,can1Tx,1000000), 
            can2(can2Rx,can2Tx,1000000) 
            {exists = true;}

        

        enum CANBus {CANBUS_1, CANBUS_2};

        /**
        * @brief Get feedback back from the motor
        * 
        */
        bool getFeedback(int *id, uint8_t bytes[], CANBus bus){
            rxMsg.clear();
            rxMsg.len = 8;
            if (busAt(bus)->read(rxMsg)) {
                //printMsg(rxMsg);
                *id = rxMsg.id;
                for(int i = 0;  i < 8; i ++){
                    rxMsg >> bytes[i]; //Extract information from rxMsg and store it into the bytes array
                }
                return true;
                //printf("Motor 0x%x:\tAngle (0,8191):%d\tSpeed  ( RPM ):%d\tTorque ( CUR ):%d\tTemperature(C):%d \n",rxMsg.id,feedback[motorID][0],feedback[motorID][1],feedback[motorID][2],feedback[motorID][3]);
            }
            return false;
            //CAN Recieving from feedback IDs
        }

        /**
        * @brief Prints a CANMessage nicely
        * 
        * @param msg 
        */
        static void printMsg(CANMessage& msg)
        {
            printf("  ID      = 0x%.3x\r\n", msg.id);
            printf("  Type    = %d\r\n", msg.type);
            printf("  Format  = %d\r\n", msg.format);
            printf("  Length  = %d\r\n", msg.len);
            printf("  Data    =");
            for (int i = 0; i < msg.len; i++)
                printf(" 0x%.2X", msg.data[i]);
            printf("\r\n");
        }

        /**
        * @brief Raw sending of CAN Messages
        * 
        * @param id the CAN ID you're sending to
        * @param bytes the bytes you're sending (8)
        * @param bus the bus you're sending the CAN messages to
        */
        bool rawSend(int id, int8_t bytes[], CANBus bus){
            static int errorCount = 0;
            txMsg.clear(); // clear Tx message storage
            txMsg.id = id; 

            for(int i = 0; i < 8; i++){
                txMsg << bytes[i]; //Take data from bytes array and one at a time store it into txMsg
            }

            bool isWrite = busAt(bus)->write(txMsg);
            
            if(isWrite == 0) {
                errorCount++;
            }
            else{
                errorCount = 0;
            }

            if (errorCount > 1000){
                //printf("Transmission error in rawSend()\n");
            }//printMsg(txMsg);
            return isWrite;
        }


        /**
        * @brief Handles two CAN busses for a total of 16 motors for a robot.
        *
        * @return Returns a pointer to the correct CAN bus (either can 1 or can 2)
        */
        CAN* busAt(CANBus bus){
            if(bus == CANBUS_1)
                return &can1;
            else if(bus == CANBUS_2)
                return &can2;
            return NULL;
        }
};
#endif