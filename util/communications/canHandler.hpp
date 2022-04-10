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

        //////////////////////////////////////////////
        //VERY IMPORTANT TO SET FREQUENCY HERE AND NOW
        //////////////////////////////////////////////
        CANHandler(PinName can1Rx, PinName can1Tx, PinName can2Rx, PinName can2Tx):
            can1(can1Rx,can1Tx,1000000), 
            can2(can2Rx,can2Tx,1000000)
        {exists = true;}

        enum CANBus {CANBUS_1, CANBUS_2};

        /**
            * @brief Get feedback back from the motor
            * 
            */
        bool getFeedback(uint8_t bytes[], CANBus bus){
            rxMsg.clear();
            if (busAt(bus)) {
                for(int i = 0;  i < 8; i ++){
                    rxMsg >> bytes[i]; //2 bytes per motor
                }
                return true;
                //printf("Motor 0x%x:\tAngle (0,8191):%d\tSpeed  ( RPM ):%d\tTorque ( CUR ):%d\tTemperature(C):%d \n",rxMsg.id,feedback[motorID][0],feedback[motorID][1],feedback[motorID][2],feedback[motorID][3]);
            }
            return false;
            //CAN Recieving from feedback IDs
        }

        /**
        * @brief Raw sending of CAN Messages
        * 
        * @param id the CAN ID you're sending to
        * @param bytes the bytes you're sending
        * @param bus the bus you're sending the CAN messages to
        */
        bool rawSend(int id, int8_t bytes[], CANBus bus){
            txMsg.clear(); // clear Tx message storage
            txMsg.id = id; 

            for(int i = 0;  i < 8; i ++){
                txMsg << bytes[i]; //2 bytes per motor
            }
            bool isWrite = 1;
            isWrite = (*busAt(bus)).write(txMsg);
            if(isWrite == 0){
                printf("Transmission error\n");
                //break; 
            }
            return isWrite;
        }

        /**
        * Handles two CAN busses for a total of 16 motors for a robot.
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