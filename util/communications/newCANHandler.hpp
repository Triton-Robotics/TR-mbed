#include "mbed.h"
#include "CANMsg.h"

///////////////////////////////////////////////////////////////////////
//                        ____               __       __  __  _ _____                   ____               ____     ___   __   _ ___    __  _          
// _______ ___ _____ ___ / __/__  ___ ____ _/ /  ___ / /_/ /_(_) ___/__  __ _  ___ ___ / __ \___  _______ /  _/__  / _ | / /  (_) _/__ / /_(_)_ _  ___ 
/// __/ _ `/ // (_-</ -_)\ \/ _ \/ _ `/ _ `/ _ \/ -_) __/ __/ / /__/ _ \/  ' \/ -_|_-</ /_/ / _ \/ __/ -_)/ // _ \/ __ |/ /__/ / _/ -_) __/ /  ' \/ -_)
//\__/\_,_/\_,_/___/\__/___/ .__/\_,_/\_, /_//_/\__/\__/\__/_/\___/\___/_/_/_/\__/___/\____/_//_/\__/\__/___/_//_/_/ |_/____/_/_/ \__/\__/_/_/_/_/\__/ 
//                        /_/        /___/                                                                                                             
//
///////////////////////////////////////////////////////////////////////
#ifndef newCanHandler_hpp
#define newCanHandler_hpp

#define CAN_BAUD 1000000

class NewCANHandler{
    private:
        CANMsg txMsg; //Message object reused to send messages to motors
        CANMsg rxMsg; //Message object reused to recieve messages from motors
        
        
    public:
        CAN can;

        enum CANBus {CANBUS_1, CANBUS_2, NOBUS};

        bool exists = false;
        // Declaring CanHandler, can1, and can2
        
        NewCANHandler():
            can(PA_11,PA_12,CAN_BAUD)
            {exists = false;}

        NewCANHandler(PinName canRx, PinName canTx):
            can(canRx,canTx,CAN_BAUD)
            {exists = true;}

        void attach	(Callback< void()> 	func,
        CAN::IrqType 	type = CAN::IrqType::RxIrq 
        ){
            can.attach(func,type);
        }

        void updateCANs(PinName canRx, PinName canTx){
            //can = new CAN(canRx,canTx,1000000);
            CAN can(canRx,canTx,CAN_BAUD);
        }

        CAN* getCAN(){
            return &can;
        }

        /**
        * @brief Get feedback back from the motor
        * 
        */
        bool getFeedback(int *id, uint8_t bytes[]){
            bool gotMsg = false;
            rxMsg.clear();
            rxMsg.len = 8;
            while (can.read(rxMsg)) {
                int err = can.rderror();
                if (err){
                    printf("[%d CAN Read Errors]\n", err);
                    can.reset();
                    return false;
                }
                *id = rxMsg.id;
                for(int i = 0;  i < 8; i ++){
                    rxMsg >> bytes[i]; //Extract information from rxMsg and store it into the bytes array
                }
                gotMsg = true;
                //printf("Motor 0x%x:\tAngle (0,8191):%d\tSpeed  ( RPM ):%d\tTorque ( CUR ):%d\tTemperature(C):%d \n",rxMsg.id,feedback[motorID][0],feedback[motorID][1],feedback[motorID][2],feedback[motorID][3]);
            }
            return gotMsg;
            //CAN Recieving from feedback IDs
        }

        /**
        * @brief Prints a CANMessage nicely
        * 
        * @param msg 
        */
        static void printMsg(CANMessage& msg)
        {
            printf(" ID = 0x%.3x\t", msg.id);
            for (int i = 0; i < msg.len; i++)
                printf(" 0x%.2X", msg.data[i]);
            printf("\n");
        }

        /**
         * @brief Raw sending of CAN Messages
         * 
         * @param id the CAN ID you're sending to
         * @param bytes the bytes you're sending (8)
         * @param bus the bus you're sending the CAN messages to
         */
        bool rawSend(int id, int8_t bytes[]){
            static int errorCount = 0;
            txMsg.clear(); // clear Tx message storage
            txMsg.id = id; 

            for(int i = 0; i < 8; i++){
                txMsg << int8_t(bytes[i]); //Take data from bytes array and one at a time store it into txMsg
            }

            //printMsg(txMsg);

            bool isWrite = can.write(txMsg);
            
            if(isWrite == 0) {
                errorCount++;
                can.reset();
            }
            else{
                errorCount = 0;
            }

            if (errorCount > 1000){
                printf("[CAN Connection Issues SEND]\n");
            }
            //printMsg(txMsg);
            return isWrite;
        }
};
#endif