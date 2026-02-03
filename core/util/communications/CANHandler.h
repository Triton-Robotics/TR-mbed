///////////////////////////////////////////////////////////////////////
//                        ____               __       __  __  _ _____                   ____               ____     ___   __   _ ___    __  _          
// _______ ___ _____ ___ / __/__  ___ ____ _/ /  ___ / /_/ /_(_) ___/__  __ _  ___ ___ / __ \___  _______ /  _/__  / _ | / /  (_) _/__ / /_(_)_ _  ___ 
/// __/ _ `/ // (_-</ -_)\ \/ _ \/ _ `/ _ `/ _ \/ -_) __/ __/ / /__/ _ \/  ' \/ -_|_-</ /_/ / _ \/ __/ -_)/ // _ \/ __ |/ /__/ / _/ -_) __/ /  ' \/ -_)
//\__/\_,_/\_,_/___/\__/___/ .__/\_,_/\_, /_//_/\__/\__/\__/_/\___/\___/_/_/_/\__/___/\____/_//_/\__/\__/___/_//_/_/ |_/____/_/_/ \__/\__/_/_/_/_/\__/ 
//                        /_/        /___/                                                                                                             
//
///////////////////////////////////////////////////////////////////////
#ifndef CANHandler_hpp
#define CANHandler_hpp

#include "mbed.h"
#include "CANMsg.h"
#include <cstdint>
#include <vector>


#define CAN_BAUD 1000000

// Exact ID callback
struct ExactCallback {
    uint32_t id;
    std::function<void(const CANMsg*)> func;
};

// Range callback (inclusive)
struct RangeCallback {
    uint32_t start_id;
    uint32_t end_id;
    std::function<void(const CANMsg*)> func;
};

class CANHandler{
    private:
        CANMsg txMsg; //Message object reused to send messages to motors
        CANMsg rxMsg; //Message object reused to recieve messages from motors
        
        public:
        CAN can;
        
        // callback lists
        std::vector<ExactCallback> exact_;
        std::vector<RangeCallback> range_;
        
        enum CANBus {CANBUS_1, CANBUS_2, NOBUS};

        bool exists = false;
        // Declaring CanHandler, can1, and can2
        
        CANHandler():
            can(PA_11,PA_12,CAN_BAUD), exact_(), range_()
            {exists = false;}

        CANHandler(PinName canRx, PinName canTx):
            can(canRx,canTx,CAN_BAUD), exact_(), range_()
            {exists = true;}

        void attach	(Callback< void()> 	func, CAN::IrqType type = CAN::IrqType::RxIrq){
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
         * @brief Add a callback for a specific id
         * 
         * @param id CANMessage id
         * @param func Callback function
         */
        void registerCallback(uint32_t id, std::function<void(const CANMsg*)> func) {
            ExactCallback exact_id = {id, std::move(func)};
            exact_.push_back(exact_id);
        }

        /**
         * @brief Add a callback for multiple ids
         * 
         * @param stard_id first CANMessage id
         * @param stard_id last CANMessage id
         * @param func Callback function
         */
        void registerCallback(uint32_t start_id, uint32_t end_id, std::function<void(const CANMsg*)> func) {
            RangeCallback range_ids = {start_id, end_id, std::move(func)};
            range_.push_back(range_ids);
        }

        /**
         * @brief Get feedback from all CAN devices, and send to relevant callbacks
         */
        void readAllCan() {
            rxMsg.clear();
            // Read through all CAN messages in this frame
            while(can.read(rxMsg)) {
                if (can.rderror()) {
                    break;
                }

                // Look through all direct callbacks
                for (auto callbacks : exact_) {
                    if (rxMsg.id == callbacks.id) {
                        callbacks.func(&rxMsg);
                    }
                }

                // Look through all range callbacks
                for (auto callbacks : range_) {
                    if (rxMsg.id >= callbacks.start_id && rxMsg.id <= callbacks.end_id) {
                        callbacks.func(&rxMsg);
                    }
                }
                rxMsg.clear();
            }
        }

        /**
        * @brief Get feedback back from the motor
        * 
        */
        bool getFeedback(int *id, uint8_t bytes[], int canbus){
            bool gotMsg = false;
            rxMsg.clear();
            rxMsg.len = 8;
            if (can.read(rxMsg)) {
                int err = can.rderror();
                if (err){
                    printf("[%d CAN Read Errors] on CANBus_%d\n", err, canbus + 1);
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

        bool rawRead(int *id, uint8_t bytes[], int length){
            bool gotMsg = false;
            rxMsg.clear();
            rxMsg.len = length;
            if (can.read(rxMsg)) {
                int err = can.rderror();
                if (err){
                    printf("[%d CAN Read Errors]\n", err);
                    can.reset();
                    return false;
                }
                *id = rxMsg.id;
                for(int i = 0;  i < rxMsg.len; i ++){
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
        bool rawSend(int id, int8_t bytes[], int length = 8){
            static int errorCount = 0;
            txMsg.clear(); // clear Tx message storage
            txMsg.id = id; 

            for(int i = 0; i < length; i++){
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
                //printf("[CAN Connection Issues SEND]\n");
            }
            //printMsg(txMsg);
            return isWrite;
        }
};
#endif