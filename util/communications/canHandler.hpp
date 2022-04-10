#include "mbed.h"
#include "CANMsg.h"

class CANHandler{
    private:
        CANMsg txMsg; //Message object reused to send messages to motors
        CANMsg rxMsg; //Message object reused to recieve messages from motors
        CAN can1;
        CAN can2;
        

    public:
        CANHandler(PinName can1Rx, PinName can1Tx, PinName can2Rx, PinName can2Tx);
        enum CANBus {CANBUS_1, CANBUS_2};
        bool rawSend(int id, int bytes[], CANBus bus);
        bool getFeedback(int id, int bytes[], CANBus bus);
        CAN* busAt(CANBus bus);
};