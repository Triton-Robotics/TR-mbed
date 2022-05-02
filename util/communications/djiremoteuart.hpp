#include "../communications/SerialCommunication.hpp"
#include "../helperFunctions.hpp"

enum joysticks{
    LEFTJOYX,
    LEFTJOYY,
    RIGHTJOYX,
    RIGHTJOYY,
    WHEEL,
};

enum switches {
    LSWITCH,
    RSWITCH
};

class DJIRemote : SerialCommunication {
    private: 
        char mymessage[12];
        int data[7] = {0,0,0,0,0,0,0};

        int charToNum(char MSC, char LSC, bool switches = 0) {
            if (!switches)
                return ((int)(MSC)-32) * 94 + ((int)(LSC)-32) - 660;
            else {
                int i = ((int)(LSC)-32);
                if (i == 1)
                    return 3;
                else if (i == 2) 
                    return 1;
                else
                    return 2; 
            }
                
        }


        void getData() {
            data[0] = charToNum(mymessage[0], mymessage[1]);
            data[1] = charToNum(mymessage[2], mymessage[3]);
            data[2] = charToNum(mymessage[4], mymessage[5]);
            data[3] = charToNum(mymessage[6], mymessage[7]);
            data[4] = charToNum(mymessage[8], mymessage[9]);
            data[5] = charToNum(' ', mymessage[10], 1);
            data[6] = charToNum(' ', mymessage[11], 1);
        }

    public: 
    DJIRemote(PinName TX_PIN, PinName RX_PIN) : SerialCommunication(TX_PIN, RX_PIN, 1000000) {}

    void remoteUpdate(bool printData = 0) {
        if (update(mymessage, sizeof(mymessage), 20)) {
            getData();

            if (printData) {
            for (int i = 0; i < 7; i++) 
                printf("%d\t", data[i]);
            printf("\n");
            }
        }
    }

    float getStickData(joysticks stick, float lowerbound, float upperbound) {
        bool negative = false;
        if (data[stick] < 0)
            negative = true;
        float val = map(abs(data[stick]), 0, 660, lowerbound, upperbound);
        if (negative)
            val *= -1;
        return val;
    }

    /**
     * @brief Get data from Switches
     * 
     * @param switches Desired Switch 
     * @return int value (1, 2, or 3) (Low, Medium, High)
     */
    int getSwitchData(switches switchie) {
        return data[switchie+5];
    }

};

