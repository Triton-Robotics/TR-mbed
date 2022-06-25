#include "../communications/SerialCommunication.hpp"
#include "../helperFunctions.hpp"
#include "../buttonanalyzer.hpp"
#include <iostream>
using namespace std;
/**
 * enum for all axix coming from the DJIRemote
 */
enum axis{
    LEFTJOYX,
    LEFTJOYY,
    RIGHTJOYX,
    RIGHTJOYY,
    WHEEL,
};

/**
 * enum for the switches coming from the DJIRemote
 */
enum switches {
    LSWITCH,
    RSWITCH
};

enum mouse {
    SPEEDX,
    SPEEDY,
    SPEEDZ,
    LCLICK,
    RCLICK
};

enum keyboard {
    W,
    S,
    A,
    D,
    SHIFT,
    CTRL,
    Q,
    E,
    R,
    F,
    G,
    Z,
    X,
    C,
    V
};

/**
 * The DJIRemote handler for Robomaster Devboard 
 * sending remote signals through UART
 */

static buttonanalyzer keyboardkeys[15];
static buttonanalyzer Mousebuttons[2];

class DJIRemote : SerialCommunication {
    private: 
        char mymessage[36]; //message to be read
        int data[12]; //data out
        int keyboardData[16];

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

            data[7] = charToNum(mymessage[12], mymessage[13]); //mousex
            data[8] = charToNum(mymessage[14], mymessage[15]); //mousey
            data[9] = charToNum(mymessage[16], mymessage[17]); //mousez
            data[10] = charToNum(' ', mymessage[18], 1)-2; // mouse left
            data[11] = charToNum(' ', mymessage[19], 1)-2; // mouse right

            int16ToBitArray((int)mymessage[20], keyboardData);

            for (int i = 0; i < 15; i++) 
                keyboardkeys[i].update(keyboardData[15-i]);
            
            Mousebuttons[0].update(data[10]); //mouse left
            Mousebuttons[1].update(data[11]); //mouse right

        }

    public: 
    /**
     * @brief Constructor for DJIRemote UART handler
     * 
     * @param TX_PIN the tx pin
     * @param RX_PIN the rx pin
     */
    DJIRemote(PinName TX_PIN, PinName RX_PIN) : SerialCommunication(TX_PIN, RX_PIN, 100000) {}

    /**
     * @brief get data from remote, must be continuously called!!
     * 
     * @param printData Type of data to print
     *  1 - DJI Remote data
     *  2 - Keyboard Data
     *  3 - Mouse Data
     */
    void remoteUpdate(int printData = 0) {
        if (update(mymessage, sizeof(mymessage), 5)) {
            getData();
            if (printData == 1) { // print DJI Remote data
                for (int i = 0; i < 7; i++)
                    printf("%d\t", data[i]);
                printf("\n");
            }
            else if (printData == 2) { // print Keyboard data
                for (int i = 0; i < 15; i++)
                    printf("%d\t", keyboardkeys[i].getStatus());
                printf("\n");
            }
            else if (printData == 3) { // print Mouse data
                printf("%d\t%d\t%d\t%d\t%d\n", getMouseData(SPEEDX),getMouseData(SPEEDY),getMouseData(SPEEDZ),getMouseData(LCLICK),getMouseData(RCLICK));
            }
        }
    }

    /**
     * @brief get a stick's data
     * 
     * @param lowerbound the lower bound of the output
     * @param upperbound the upper bound of the output
     */
    float getStickData(axis stick, float lowerbound = 0, float upperbound = 1) {
        bool negative = false;
        if (data[stick] < 0)
            negative = true;
        float val = map(abs(data[stick]), 0, 660, lowerbound, upperbound);
        if (negative)
            val *= -1;
        return val;
    }
    

    int getRawStick(axis stick) {
        return data[stick];
    }

    void getArray(int* out){
        for (int i = 0; i < 7; i++){
            out[i] = data[i];
        }
    }
    int getSwitchData(switches switchie) {
        return data[switchie+5];
    }

    int getMouseData(mouse button) {
        return data[7 + button];
    }

    bool getMouseButtonTogState(mouse button) {
        return Mousebuttons[button-3].getToggle();
    }

    bool getKeyState(keyboard key) {
        return keyboardkeys[key].getStatus();
    }

    bool getToggleState(keyboard key) {
        return keyboardkeys[key].getToggle();
    }

    bool getInitialPress(keyboard key) {
        return keyboardkeys[key].getInitialPress();
    }

    bool getInitialRelease(keyboard key) {
        return keyboardkeys[key].getInitialRelease();
    }



};

