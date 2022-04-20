#include "mbed.h"
#include <cstdlib>
#include <cctype>
#include "buttonanalyzer.hpp"


static BufferedSerial mySerial(PA_0, PA_1, 9600);


enum MyButtons {
    SMALL,
    YELLOW,
    BLUE,
    RED,
    GREEN,
};

enum MyPots{
    BOTTOM,
    LEFT,
    MIDDLE,
    RIGHT,
};

class InputTester{
    private:
        char message[35];
        int data[9] = {0,0,0,0,0,0,0,0,0};
        buttonanalyzer myButtons[5];

    float mapPots(int x, float out_min, float out_max) {
        float in_min = 0;
        float in_max = 1010;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    int toNum(char nums[]) {
            int i = 0;
            if (nums[i] == '-')
                i++;
            while(nums[i] != '\0' && nums[i] != '\n') {
                if (!isdigit(nums[i])) {
                    printf("Could not convert on %c\n", nums[i]);
                    return NULL;
                }
                i++;
            }
            return std::atoi(nums);
        }

    void getData(char msg[]) {
        int i = 0;
        int z = 0;
        int curIndex = 0;
        char numHolder[5] = {'\0','\0','\0','\0','\0'};
        while (msg[i] != '*') {

            if (isdigit(msg[i])) {
                numHolder[z] = msg[i];
                z++;
            }
            if (msg[i] == '|') {
                data[curIndex++] = toNum(numHolder);
                //printf("Nums: %d ", toNum(numHolder));
                z = 0;
                for (int index = 0; index < sizeof(numHolder); index++)
                numHolder[index] = '\0';
                }
            i++;
        }        
    }

    public: 

    InputTester() {}
    /**
    * @brief Update Potentiometer and Button data
    * NOTE: MUST BE CALLED IN MAIN LOOP
    *
    */
    void update() {
        for (int i = 0; i < 5; i++) {
            myButtons[i].update(data[i+4]);
        }
        if (mySerial.readable()) {
            ThisThread::sleep_for(50ms); // Very important so that you can actually "read" the entire message. Should be: Time(ms) = NumChars + 5
            mySerial.read(message, sizeof(message));
            //printf("message : %s", message);

            getData(message);

            for (int i = 0; i < sizeof(message); i++)
                message[i] = '\0';

        }
    }    

    /**
     * @brief Get data from Pot
     * 
     * @param pot Desired Potentiometer 
     * @param lowBound Lowest possible desired float value
     * @param highBound Highest possible desired float value
     * @return FLOAT value (NOTE: UNABLE TO BE PRINTFed UNLESS CAST TO AN INT)
     */
    float getPot(MyPots pot,float lowBound, float highBound) {
        return mapPots(data[pot], lowBound, highBound);
    }

    /**
     * @brief Returns button status
     * 
     * @param desiredButton Desired Button 
     * @return Either pressed or not pressed (1 or 0)
     */
    bool getButtonStatus(MyButtons desiredButton) {
        return myButtons[desiredButton].getStatus();
    }

    /**
     * @brief Returns toggling of button
     * 
     * @param desiredButton Desired Button 
     * @return Either toggled on or off. Default is off.
     */
    bool getButtonToggle(MyButtons desiredButton) {
        return myButtons[desiredButton].getToggle();
    }

    /**
     * @brief Returns the moment the button is pressed
     * 
     * @param desiredButton Desired Button 
     * @return 1 at the moment the button is pressed. 0 Elsewise.
     */
    bool getButtonPress(MyButtons desiredButton) {
        return myButtons[desiredButton].getInitialPress();
    }

    /**
     * @brief Returns the moment the button is released
     * 
     * @param desiredButton Desired Button 
     * @return 1 at the moment the button is released. 0 Elsewise.
     */
    bool getButtonRelease(MyButtons desiredButton) {
        return myButtons[desiredButton].getInitialRelease();
    }

    
};
