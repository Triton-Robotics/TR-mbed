#include "mbed.h"
#include <cstdlib>
#include <cctype>


static BufferedSerial mySerial(PA_0, PA_1, 9600);
//Thread thread;

class InputTester{
    private:
        char message[32];
        int data[9];

    public: 

    InputTester() {}

     void updateThread() {
        while (true) {
            update();
            ThisThread::sleep_for(1ms);
        }
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

    void update() {
        if (mySerial.readable()) {
            ThisThread::sleep_for(31ms); // Very important so that you can actually "read" the entire message. Should be: Time(ms) = NumChars + 5
            mySerial.read(message, sizeof(message));
            printf("message : %s", message);

            getData(message);

            for (int i = 0; i < sizeof(message); i++)
                message[i] = '\0';

        }
    }    

    
};
