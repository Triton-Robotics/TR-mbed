#include "include/infantry.hpp"

    Infantry::Infantry(): 
        led(LED1), 
        led2(PA_5)
    {
        
    }

    int Infantry::execute() {
        led = !led;
        osDelay(500);
        return 1;
    }

    void Infantry::log() {
        printf("everythin going good!\n");
    }

    int Infantry::finish(){
        return 1;
    }
