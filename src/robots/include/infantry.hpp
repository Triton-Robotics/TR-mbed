#include "robot.hpp"

class Infantry : Robot {
    DigitalOut led;
    DigitalOut led2;
    
    public:
        Infantry();
        
        void log();
        int execute();
        int finish();
};