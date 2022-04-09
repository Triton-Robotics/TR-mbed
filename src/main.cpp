#include "main.hpp"

Motor wheel(7,GIMBLY);
Motor m3(1,STANDARD);

int main(){
    motorDebug = 1;
    while(1){
        printf("DELETUS\n");
        int val = 8738;
        wheel.setDesiredCurrent(val);
        m3.setDesiredCurrent(500);
        Motor::tick();
    }
}