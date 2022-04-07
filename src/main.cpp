#include "main.hpp"

Motor wheel(1,STANDARD);

int main(){
    
    while(1){
        int val = 8738;
        wheel.setDesiredCurrent(val);
    }
}