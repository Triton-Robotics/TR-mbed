#include "oledDisplay.h"

//oledDisplay::i2c(PB_7,PB_8);

oledDisplay::oledDisplay() : i2c(PB_7, PB_8) , oled(i2c,PA_7){
    printf("aaaaaa\n");
}

//display the total number of motor count defined
//display the total number of motor that actually exist
int oledDisplay::motorCount(){

    int count = DJIMotor::motorCount;
    oled.clearDisplay();
    oled.writeChar('c');
    //next goal be able to display inteher on t
    oled.display();
    return EXIT_SUCCESS;
}

//    oled.setTextCursor(0,0);
//    char str[5] = "";
//    sprintf(str, "s%d\n",15);
//oled.write("motor",5);
//you need to write display everytime you use anything with write or write char,
// or there will be a weird output
//    oled.write(str, 2);
//    oled.writeChar('t');
//    oled.writeChar('e');