#include "oledDisplay.h"
#include "peripherals/oled/SSD1308.h"

//I2C i2c(D_SDA, D_SCL);
//SSD1308 oled(&i2c,0x78);

//oledDisplay::i2c(PB_7,PB_8);

oledDisplay::oledDisplay() : i2c(PB_7, PB_8) , oled(&i2c,PA_7){
    printf("aaaaaa\n");
    oled.writeChar('c');
}

//display the total number of motor count defined
//display the total number of motor that actually exist
int oledDisplay::motorCount(){
    printf("is this working\n");
    oled.clearDisplay();

    oled.write("motor",5);

//    char count[4];
//    int literalCount = 0;
//
//    snprintf(count, 4, "%d",DJIMotor::motorCount);
//    oled.writeString(0,0,count);
//    int row = 0;
//
//    for(int i = 0; i < 2; i++){
//        printf("loop 1");
//        for(int j = 0; j < 3; j++){
//            printf("loop 2");
//            for(int k = 0; k < 4; k++){
//                if(DJIMotor::getMotorExist(i,j,k)){
//                    if(DJIMotor::s_isMotorConnected((CANHandler::CANBus)i, j, k)) {
//                        oled.writeString(row, 0, "Connected:");//...print which motor is connected
//                        row++;
//                    } else {
//                        oled.writeString(row, 0, "Disconnected"); //...print which motor is disconnected
//                        row++;
//                    }
//                } else {
//
//                }
//            }
//        }
//
//        printf("infinite loop?");
//        //check if the time_elapse is short using the variable in the DJIMotor
//    }
//
//    //next goal be able to display inteher on t
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