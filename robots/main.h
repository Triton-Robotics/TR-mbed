//
// Created by ankit on 1/31/23.
//

#ifndef TR_EMBEDDED_MAIN_H
#define TR_EMBEDDED_MAIN_H

#include "mbed.h"

#include "helperFunctions.hpp"
#include "algorithms/PID.h"
#include "peripherals/oled/SSD1308.h"
#include "subsystems/Chassis.h"

#include "motor/PWMMotor.h"
#include "motor/DJIMotor.h"

#include "communications/CANHandler.h"
#include "communications/djiremoteuart.h"
#include "communications/SerialCommunication.h"
#include "communications/referee/ref_serial.h"
#include "communications/referee/ref_ui.h"

#include <cstring>

// #define OLED_SDA                  PB_9
// #define OLED_SCL                  PB_8
// I2C i2c(OLED_SDA, OLED_SCL);

// SSD1308 oled(&i2c, 0x78);

//#include "robots/include/infantry.hpp"#include "communications/DJIRemote.h"


static DJIRemote myremote(PA_0, PA_1);
static BufferedSerial referee(PC_10, PC_11, 115200); // Nucleo board: top left male pins.
static bool enablePrintRefData = 0;

CANHandler canHandler1(PA_11, PA_12);
CANHandler canHandler2(PB_12, PB_13);

static int lX = 0;
static int lY = 0;
static int rX = 0;
static int rY = 0;
static int Wh = 0;
static int lS = 0;
static int rS = 0;

void refereeThread(){

    // int loop=0;
    // while(1){
    if(referee.readable()){
        JudgeSystem_USART_Receive_DMA(&referee);
        Judge_GetMessage(JUDGESYSTEM_PACKSIZE);

        // if(loop % 10==0){ // print out only every 10 iterations
        if(enablePrintRefData){
            // string id="robot id: " + to_string(get_robot_id()) + "  ";
            string output = "robot id: %s  ";
            // string hp="robot hp: " + to_string(get_remain_hp() ) +"  ";

            // cout << id;
            if(is_red_or_blue()){
                // cout<<"RED  ";
                output += "RED  ";
            }
            else{
                // cout<<"BLUE  ";
                output += "BLUE  ";
            }
            // printf("robot hp: %d  ", ext_game_robot_state.data.remain_HP);
            // printf("max hp: %d  ", ext_game_robot_state.data.max_HP);
            printf("angle: %d  \n", (int)ext_game_robot_pos.data.yaw);
            // cout << "angle: "<< ext_game_robot_pos.data.yaw;
            output += "robot hp: %d  max hp: %d  angle: %d  power: %d  current: %d  volt: %d \n";

            // printf("power: %d  ", ext_power_heat_data.data.chassis_power);
            // printf("current: %d  ", ext_power_heat_data.data.chassis_current);
            // printf("volt: %d \n", ext_power_heat_data.data.chassis_volt);

            // printf(output.c_str(), get_robot_id(), ext_game_robot_state.data.remain_HP, ext_game_robot_state.data.max_HP, (int)ext_game_robot_pos.data.yaw,
            // (int)ext_power_heat_data.data.chassis_power, ext_power_heat_data.data.chassis_current, ext_power_heat_data.data.chassis_volt);

            // }
        }
    }
    else{
        // if(loop % 10==0){ // print out only every 10 iterations
        if(enablePrintRefData){
            printf("REFEREE - Not readable!\n");
        }
        // }
    }

    if(referee.writable()){
        // RobotStatus_LEDYellow(&referee);
        // RobotStatus_LEDYellow(&pc);
        // Show_CrossHair(&referee);

        // // delete drawing
        // ext_student_interactive_header_data_delete_t custom_delete_draw;
        // {
        //     custom_delete_draw.data_cmd_id=0x0100;
        //     custom_delete_draw.sender_ID=get_robot_id();//发送者ID，机器人对应ID
        //     custom_delete_draw.receiver_ID=get_robot_id()+0x0100;//接收者ID，操作手客户端ID
        //     {
        //         custom_delete_draw.graphic_custom.operate_tpye = 1;
        //         custom_delete_draw.graphic_custom.layer = 5;
        //     }
        // }

        // // draw text
        // ext_student_interactive_header_data_character_t custom_character_draw;
        // {
        //     custom_character_draw.data_cmd_id=0x0110;//绘制七个图形（内容ID，查询裁判系统手册）//0104 for 7 diagrams, 0110 for drawing text
        //     custom_character_draw.sender_ID=get_robot_id();//发送者ID，机器人对应ID
        //     custom_character_draw.receiver_ID=get_robot_id()+0x0100;//接收者ID，操作手客户端ID
        //     //自定义图像数据
        //     {
        //         // For drawing text ------------------

        //         //char c[] = "hello";
        //         string c = "power: " + to_string((int)ext_power_heat_data.data.chassis_power);
        //         // if(loop>50){
        //         //     strcpy(c,"byebye");
        //         // }
        //         for(int i=0; i<c.length(); i++){
        //             custom_character_draw.graphic_custom.data[i]=c[i];
        //         }
        //         for(int i=c.length(); i<30; i++){ // have to fill in the rest of the char array
        //             custom_character_draw.graphic_custom.data[i]=' ';
        //         }

        //         custom_character_draw.graphic_custom.grapic_data_struct.graphic_name[0] = 97;
        //         // custom_character_draw.graphic_custom.grapic_data_struct.graphic_name[1] = 98;
        //         // custom_character_draw.graphic_custom.grapic_data_struct.graphic_name[2] = 99;//图形名
        //         //上面三个字节代表的是图形名，用于图形索引，可自行定义
        //         custom_character_draw.graphic_custom.grapic_data_struct.operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
        //         custom_character_draw.graphic_custom.grapic_data_struct.graphic_tpye=7;//图形类型，0为直线，其他的查看用户手册
        //         custom_character_draw.graphic_custom.grapic_data_struct.layer=5;//图层数
        //         custom_character_draw.graphic_custom.grapic_data_struct.color=1;//颜色
        //         custom_character_draw.graphic_custom.grapic_data_struct.start_angle=20;
        //         custom_character_draw.graphic_custom.grapic_data_struct.end_angle=10;
        //         custom_character_draw.graphic_custom.grapic_data_struct.width=2;
        //         custom_character_draw.graphic_custom.grapic_data_struct.start_x=SCREEN_LENGTH/2 +100;//10
        //         custom_character_draw.graphic_custom.grapic_data_struct.start_y=SCREEN_WIDTH/2 +100;

        //     }
        // }

        // For graphing diagrams ----------------------------
        ext_student_interactive_header_data_graphic_t custom_graphic_draw;	//自定义图像
        {
            custom_graphic_draw.data_cmd_id=0x0104;//绘制七个图形（内容ID，查询裁判系统手册）//0104 for 7 diagrams, 0110 for drawing text
            custom_graphic_draw.sender_ID=get_robot_id();//发送者ID，机器人对应ID
            custom_graphic_draw.receiver_ID=get_robot_id()+0x0100;//接收者ID，操作手客户端ID
            {
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 7;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 8;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 9;//图形名
                //上面三个字节代表的是图形名，用于图形索引，可自行定义
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].layer=9;//图层数
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].color=1;//颜色
                // custom_graphic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
                // custom_graphic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].width=2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2 -100;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2 -50;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].end_x=SCREEN_LENGTH/2 +100;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].end_y=SCREEN_WIDTH/2 -50;
                // custom_graphic_draw.graphic_custom.grapic_data_struct[0].radius=10;
            }
            {
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].graphic_name[0] = 1;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=1;//图形类型，0为直线，其他的查看用户手册
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].layer=9;//图层数
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].color=1;//颜色
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].start_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].end_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].width=2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].start_x=SCREEN_LENGTH/2 -75;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].start_y=SCREEN_WIDTH/2 -100;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].end_x=SCREEN_LENGTH/2 +75;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].end_y=SCREEN_WIDTH/2 -100;
            }
            {
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].graphic_name[0] = 2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].layer=9;//图层数
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].color=1;//颜色
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].start_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].end_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].width=2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].start_x=SCREEN_LENGTH/2 -50;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].start_y=SCREEN_WIDTH/2 -150;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].end_x=SCREEN_LENGTH/2 +50;
                custom_graphic_draw.graphic_custom.grapic_data_struct[2].end_y=SCREEN_WIDTH/2 -150;
            }
        }

        // referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_delete_draw,sizeof(custom_delete_draw),&referee);
        ui_delete_all(&referee);
        // referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_character_draw,sizeof(custom_character_draw),&referee);
        referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_graphic_draw,sizeof(custom_graphic_draw),&referee);

        string powerStr = "power: " + to_string((int)ext_power_heat_data.data.chassis_power);
        ui_graph_character(&referee, 1, powerStr, SCREEN_LENGTH/2 +100, SCREEN_WIDTH/2 +100, 99);

        // printf("write!");
        // pc.write("hello",5);
        string angleStr = "angle: " + to_string((int)ext_game_robot_pos.data.yaw);
        ui_graph_character(&referee, 1, angleStr, SCREEN_LENGTH/2 +100, SCREEN_WIDTH/2 +150, 10);

        ext_student_interactive_header_data_robot_comm_t custom_comm;
        {
            custom_comm.data_cmd_id=0x0200;
            custom_comm.sender_ID=get_robot_id();//发送者ID，机器人对应ID
            custom_comm.receiver_ID=101;//接收者ID，操作手客户端ID
            {
                char toSend[] = "helloworld";

                if(rS==3){
                    // custom_comm.data.data[0]='a';
                    toSend[0] = 'g';
                }
                else{
                    //     custom_comm.data.data[0]='b';
                    toSend[0]='h';
                }
                // custom_comm.data.data[0]='t';
                // custom_comm.data.data[1]='r';
                memcpy(&custom_comm.data.data, toSend, sizeof(toSend));
            }
        }
        referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_comm,sizeof(custom_comm),&referee);

    }
    else {
        printf("Not writable!\n"); // usually it is never not writable
    }


    // loop++;
    // if(loop>100){loop=0;}
    // While loop interval
    //     ThisThread::sleep_for(50ms);
    // }
}

static void remoteRead(){ // for threadless
    myremote.remoteUpdate();
    lX = myremote.getStickData(LEFTJOYX,0,1000);
    lY = myremote.getStickData(LEFTJOYY,0,1000);
    rX = myremote.getStickData(RIGHTJOYX,0,1000);
    rY = myremote.getStickData(RIGHTJOYY,0,1000);
    Wh = myremote.getStickData(WHEEL,0,1000);
    lS = myremote.getSwitchData(LSWITCH);
    rS = myremote.getSwitchData(RSWITCH);
    if(lX > 1000 || lX < -1000)
        lX = 0;
    if(rX > 1000 || rX < -1000)
        rX = 0;
    if(lY > 1000 || lY < -1000)
        lY = 0;
    if(rY > 1000 || rY < -1000)
        rY = 0;
}

static void remotePrint(){
    // for (int i = 0; i < 7; i++)
    //     printf("%d\t", dats[i]);
    printf("%d\t%d\t%d\t%d\t%d\t%d\t",lX,lY,rX,rY,lS,rS);
    printf("\n");
}


//CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

//Thread threadingRemote(osPriorityNormal);
//Thread threadingReferee(osPriorityLow);
//Thread threadingPrint(osPriorityBelowNormal);
//Thread threadingLogger(osPriorityLow);


#endif //TR_EMBEDDED_MAIN_H
