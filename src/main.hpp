#include "mbed.h"
#include "../util/motor/pwmmotor.cpp"
#include "../util/communications/include/DJIRemote.hpp"
#include "../util/communications/newCANHandler.hpp"
#include "../util/algorithms/pid.hpp"
//#include "subsystems/ChassisSubsystem.hpp"
//#include "subsystems/TurretSubsystem.hpp"
#include "../util/communications/djiremoteuart.hpp"
#include "../util/helperFunctions.hpp"
#include "../util/communications/SerialCommunication.hpp"
#include "../util/motor/CANMotor.hpp"
#include "../util/communications/ref_serial.cpp"



//#include "robots/include/infantry.hpp"

//屏幕分辨率1920x1080
// #define SCREEN_WIDTH 1080
// #define SCREEN_LENGTH 1920
#define SCREEN_WIDTH 720
#define SCREEN_LENGTH 1280

static DJIRemote myremote(PA_0, PA_1);
BufferedSerial referee(PC_10, PC_11, 115200); // Nucleo board: top left male pins. 

NewCANHandler canHandler1(PA_11,PA_12);
NewCANHandler canHandler2(PB_12,PB_13);

static int lX = 0;
static int lY = 0;
static int rX = 0;
static int rY = 0;
static int Wh = 0;
static int lS = 0;
static int rS = 0;

static void refereeThread(){
    ext_student_interactive_header_data_t custom_grapic_draw;			//自定义图像绘制
    // ext_client_custom_graphic_seven_t custom_graphic;	//自定义图像

//初始化图形数据变量
	//自定义图形绘制
	{
    
		custom_grapic_draw.data_cmd_id=0x0110;//绘制七个图形（内容ID，查询裁判系统手册）//0104 for 7 diagrams, 0110 for drawing text
		

			custom_grapic_draw.sender_ID=3;//发送者ID，机器人对应ID，此处为蓝方英雄
			custom_grapic_draw.receiver_ID=0x0103;//接收者ID，操作手客户端ID，此处为蓝方英雄操作手客户端
		//自定义图像数据
		{
            // For drawing text ------------------

            char c[] ="hello";
            for(int i=0; i<sizeof(c); i++){
                custom_grapic_draw.graphic_custom.data[i]=c[i];
            }
            for(int i=sizeof(c); i<30; i++){ // have to fill in the rest of the char array
                custom_grapic_draw.graphic_custom.data[i]=' ';
            }

            custom_grapic_draw.graphic_custom.grapic_data_struct.graphic_name[0] = 97;
            custom_grapic_draw.graphic_custom.grapic_data_struct.graphic_name[1] = 97;
            custom_grapic_draw.graphic_custom.grapic_data_struct.graphic_name[2] = 0;//图形名
            //上面三个字节代表的是图形名，用于图形索引，可自行定义
            custom_grapic_draw.graphic_custom.grapic_data_struct.operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
            custom_grapic_draw.graphic_custom.grapic_data_struct.graphic_tpye=7;//图形类型，0为直线，其他的查看用户手册
            custom_grapic_draw.graphic_custom.grapic_data_struct.layer=1;//图层数
            custom_grapic_draw.graphic_custom.grapic_data_struct.color=1;//颜色
            custom_grapic_draw.graphic_custom.grapic_data_struct.start_angle=30;
            custom_grapic_draw.graphic_custom.grapic_data_struct.end_angle=100;
            custom_grapic_draw.graphic_custom.grapic_data_struct.width=3;
            custom_grapic_draw.graphic_custom.grapic_data_struct.start_x=SCREEN_LENGTH/2 +100;
            custom_grapic_draw.graphic_custom.grapic_data_struct.start_y=SCREEN_WIDTH/2;


            // For graphing diagrams ----------------------------

            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;//图形名
            // //上面三个字节代表的是图形名，用于图形索引，可自行定义
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer=1;//图层数
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].color=1;//颜色
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle=0;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle=0;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].width=2;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x=SCREEN_LENGTH/2 +200;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y=SCREEN_WIDTH/2 +200;
            // custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius=1;
        }
    }

    int loop=0;
    while(1){
        if(referee.readable()){
            JudgeSystem_USART_Receive_DMA(&referee);
            Judge_GetMessage(JUDGESYSTEM_PACKSIZE);
            
        //     if(loop % 10==0){ // print out only every 10 iterations
        //         string id="robot id: " + to_string(get_robot_id()) + "  ";
        //         //string hp="robot hp: " + to_string(get_remain_hp() ) +"  ";

        //         cout << id;
        //         if(is_red_or_blue()){
        //             cout<<"RED  ";
        //         }
        //         else{
        //             cout<<"BLUE  ";
        //         }
        //         printf("robot hp: %d  ", ext_game_robot_state.data.remain_HP);
        //         printf("max hp: %d  ", ext_game_robot_state.data.max_HP);
        //         printf("angle: %f  ", ext_game_robot_pos.data.yaw);
        //         // cout << "angle: "<< ext_game_robot_pos.data.yaw;
                
        //         printf("power: %f  ", ext_power_heat_data.data.chassis_power);
        //         printf("current: %d  ", ext_power_heat_data.data.chassis_current);
        //         printf("volt: %d \n", ext_power_heat_data.data.chassis_volt);

        //         // pc.write(&id, id.length());
        //         // pc.write(&hp, hp.length());
        //     }
            
        // }
        // else{
        //     if(loop % 10==0){ // print out only every 10 iterations
        //         printf("REFEREE - Not readable!\n");
        //     }
        }

        if(referee.writable()){
            // RobotStatus_LEDYellow(&referee);
            // RobotStatus_LEDYellow(&pc);
            // Show_CrossHair(&referee);
            referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_grapic_draw,sizeof(custom_grapic_draw),&referee);
            
            // printf("write!");
            // pc.write("hello",5);
        }
        else {
            printf("Not writable!\n");
        }


        loop++;
        if(loop>100){loop=0;}
        // While loop interval
        ThisThread::sleep_for(50ms);
    }
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
