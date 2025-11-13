#include "ref_operations.h"

void refereeThread(BufferedSerial* referee){

    // int loop=0;
    // while(1){
    if(referee->readable()){
        int rad = JudgeSystem_USART_Receive_DMA(referee);
        Judge_GetMessage(rad);

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
            // printf("angle: %d  \n", (int)ext_game_robot_pos.data.yaw);
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

    if(referee->writable()){

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
            {
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].graphic_name[0] = 3;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].operate_tpye=1;//图形操作，0：空操作；1：增加；2：修改；3：删除；
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].graphic_tpye=0;//图形类型，0为直线，其他的查看用户手册
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].layer=9;//图层数
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].color=1;//颜色
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].start_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].end_angle=0;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].width=2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].start_x=SCREEN_LENGTH/2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].start_y=SCREEN_WIDTH/2 +100;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].end_x=SCREEN_LENGTH/2;
                custom_graphic_draw.graphic_custom.grapic_data_struct[3].end_y=SCREEN_WIDTH/2 -250;
            }
        }

        ui_delete_layer(referee, 5);
        referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_graphic_draw,sizeof(custom_graphic_draw),referee);

        // string powerStr = "power: " + to_string((int)power_heat_data.chassis_power);
        // ui_graph_characters(referee, 1, powerStr, SCREEN_LENGTH/2 +100, SCREEN_WIDTH/2 +100, 99);

        // // string angleStr = "angle: " + to_string((int)ext_game_robot_pos.data.yaw);
        // ui_graph_characters(referee, 1, angleStr, SCREEN_LENGTH/2 +100, SCREEN_WIDTH/2 +150, 10);

        /* Robot communication to be worked on in the future */
        /*
        ext_student_interactive_header_data_robot_comm_t custom_comm;
        {
            custom_comm.data_cmd_id=0x0200;
            custom_comm.sender_ID=get_robot_id();//发送者ID，机器人对应ID
            custom_comm.receiver_ID=101;//接收者ID，操作手客户端ID
            {
                char toSend[] = "helloworld";

                if(rS==Remote::SwitchState::UP){
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
        referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_comm,sizeof(custom_comm),referee);
        */
    }
    else {
        if(enablePrintRefData){
            printf("Not writable!\n"); // usually it is never not writable
        }
    }


    // loop++;
    // if(loop>100){loop=0;}
    // While loop interval
    //     ThisThread::sleep_for(50ms);
    // }
}
