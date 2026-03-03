#include "ThisThread.h"
#include "ref_serial.h"


Referee::Referee(PinName pin_tx, PinName pin_rx) : ref(pin_tx, pin_rx, 115200) 
{
    memset(JudgeSystem_rxBuff, 0, JUDGESYSTEM_PACKSIZE);

    this->readThread_.start(callback(this, &Referee::readThread));
    this->writeThread_.start(callback(this, &Referee::writeThread));
}


bool Referee::readable()
{
    return ref.readable();
}


void Referee::read()
{
    if(ref.readable())
    {
        int rad = JudgeSystem_USART_Receive_DMA();
        mutex_read_.lock();
        memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
        mutex_read_.unlock();
        Judge_GetMessage(rad);

        if(enablePrintRefData){
            string output = "robot id: %s  ";

            if(is_red_or_blue()){
                output += "RED  ";
            }
            else{
                output += "BLUE  ";
            }
            output += "robot hp: %d  max hp: %d  angle: %d  power: %d  current: %d  volt: %d \n";
        }
    }
    else{
        if(enablePrintRefData){
            printf("REFEREE - Not readable!\n");
        }
    }
}


// TODO FIGURE OUT HOW THIS WORKS

/* Figured out how it works, All praise https://terra-1-g.djicdn.com/b2a076471c6c4b72b574a977334d3e05/RM2025/RoboMaster%20Referee%20System%20Serial%20Port%20Protocol%20V1.9.0%EF%BC%8820250704%EF%BC%89.pdf
Page 28

The write function is basically just a fool-proof way of sending a data packet to the ref system, with certain bits in the bytes corresponding to the data
It's a lot to write out, just go to the above link if you want the details, but in a nut shell we're sending
Graphic name (some 3 bit name for it)
Graphic function (0 add, 1 add, 2: modify, 3: delete)
Graphic type : 0 for straight line, 1: Rectangle, 2: Circle, 3: Ellipse, 4: Arc, 5: floating number, 6: integer, 7: character
Number of Layers (0-9 is valid)
Color (0: Team Color (Red/Blue), 1: Yellow, 2: Green, 3: Orange 4: Purplish Red, 5: Pink, 6: Cyan, 7: Black, 8: white)

There's more data depending on the type of graphic, just go to the sheet for that*/

void Referee::write()
{
    if(ref.writable()){
        
        // For graphing diagrams ----------------------------
        ext_student_interactive_header_data_graphic_t custom_graphic_draw;	//自定义图像 // Custom image
        {
            custom_graphic_draw.data_cmd_id=0x0104;//Draw seven figures (Content ID, refer to the Referee System Manual) //0104 for 7 diagrams, 0110 for drawing text
            custom_graphic_draw.sender_ID=get_robot_id();//Sender ID,  Bot ID
            custom_graphic_draw.receiver_ID=get_robot_id()+0x0100;//Recipient ID, Operator Client ID
            {
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = 7;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = 8;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = 9;//Graphic Name (No I don't know what they mean by that (图形名))
                //The top three bytes represent the graphic name, used for graphic indexing, and can be defined by the user.
                
                //Graphical operations: 0: No operation; 1: Add; 2: Modify; 3: Delete;
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].operate_tpye=1; // This typo is intentional, tbf you'd probably make typos if you programmed in chinese 
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye=0; //Graphic type
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].layer=9;//Number of Layers
                custom_graphic_draw.graphic_custom.grapic_data_struct[0].color=1;//Color (颜色) 
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
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].operate_tpye=1;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].graphic_tpye=1;
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].layer=9;//
                custom_graphic_draw.graphic_custom.grapic_data_struct[1].color=1;//
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

        ui_delete_layer(5);
        referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_graphic_draw,sizeof(custom_graphic_draw));

        // string powerStr = "power: " + to_string((int)power_heat_data.chassis_power);
        // ui_graph_characters(&ref, 1, powerStr, SCREEN_LENGTH/2 +100, SCREEN_WIDTH/2 +100, 99);

        // // string angleStr = "angle: " + to_string((int)ext_game_robot_pos.data.yaw);
        // ui_graph_characters(&ref, 1, angleStr, SCREEN_LENGTH/2 +100, SCREEN_WIDTH/2 +150, 10);

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
        referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_comm,sizeof(custom_comm), &ref);
        */
    }
    else {
        if(enablePrintRefData){
            printf("Not writable!\n"); // usually it is never not writable
        }
    }
}


void Referee::readThread()
{
    while(1)
    {
        // Un-nested read since all it does is call another fn
        // read();
        if(ref.readable())
        {
            int rad = JudgeSystem_USART_Receive_DMA();
            mutex_read_.lock();
            memcpy(JudgeSystem_rxBuff, JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
            mutex_read_.unlock();
            Judge_GetMessage(rad);

            if(enablePrintRefData){
                string output = "robot id: %s  ";

                if(is_red_or_blue()){
                    output += "RED  ";
                }
                else{
                    output += "BLUE  ";
                }
                output += "robot hp: %d  max hp: %d  angle: %d  power: %d  current: %d  volt: %d \n";
            }
        }
        else{
            if(enablePrintRefData){
                printf("REFEREE - Not readable!\n");
            }
        }

        ThisThread::yield();
    }
}


void Referee::writeThread()
{
    while(1)
    {
        // TODO: figure out write then un-nest it
        write();
        ThisThread::yield();
    }
}


uint8_t Referee::get_robot_id()
{
    return robot_status.robot_id;
}


uint8_t Referee::get_remain_hp()
{
    return robot_status.current_HP;
}


uint8_t Referee::get_game_progress()
{
    return game_status.game_progress;
}


/**
  * @brief  Determine whether you are on the red or blue team
  * @param  void
  * @retval RED   BLUE
  * @attention  Data packaging, then transmission to the judging system via serial port upon completion.
  */
bool Referee::is_red_or_blue()
{
    Judge_Self_ID = robot_status.robot_id; //Read the current robot ID

    if (robot_status.robot_id > 10) //Remember from the constants doc, 1-10 are red, 11-20 are blue
    {
        return 0; //蓝方 (blue)
    }
    else
    {
        return 1; //红方 (red)
    }
}