#include "ref_serial.h"

void Referee::ui_graph_characters(int operation_type, string str, int x, int y, char name){
    ext_student_interactive_header_data_character_t custom_character_draw;
    {            
        custom_character_draw.data_cmd_id=0x0110; //0104 for 7 diagrams, 0110 for drawing text		
        custom_character_draw.sender_ID=get_robot_id();//发送者ID，机器人对应ID
        custom_character_draw.receiver_ID=get_robot_id()+0x0100;//接收者ID，操作手客户端ID
        {
            for(int i=0; i<str.length(); i++){
                custom_character_draw.graphic_custom.data[i]=str[i];
            }
            for(int i=str.length(); i<30; i++){ // have to fill in the rest of the char array
                custom_character_draw.graphic_custom.data[i]=' ';
            }

            custom_character_draw.graphic_custom.grapic_data_struct.graphic_name[0] = name;//图形名
            custom_character_draw.graphic_custom.grapic_data_struct.operate_tpye=operation_type;//图形操作，0：空操作；1：增加；2：修改；3：删除；
            custom_character_draw.graphic_custom.grapic_data_struct.graphic_tpye=7;//图形类型，0为直线，其他的查看用户手册
            custom_character_draw.graphic_custom.grapic_data_struct.layer=5;//图层数
            custom_character_draw.graphic_custom.grapic_data_struct.color=1;//颜色
            custom_character_draw.graphic_custom.grapic_data_struct.start_angle=20;
            custom_character_draw.graphic_custom.grapic_data_struct.end_angle=10;
            custom_character_draw.graphic_custom.grapic_data_struct.width=2;
            custom_character_draw.graphic_custom.grapic_data_struct.start_x = x;
            custom_character_draw.graphic_custom.grapic_data_struct.start_y = y;
        }
    }
    referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_character_draw,sizeof(custom_character_draw));
}

void Referee::ui_delete_layer(int layer){
    ext_student_interactive_header_data_delete_t custom_delete_draw;
    {
        custom_delete_draw.data_cmd_id=0x0100;		
        custom_delete_draw.sender_ID=get_robot_id();//发送者ID，机器人对应ID
        custom_delete_draw.receiver_ID=get_robot_id()+0x0100;//接收者ID，操作手客户端ID
        {
            custom_delete_draw.graphic_custom.operate_tpye = 1; // 1=delete a layer, 2=delete all
            custom_delete_draw.graphic_custom.layer = layer;
        }
    }
    referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_delete_draw,sizeof(custom_delete_draw));
}