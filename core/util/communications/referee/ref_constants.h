#include "mbed.h"


// Screen resolution of your Robomaster Client
#define SCREEN_LENGTH 1920
#define SCREEN_WIDTH 1080


// Writing data stuff
#define MAX_SIZE          128    //Maximum length of uploaded data
#define frameheader_len  5       //Frame header Length
#define cmd_len          2       //Command Code Length
#define crc_len          2       //CRC16 checksum (that's the literal translation idfk)

#define send_max_len 300 //200


// -------------------------------------
// From South China University of Technology  ----------------------------------
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.h/RM_JudgeSystem.h
// 2021 section


//Corresponding Communication Protocol Format   frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
#define       LEN_HEADER        5/*frame_header*/
#define       LEN_CMDID         2/*cmd_id*/
#define       LEN_TAIL          2/*frame_tail*/

//起始字节，协议固定为0xA5
#define      JUDGE_FRAME_HEADER            0xA5
#define      JUDGESYSTEM_PACKSIZE 		    389u		//Referee System Package Size(354+35)

//红蓝方
#define BLUE 0
#define RED  1

//信息传输
#define TRUE 1
#define FALSE 0

/***********************************命令码ID*************************************/

#define       Judge_Game_StatusData              0x0001 //11
#define       Judge_Game_ResultData              0x0002 //1
#define       Judge_Robot_HP                     0x0003 //32
#define       Judge_Event_Data                   0x0101 //4
#define       Judge_Supply_Station               0x0102 //3
#define       Judge_Referee_Warning              0x0104 //3
#define       Judge_Dart_Countdown               0x0105 //3
#define       Judge_Robot_State                  0x0201 //13
#define       Judge_Power_Heat                   0x0202 //16
#define       Judge_Robot_Position               0x0203 //16
#define       Judge_Robot_Buff                   0x0204 //7
#define       Judge_Aerial_Energy                0x0205 
#define       Judge_Injury_State                 0x0206 //1
#define       Judge_RealTime_Shoot               0x0207 //7
#define       Judge_Remaining_Rounds             0x0208 //6
#define       Judge_Robot_RFID                   0x0209 //4
#define       Judge_Dart_Client                  0x020A //6
#define       Judge_Robot_Communicate            0x0301 //127
#define       Judge_User_Defined                 0x0302 //30
#define       Judge_Map_Interaction              0x0303 //15
#define       Judge_KeyMouse_Message             0x0304 //12
#define       Judge_Client_Map                   0x0305 //24

/***************************DATA_Length*************************/

#define       JUDGE_EXTRA						9
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define       JudgeLength_Game_StatusData        20
#define       JudgeLength_Game_ResultData        10
#define       JudgeLength_Robot_HP               41
#define       JudgeLength_Event_Data             13
#define       JudgeLength_Supply_Station         12
#define       JudgeLength_Referee_Warning        12
#define       JudgeLength_Dart_Countdown         12
#define       JudgeLength_Robot_State            22
#define       JudgeLength_Power_Heat             25
#define       JudgeLength_Robot_Position         25
#define       JudgeLength_Robot_Buff             15
#define       JudgeLength_Aerial_Energy          11
#define       JudgeLength_Injury_State           10
#define       JudgeLength_RealTime_Shoot         16
#define       JudgeLength_Remaining_Rounds       15
#define       JudgeLength_Robot_RFID             13
#define       JudgeLength_Dart_Client            21
#define       JudgeLength_Robot_Commute          35
#define       JudgeLength_Robot_Map              26


// needed for anonymous structs defined below
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"


// -------------------------------------
// BELOW: from SCUT code line 733-1102

/* 自定义帧头 */
typedef struct //__packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;

}xFrameHeader;

/* ID: 0x0001    Byte: 11     Match Status Data */
typedef struct
{
    union {
        uint8_t dataBuff[11];
        struct
        {
            uint8_t game_type : 4;             //Competition Type
            uint8_t game_progress : 4;         //Current stage of the competition
            uint16_t stage_remain_time;        //Time remaining in current phase in seconds
        };
    }data;
    uint8_t infoUpdateFlag;

}ext_map_command_t;

/* ID: 0x0001    Byte: 11     Match Status Data */
typedef struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
 uint8_t infoUpdateFlag;
}game_status_t;

/* ID: 0x0002    Byte: 1       Match Results Data */
typedef struct
{
 uint8_t winner;
 uint8_t InfoUpdataFlag;
}game_result_t;

/* ID: 0x0003     Byte: 32     Competition Robot Health Data */
typedef struct
{
    union
    {
        uint8_t dataBuff[32];
        struct __packed
        {
            uint16_t red_1_robot_HP;//Red 1 Hero Robot HP: 0 for off-field and penalized units
            uint16_t red_2_robot_HP;//Red 2 Engineering Robot Health Points
            uint16_t red_3_robot_HP;//Red 3 Infantry Robot Health Points
            uint16_t red_4_robot_HP;//Red 4 ^
            uint16_t red_5_robot_HP;//Red 5 ^
            uint16_t red_7_robot_HP;//Red 7 ^
            uint16_t red_outpost_HP;//Red Outpost HP
            uint16_t red_base_HP;//Red Base HP
            uint16_t blue_1_robot_HP;
            uint16_t blue_2_robot_HP;
            uint16_t blue_3_robot_HP;
            uint16_t blue_4_robot_HP;
            uint16_t blue_5_robot_HP;
            uint16_t blue_7_robot_HP;
            uint16_t blue_outpost_HP;
            uint16_t blue_base_HP;
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_game_robot_HP_t;

/* ID: 0x0005      Byte: 11      AI Challenge Bonuses and Penalties */
typedef struct
{
    union
    {
        uint8_t dataBuff[11];
        struct __packed
        {
            uint8_t F1_zone_status:1;
            uint8_t F1_zone_buff_debuff_status:3;
            uint8_t F2_zone_status:1;
            uint8_t F2_zone_buff_debuff_status:3;
            uint8_t F3_zone_status:1;
            uint8_t F3_zone_buff_debuff_status:3;
            uint8_t F4_zone_status:1;
            uint8_t F4_zone_buff_debuff_status:3;
            uint8_t F5_zone_status:1;
            uint8_t F5_zone_buff_debuff_status:3;
            uint8_t F6_zone_status:1;
            uint8_t F6_zone_buff_debuff_status:3;
            uint16_t red1_bullet_left;
            uint16_t red2_bullet_left;
            uint16_t blue1_bullet_left;
            uint16_t blue2_bullet_left;
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101     Byte: 4       Venue Event Data */
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        struct __packed
        {
            uint32_t event_type;
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_event_data_t;

/* ID: 0x0102     Byte: 4       Field Supply Station Action Indicator Data; Maybe check documents */
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        struct __packed
        {
            uint8_t supply_projectile_id;  //Supply Station Entrance ID
            uint8_t supply_robot_id;       //Reloading Robot ID
            uint8_t supply_projectile_step;//Ejection port open/closed status
            uint8_t supply_projectile_num; //Number of Reloads
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_supply_projectile_action_t;

/* ID: 0X0104          Byte: 2       Referee Warning Data */
typedef struct  
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
 uint8_t InfoUpdataFlag;
}referee_warning_t;

/* ID: 0x0105          Byte: 1       飞镖发射口倒计时  */
typedef struct
    {
    union
    {
        uint8_t dataBuff[1];
        struct __packed
        {
            uint8_t dart_remaining_time;//15s 倒计时
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_dart_remaining_time_t;

/* ID: 0X0201          Byte: 27      Robot Status Data */
typedef struct
{
    union
    {
        uint8_t dataBuff[27];
        struct __packed
        {
            uint8_t robot_id;

			uint8_t robot_level;
			uint16_t current_HP; // duh
			uint16_t maximum_HP;// 
			uint16_t shooter_barrel_cooling_value; //Robot's 17mm cooling rate per second
			uint16_t shooter_barrel_heat_limit;// Robot's heat limit
			uint16_t chassis_power_limit;  //Robot 1 17mm Muzzle Velocity Cap Unit: m/s
			uint8_t power_management_output;

		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_status_t;

typedef struct __packed
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t current_HP;
 uint16_t maximum_HP;
 uint16_t shooter_barrel_cooling_value;
 uint16_t shooter_barrel_heat_limit;
 uint16_t chassis_power_limit;
 uint8_t power_management_output;
 uint8_t InfoUpdataFlag;
}robot_status_t;

/* ID: 0X0202          Byte: 16      Real-time power and heat data */
typedef struct  
{
 uint16_t reserved1; //REMOVED chassis_voltage
 uint16_t reserved2; //REMOVED chassis_current
 float reserved3; //REMOVED chassis_power
 uint16_t buffer_energy;
 uint16_t shooter_17mm_1_barrel_heat;
 uint16_t shooter_17mm_2_barrel_heat;
 uint16_t shooter_42mm_barrel_heat;
 uint8_t InfoUpdataFlag;
}power_heat_data_t;

/* ID: 0X0203          Byte: 16      Robot Position Data  */
typedef struct  
{
 float x;
 float y;
 float angle;
 uint8_t InfoUpdataFlag;
}robot_pos_t;

/* ID: 0X0204          Byte: 1       Robot Gain Data */
typedef struct
{
    union
    {
        uint8_t dataBuff[1];
        struct __packed
        {
            uint8_t power_rune_buff;/*bit 0：Health Regeneration Status
                                bit 1：Muzzle Heat Dissipation Status (literal translation was dissipation acceleration)
                                bit 2：Robot Defense Bonus
                                bit 3：Robot Attack Bonus*/
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_buff_t;

typedef struct __packed
{
 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
}buff_t;

/* ID: 0X0205          Byte: 2       空中机器人能量状态数据 */
typedef struct
{
    union
    {
        uint8_t dataBuff[2];
        struct __packed
        {
            uint8_t attack_time;//可攻击时间 单位 s。30s 递减至 0
        };
    }data;
    uint8_t InfoUpdataFlag;
}aerial_robot_energy_t;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct __packed
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
 uint8_t InfoUpdataFlag;
}hurt_data_t;

/* ID: 0X0207          Byte: 7       Real-time Shooting Data */
typedef struct  
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
 uint8_t InfoUpdataFlag;
}shoot_data_t;

/* ID: 0X0208          Byte: 6       Remaining Bullets to fire */
typedef struct  
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
 uint8_t InfoUpdataFlag;
}projectile_allowance_t;

/* ID: 0X0209          Byte: 4       Robot RFID Status */
/*bit 0：Base Gain Point RFID Status；
bit 1：High-Ground Gain Point RFID Status；
bit 2：Energy Mechanism Activation Point RFID Status；
bit 3：Flying ramp gain point RFID Status (the translator was a lil confused about the first part 飞坡增益点 RFID 状态)；
bit 4：Outpost Gain Point RFID Status；
bit 5：Resource Island Bonus Points RFID Status；
bit 6：Health Point Bonus Point RFID Status；
bit 7：Engineering Robot Blood Refill Card RFID Status；
The RFID status does not fully represent the corresponding bonus or penalty state. 
For example, enemy-controlled high ground bonus points cannot grant the corresponding bonus effects.*/
typedef struct
{
    union
    {
        uint8_t dataBuff[4];
        struct __packed
        {
            uint32_t rfid_status;
        };
    }data;
    uint8_t InfoUpdataFlag;
}ext_rfid_status_t;



// ----------------------- Line 1163

/* ID: 0X0301          Byte: n       Interaction Data Between Robots */
typedef  struct  
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

/* data */
typedef  struct __packed
{
    uint8_t data[20];//Data segment n is less than 113
}robot_interactive_data_t;

/* 
	Client
Client Custom Data: cmd_id:0x0301. Content ID:0x0100 0x0101 0x0102 0x0103 0x0110 0x0104
Transmission Frequency: Upper limit 10Hz

*/

/* Client Deletes Graphics Communication Between Bots：0x0301 */
typedef  struct  
{
    uint8_t operate_tpye;
    uint8_t layer;
}ext_client_custom_graphic_delete_t;

// my copy
typedef  struct __packed
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    ext_client_custom_graphic_delete_t graphic_custom; // ADDED
}ext_student_interactive_header_data_delete_t;

/* Graphical Data */
typedef  struct  
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
}graphic_data_struct_t;

typedef  struct __packed
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    int32_t data;
}ClientData_struct_t;

/* Client-side graphics rendering Robot-to-robot communication：0x0301 */
typedef  struct  
{
    graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

/* Client-side rendering of two graphics Robot-to-robot communication：0x0301 */
typedef  struct  
{
    graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

/* Client draws five shapes Robot-to-robot communication：0x0301 */
typedef  struct  
{
    graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

/* Client draws seven shapes Robot-to-robot communication：0x0301 */
typedef  struct  
{
    graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

/* Client-side character rendering Robot-to-robot communication：0x0301 */
typedef  struct  
{
    graphic_data_struct_t grapic_data_struct;
    char data[30];
}ext_client_custom_character_t;


// MY ADDED STRUCTS ------------------

/* ID: 0X0301          Byte: n       Interaction Data Between Robots */
typedef  struct  
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    ext_client_custom_graphic_seven_t graphic_custom; // ADDED
}ext_student_interactive_header_data_graphic_t;

// my copy
typedef  struct __packed
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    ext_client_custom_character_t graphic_custom; // ADDED
}ext_student_interactive_header_data_character_t;

// my copy
typedef  struct __packed
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
    robot_interactive_data_t data; // ADDED
}ext_student_interactive_header_data_robot_comm_t;

// ------------------- Line 1330

// /*Robot Interaction Information：0x0301*/
// typedef __packed struct
// {
// 	xFrameHeader   							txFrameHeader;//Frame Header
// 	uint16_t								CmdID;//Command code
// 	ext_student_interactive_header_data_t   dataFrameHeader;//Data Segment Header Structure
// 	robot_interactive_data_t  	 			interactData;//Data Segment
// 	uint16_t		 						FrameTail;//Frame Tail
// }ext_CommunatianData_t;

// //Frame Header  Command Code   Data Segment Header Structure   Data Segment   帧尾

// /*客户端结构体*/
// //上传客户端
// typedef __packed struct
// {
// 	xFrameHeader   							txFrameHeader;//Frame header
// 	uint16_t		 						CmdID;//Command code
// 	ext_student_interactive_header_data_t   dataFrameHeader;//Data Segment Header Structure
// 	graphic_data_struct_t cilentData[7];//Data Segment
// 	uint16_t		 						FrameTail;//Frame Tal
// }ext_SendClientData_t;

// typedef __packed struct
// {
// 	xFrameHeader   							txFrameHeader;//帧头
// 	uint16_t		 						CmdID;//命令码
// 	ext_student_interactive_header_data_t   dataFrameHeader;//Data Segment Header Structure
// 	graphic_data_struct_t cilentData[5];//Data Segment
// 	uint16_t		 						FrameTail;//Frame Tail
// }ext_ShowCrossHair_t;

#pragma GCC diagnostic pop