#ifndef _REFEREE_HPP
#define _REFEREE_HPP

#include "mbed.h"
#include "crc.cpp"

class Referee {
public:
    Referee(PinName pin_tx, PinName pin_rx);
    void clearRxBuffer();
    void read();
    //BufferedSerial getRef();
    bool readable();

private:
    BufferedSerial ref;
};


// -------------------------------------
// From South China University of Technology 华南理工大学广州学院-野狼战队-步兵代码 ----------------------------------
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.h/RM_JudgeSystem.h
// 2021 section


//对应通信协议格式   frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16,整包校验)
#define       LEN_HEADER        5/*frame_header*/
#define       LEN_CMDID         2/*cmd_id*/
#define       LEN_TAIL          2/*frame_tail*/

//起始字节，协议固定为0xA5
#define      JUDGE_FRAME_HEADER            0xA5
#define      JUDGESYSTEM_PACKSIZE 		    389u		//裁判系统包大小(354+35)

//红蓝方
#define BLUE 0
#define RED  1

//信息传输
#define TRUE 1
#define FALSE 0

/***********************************命令码ID*************************************/

#define       Judge_Game_StatusData              0x0001 
#define       Judge_Game_ResultData              0x0002 
#define       Judge_Robot_HP                     0x0003 
#define       Judge_Dart_Launch                  0x0004
#define       Judge_AI_ChallengeBuff             0x0005
#define       Judge_Event_Data                   0x0101
#define       Judge_Supply_Station               0x0102
//#define       Judge_Request_Recharge             0x0103(对抗赛未开放)
#define       Judge_Referee_Warning              0x0104
#define       Judge_Dart_Countdown               0x0105
#define       Judge_Robot_State                  0x0201
#define       Judge_Power_Heat                   0x0202
#define       Judge_Robot_Position               0x0203
#define       Judge_Robot_Buff                   0x0204
#define       Judge_Aerial_Energy                0x0205
#define       Judge_Injury_State                 0x0206
#define       Judge_RealTime_Shoot               0x0207
#define       Judge_Remaining_Rounds             0x0208
#define       Judge_Robot_RFID                   0x0209
#define       Judge_Dart_Client                  0x020A
#define       Judge_Robot_Communicate            0x0301
#define       Judge_User_Defined                 0x0302
#define       Judge_Map_Interaction              0x0303
#define       Judge_KeyMouse_Message             0x0304
#define       Judge_Client_Map                   0x0305

/***************************DATA_Length*************************/
/*Calculation:frame_header(5-byte)+cmd_id(2-byte)+data(n-byte)+frame_tail(2-byte,CRC16)*/

#define       JudgeLength_Game_StatusData        20
#define       JudgeLength_Game_ResultData        10
#define       JudgeLength_Robot_HP               41
#define       JudgeLength_Dart_Launch            12
#define       JudgeLength_AI_ChallengeBuff       20
#define       JudgeLength_Event_Data             13
#define       JudgeLength_Supply_Station         13
//#define       JudgeLength_Request_Recharge       11(对抗赛未开放)
#define       JudgeLength_Referee_Warning        11
#define       JudgeLength_Dart_Countdown         10
#define       JudgeLength_Robot_State            36
#define       JudgeLength_Power_Heat             25
#define       JudgeLength_Robot_Position         25
#define       JudgeLength_Robot_Buff             10
#define       JudgeLength_Aerial_Energy          10
#define       JudgeLength_Injury_State           10
#define       JudgeLength_RealTime_Shoot         16
#define       JudgeLength_Remaining_Rounds       15
#define       JudgeLength_Robot_RFID             13
#define       JudgeLength_Dart_Client            21
#define       JudgeLength_Robot_Commute          35
#define       JudgeLength_Robot_Map              26

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

/* ID: 0x0001    Byte: 11     比赛状态数据 */
typedef struct
{
  union {
		uint8_t dataBuff[11];
		__packed struct {
			uint8_t game_type : 4;             //比赛类型
			uint8_t game_progress : 4;         //当前比赛阶段
			uint16_t stage_remain_time;        //当前阶段剩余时间  单位s
		};
	}data;
	uint8_t infoUpdateFlag;
}ext_game_status_t;

/* ID: 0x0002    Byte: 1       比赛结果数据 */
typedef struct
{
	union{
	  uint8_t dataBuff[1];
		__packed struct
		{
			uint8_t winner;//0平局 1红胜 2蓝胜
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_result_t;

/* ID: 0x0003     Byte: 32     比赛机器人血量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[32];
		__packed struct
		{
	     uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
       uint16_t red_2_robot_HP;//红 2 工程机器人血量
       uint16_t red_3_robot_HP;//红 3 步兵机器人血量
       uint16_t red_4_robot_HP;//红 4 步兵机器人血量
       uint16_t red_5_robot_HP;//红 5 步兵机器人血量
       uint16_t red_7_robot_HP;//红 7 步兵机器人血量
       uint16_t red_outpost_HP;//红方前哨战血量
       uint16_t red_base_HP;//红方基地血量
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

/* ID: 0x0004      Byte: 3    飞镖发射状态 */
typedef struct
{
	union
	{
		uint8_t dataBuff[3];
		__packed struct
		{
			uint8_t dart_belong;//发射飞镖的队伍：1：红方飞镖2：蓝方飞镖
			uint16_t stage_remaining_time;//发射时的剩余比赛时间，单位 s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_status_t;

/* ID: 0x0005      Byte: 11      人工智能挑战赛加成与惩罚 */
typedef struct
{
	union
	{
		uint8_t dataBuff[11];
		__packed struct
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

/* ID: 0x0101     Byte: 4       场地事件数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[4];
		__packed struct
		{
			uint32_t event_type;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_event_data_t;

/* ID: 0x0102     Byte: 4       场地补给站动作标识数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[4];
		__packed struct
		{
			uint8_t supply_projectile_id;  //补给站口 ID
			uint8_t supply_robot_id;       //补弹机器人 ID
			uint8_t supply_projectile_step;//出弹口开闭状态 
			uint8_t supply_projectile_num; //补弹数量
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_supply_projectile_action_t;

/* ID: 0X0104          Byte: 2       裁判警告数据 */
typedef struct
{
	union
	{
    uint8_t dataBuff[2];
	  __packed struct
	 {
		 uint8_t level;//警告等级：
		 uint8_t foul_robot_id; //犯规机器人 ID：判负时，机器人 ID 为 0 黄牌、红牌时，机器人 ID 为犯规机器人 ID
	 };
	}data;
	uint8_t InfoUpdataFlag;
}ext_referee_warning_t;

/* ID: 0x0105          Byte: 1       飞镖发射口倒计时  */
typedef struct
{
	union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
			uint8_t dart_remaining_time;//15s 倒计时
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_remaining_time_t;

/* ID: 0X0201          Byte: 27      机器人状态数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[27];
		__packed struct
		{
		  uint8_t robot_id;
      uint8_t robot_level;
      uint16_t remain_HP;//机器人剩余血量
      uint16_t max_HP;//机器人上限血量
      uint16_t shooter_id1_17mm_cooling_rate; //机器人 1 号 17mm 枪口每秒冷却值
      uint16_t shooter_id1_17mm_cooling_limit;//机器人 1 号 17mm 枪口热量上限
      uint16_t shooter_id1_17mm_speed_limit;  //机器人 1 号 17mm 枪口上限速度 单位 m/s
      uint16_t shooter_id2_17mm_cooling_rate;
      uint16_t shooter_id2_17mm_cooling_limit;
      uint16_t shooter_id2_17mm_speed_limit;
      uint16_t shooter_id1_42mm_cooling_rate;
      uint16_t shooter_id1_42mm_cooling_limit;
      uint16_t shooter_id1_42mm_speed_limit;
      uint16_t chassis_power_limit;           //机器人底盘功率限制上限
			/*主控电源输出情况：
       0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
       1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
       2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；*/
      uint8_t mains_power_gimbal_output : 1;
      uint8_t mains_power_chassis_output : 1;
      uint8_t mains_power_shooter_output : 1;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_status_t;

/* ID: 0X0202          Byte: 16      实时功率热量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[16];
		__packed struct
		{
			uint16_t chassis_volt; //底盘输出电压 单位 毫伏
      uint16_t chassis_current; //底盘输出电流 单位 毫安
      float chassis_power;//底盘输出功率 单位 W 瓦
      uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
      uint16_t shooter_id1_17mm_cooling_heat;//枪口热量
      uint16_t shooter_id2_17mm_cooling_heat;
      uint16_t shooter_id1_42mm_cooling_heat;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_power_heat_data_t;

/* ID: 0X0203          Byte: 16      机器人位置数据  */
typedef struct __packed
{
  union
	{
		uint8_t dataBuff[16];
		__packed struct
		{
		 float x;//位置 x 坐标
     float y;//位置 y 坐标
     float z;//位置 z 坐标
     float yaw;//位置枪口
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_pos_t;

/* ID: 0X0204          Byte: 1       机器人增益数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
			uint8_t power_rune_buff;/*bit 0：机器人血量补血状态
                                bit 1：枪口热量冷却加速
                                bit 2：机器人防御加成
                                bit 3：机器人攻击加成*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_buff_t;
/* ID: 0X0205          Byte: 2       空中机器人能量状态数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[2];
		__packed struct
		{
			uint8_t attack_time;//可攻击时间 单位 s。30s 递减至 0
		};
	}data;
	uint8_t InfoUpdataFlag;
}aerial_robot_energy_t;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
		  uint8_t armor_id : 4;/*bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
                             五个装甲片，其他血量变化类型，该变量数值为 0。*/
      uint8_t hurt_type : 4;/*bit 4-7：血量变化类型,0x0 装甲伤害扣血；
                              0x1 模块掉线扣血；
                              0x2 超射速扣血；
                              0x3 超枪口热量扣血；
                              0x4 超底盘功率扣血；
                              0x5 装甲撞击扣血*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_hurt_t;

/* ID: 0X0207          Byte: 7       实时射击数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[7];
		__packed struct
		{
		  uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
      uint8_t shooter_id;//发射机构 ID
      uint8_t bullet_freq;//子弹射频 单位 Hz
      float bullet_speed;//子弹射速 单位 m/s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_shoot_data_t;

/* ID: 0X0208          Byte: 6       子弹剩余发送数 */
typedef struct
{
  union
	{
		uint8_t dataBuff[6];
		__packed struct
		{
		  uint16_t bullet_remaining_num_17mm;//17mm 子弹剩余发射数目
      uint16_t bullet_remaining_num_42mm;//42mm 子弹剩余发射数目
			uint16_t coin_remaining_num;//剩余金币数量
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_bullet_remaining_t;

/* ID: 0X0209          Byte: 4       机器人RFID状态 */
/*bit 0：基地增益点 RFID 状态；
bit 1：高地增益点 RFID 状态；
bit 2：能量机关激活点 RFID 状态；
bit 3：飞坡增益点 RFID 状态；
bit 4：前哨岗增益点 RFID 状态；
bit 5：资源岛增益点 RFID 状态；
bit 6：补血点增益点 RFID 状态；
bit 7：工程机器人补血卡 RFID 状态；
RFID 状态不完全代表对应的增益或处罚状态，例如敌方已占领的高地增益点，不能
获取对应的增益效果。*/
typedef struct
{
	union
	{
		uint8_t dataBuff[4];
		__packed struct
		{
			uint32_t rfid_status;
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_rfid_status_t;

/* ID: 0x020A          Byte: 12      飞镖机器人客户端指令书 */
typedef struct
{
	union
	{
		uint8_t dataBuff[12];
		__packed struct
		{
		  uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
      uint8_t dart_attack_target;//飞镖的打击目标，默认为前哨站
      uint16_t target_change_time;//切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
      uint8_t first_dart_speed; //检测到的第一枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint8_t second_dart_speed;//检测到的第二枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint8_t third_dart_speed; //检测到的第三枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint8_t fourth_dart_speed;//检测到的第四枚飞镖速度，单位 0.1m/s/LSB, 未检测是为 0
      uint16_t last_dart_launch_time;//最近一次的发射飞镖的比赛剩余时间，单位秒，初始值为 0。
      uint16_t operate_launch_cmd_time;	//最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_dart_client_cmd_t;

// --------------- THE FOLLOWING ARE MOVED FROM BELOW

/* 图形数据 */
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
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11; 
} graphic_data_struct_t;

typedef  struct __packed
{
  graphic_data_struct_t grapic_data_struct[7];
}ext_client_custom_graphic_seven_t;

typedef  struct __packed
{
  graphic_data_struct_t grapic_data_struct;
  char data[30];
}ext_client_custom_character_t;

// ----------------------- Line 1163

/* ID: 0X0301          Byte: n       机器人间交互数据 */
typedef  struct __packed
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

/* data */
typedef  struct __packed
{
	uint8_t data[20];//数据段n小于113
}robot_interactive_data_t;

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0x0100   0x0101  0x0102  0x0103  0x0110  0x0104
	发送频率：上限 10Hz


*/

/* 客户端删除图形 机器人间通信：0x0301 */
typedef  struct __packed
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

// /* 图形数据 */
// typedef __packed struct
// { 
//   uint8_t graphic_name[3]; 
//   uint32_t operate_tpye:3; 
//   uint32_t graphic_tpye:3; 
//   uint32_t layer:4; 
//   uint32_t color:4; 
//   uint32_t start_angle:9;
//   uint32_t end_angle:9;
//   uint32_t width:10; 
//   uint32_t start_x:11; 
//   uint32_t start_y:11; 
//   uint32_t radius:10; 
//   uint32_t end_x:11; 
//   uint32_t end_y:11; 
// }graphic_data_struct_t;

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

/* 客户端绘制一个图形 机器人间通信：0x0301 */
typedef  struct __packed
{
  graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;

/* 客户端绘制二个图形 机器人间通信：0x0301 */
typedef  struct __packed
{
  graphic_data_struct_t grapic_data_struct[2];
}ext_client_custom_graphic_double_t;

/* 客户端绘制五个图形 机器人间通信：0x0301 */
typedef  struct __packed
{
  graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

/* 客户端绘制七个图形 机器人间通信：0x0301 */
// typedef  struct __packed
// {
//   graphic_data_struct_t grapic_data_struct[7];
// }ext_client_custom_graphic_seven_t;

/* 客户端绘制字符 机器人间通信：0x0301 */
// typedef  struct __packed
// {
//   graphic_data_struct_t grapic_data_struct;
//   char data[30];
// }ext_client_custom_character_t;

// ------------------- Line 1330

// /*机器人交互信息：0x0301*/
// typedef __packed struct
// {
// 	xFrameHeader   							txFrameHeader;//帧头
// 	uint16_t								CmdID;//命令码
// 	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
// 	robot_interactive_data_t  	 			interactData;//数据段
// 	uint16_t		 						FrameTail;//帧尾
// }ext_CommunatianData_t;

// //帧头  命令码   数据段头结构  数据段   帧尾

// /*客户端结构体*/
// //上传客户端
// typedef __packed struct
// {
// 	xFrameHeader   							txFrameHeader;//帧头
// 	uint16_t		 						CmdID;//命令码
// 	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
// 	graphic_data_struct_t cilentData[7];//数据段
// 	uint16_t		 						FrameTail;//帧尾
// }ext_SendClientData_t;

// typedef __packed struct
// {
// 	xFrameHeader   							txFrameHeader;//帧头
// 	uint16_t		 						CmdID;//命令码
// 	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
// 	graphic_data_struct_t cilentData[5];//数据段
// 	uint16_t		 						FrameTail;//帧尾
// }ext_ShowCrossHair_t;


// ------------------------------
// Extra stuff

void JudgeSystem_USART_Receive_DMA(BufferedSerial* b);
void Judge_GetMessage(uint16_t Data_Length);
void Judge_sendPC(BufferedSerial* b);
void RobotStatus_LEDYellow(BufferedSerial* b);

extern ext_game_status_t      ext_game_status;
// extern ext_game_result_t     ext_game_result;
// extern ext_game_robot_HP_t   ext_game_robot_HP;
// extern ext_dart_status_t     ext_dart_status;
// extern ext_ICRA_buff_debuff_zone_status_t    ext_ICRA_buff_debuff_zone_status;
// extern ext_event_data_t ext_even_data;
// extern ext_supply_projectile_action_t  ext_supply_projectile_action;
// extern ext_referee_warning_t         ext_referee_warning;
// extern ext_dart_remaining_time_t     ext_dart_remaining_time;
extern ext_game_robot_status_t  ext_game_robot_state;
extern ext_power_heat_data_t  ext_power_heat_data;
extern ext_game_robot_pos_t  ext_game_robot_pos;
// extern ext_buff_t Buff;
// extern aerial_robot_energy_t  aerial_robot_energy;
// extern ext_robot_hurt_t  ext_robot_hurt;
// extern ext_shoot_data_t  ext_shoot_data;
// extern ext_bullet_remaining_t  ext_bullet_remaining;
// extern ext_rfid_status_t        ext_rfid_status;
// extern ext_dart_client_cmd_t   ext_dart_client_cmd;

// extern uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE];


// ------------------------------
// MY PART (function headers)

_Bool is_red_or_blue(void);
uint8_t get_robot_id(void);
uint8_t get_remain_hp(void);
void Show_CrossHair(BufferedSerial* b);

void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len, BufferedSerial* b);

#endif