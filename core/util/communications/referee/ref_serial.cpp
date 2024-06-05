
#include "ref_serial.h"

// top part: I try to make this object-oriented, but then realized it would be quite complicated

Referee::Referee(PinName pin_tx, PinName pin_rx) : ref(pin_tx, pin_rx, 115200) {
    printf("constructor!");
}

void Referee::clearRxBuffer(){

}

void Referee::read(){
    
}

bool Referee::readable(){
    return ref.readable();
}

// -------------------------------------
// From South China University of Technology 华南理工大学广州学院-野狼战队-步兵代码 ----------------------------------
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.c/RM_JudgeSystem.c
// 2021 section

uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE]; //接收buff
uint8_t Judge_Self_ID;        //当前机器人ID
uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID


void JudgeSystem_USART_Receive_DMA(BufferedSerial* b) // modified
{
    // b->enable_input(TRUE);
    // memset(JudgeSystem_rxBuff, 0, sizeof(JudgeSystem_rxBuff));
    b->enable_output(true);
    b->read(JudgeSystem_rxBuff, JUDGESYSTEM_PACKSIZE);
}

ext_game_status_t ext_game_status;
ext_game_result_t ext_game_result;
ext_game_robot_HP_t ext_game_robot_HP;
ext_dart_status_t ext_dart_status;
ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_debuff_zone_status;
ext_event_data_t ext_even_data;
ext_supply_projectile_action_t ext_supply_projectile_action;
ext_referee_warning_t ext_referee_warning;
ext_dart_remaining_time_t ext_dart_remaining_time;
ext_game_robot_status_t ext_game_robot_state;
ext_power_heat_data_t ext_power_heat_data;
ext_game_robot_pos_t ext_game_robot_pos;
ext_buff_t Buff;
aerial_robot_energy_t aerial_robot_energy;
ext_robot_hurt_t ext_robot_hurt;
ext_shoot_data_t ext_shoot_data;
ext_bullet_remaining_t ext_bullet_remaining;
ext_rfid_status_t ext_rfid_status;
ext_dart_client_cmd_t ext_dart_client_cmd;

// ext_CommunatianData_t CommuData; //队友通信信息
// /*客户端定义*/
// /*准心及指示灯*/
// ext_SendClientData_t ShowData; //客户端信息
// ext_ShowCrossHair_t Ex_ShowData;
// ext_DeleteClientData_t DeleteClient; //删除客户端
// /*射速等级*/
// ext_ShootLevelData_t ShowshootLv;
// ext_ShootLevelData_t ShootLvInit;
// /*超级电容容量*/
// ext_Cap_Energyvalue_t Cap_Energyvalue;
// ext_Cap_Energy_t Cap_Energyshow;
// /*工程抬升高度*/
// ext_UpliftHeightData_t Uplift_Height;
// ext_UpliftHeightData_t Uplift_HeightValue;
// /*工程夹子角度*/
// ext_ClipAngeleData_t Clip_Angle;
// ext_ClipAngeleData_t Clip_AngleValue;
// /*哨兵状态指示*/
// ext_SentryStatusData_t SentryStatus_Data;
// ext_SentryStatusData_t Sentry_Status;
// ext_CommunatianData_t Sentry_CommuData;
// /**飞镖状态指示**/
// ext_DartStatusData_t DartStatus_Data;
// ext_DartStatusData_t Dart_Status;
// ext_CommunatianData_t Dart_CommuData;
// /*车距框*/
// ext_CarDistance_t Car_Distance;
// /*机器人状态灯信息*/
// ext_LedMeaning_t Led_Meaning;
// ext_LedMeaning_t Led_Yellow;
// ext_LedMeaning_t Led_Green;
// ext_LedMeaning_t Led_Orange;
// ext_LedMeaning_t Led_Purple;
// ext_LedMeaning_t Led_Pink;
// //test
// ext_MapCommunate_t MapCommunate;
// ext_robot_command_t Robot_Command;

uint8_t Robot_Commute[26];


void Judge_GetMessage(uint16_t Data_Length)
{
    for (int n = 0; n < Data_Length;)
    {
        if (JudgeSystem_rxBuff[n] == JUDGE_FRAME_HEADER)
        {
            switch (JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8)
            {
            case Judge_Game_StatusData: //比赛状态数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_StatusData))
                {
                    memcpy(ext_game_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[11]));
                    n += JudgeLength_Game_StatusData;
                    ext_game_status.infoUpdateFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Game_ResultData: //比赛结果
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_ResultData))
                {
                    memcpy(ext_game_result.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Game_ResultData;
                    ext_game_result.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_HP: //机器人血量数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_HP))
                {
                    memcpy(&ext_game_robot_HP.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[32]));
                    n += JudgeLength_Robot_HP;
                    ext_game_robot_HP.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Launch: //飞镖发射状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Launch))
                {
                    memcpy(&ext_dart_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[3]));
                    n += JudgeLength_Dart_Launch;
                    ext_dart_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_AI_ChallengeBuff: //AI加成与惩罚
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_AI_ChallengeBuff))
                {
                    memcpy(&ext_ICRA_buff_debuff_zone_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[11]));
                    n += JudgeLength_AI_ChallengeBuff;
                    ext_ICRA_buff_debuff_zone_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Event_Data: //场地事件数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Event_Data))
                {
                    memcpy(&ext_even_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Event_Data;
                    ext_even_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Supply_Station: //补给站动作标识
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Supply_Station))
                {
                    memcpy(&ext_supply_projectile_action.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Supply_Station;
                    ext_supply_projectile_action.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Referee_Warning: //裁判系统警告信息
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Referee_Warning))
                {
                    memcpy(&ext_referee_warning.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[2]));
                    n += JudgeLength_Referee_Warning;
                    ext_referee_warning.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Countdown: //飞镖发射口倒计时
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Countdown))
                {
                    memcpy(&ext_dart_remaining_time.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Dart_Countdown;
                    ext_dart_remaining_time.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_State: //比赛机器人状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_State))
                {
                    memcpy(&ext_game_robot_state.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[27]));
                    n += JudgeLength_Robot_State;
                    ext_game_robot_state.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Power_Heat: //实时功率热量
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Power_Heat))
                {
                    memcpy(&ext_power_heat_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[16]));
                    n += JudgeLength_Power_Heat;
                    ext_power_heat_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Position: //机器人位置
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Position))
                {
                    memcpy(&ext_game_robot_pos.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[16]));
                    n += JudgeLength_Robot_Position;
                    ext_game_robot_pos.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_Buff: //机器人增益
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Buff))
                {
                    memcpy(&Buff.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Robot_Buff;
                    Buff.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Aerial_Energy: //空中机器人能量状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Aerial_Energy))
                {
                    memcpy(&aerial_robot_energy.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Aerial_Energy;
                    aerial_robot_energy.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Injury_State: //伤害状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Injury_State))
                {
                    memcpy(&ext_robot_hurt.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[1]));
                    n += JudgeLength_Injury_State;
                    ext_robot_hurt.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_RealTime_Shoot: //实时射击数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_RealTime_Shoot))
                {
                    memcpy(&ext_shoot_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[7]));
                    n += JudgeLength_RealTime_Shoot;
                    ext_shoot_data.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Remaining_Rounds: //子弹剩余数
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Remaining_Rounds))
                {
                    memcpy(&ext_bullet_remaining.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[6]));
                    n += JudgeLength_Remaining_Rounds;
                    ext_bullet_remaining.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Robot_RFID: //机器人RFID状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_RFID))
                {
                    memcpy(&ext_rfid_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Robot_RFID;
                    ext_rfid_status.InfoUpdataFlag = 1;
                }
                else
                    n++;
                break;
            case Judge_Dart_Client: //飞镖机器人客户端指令数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Client))
                {
                    memcpy(&ext_dart_client_cmd.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[12]));
                    n += JudgeLength_Dart_Client;
                    ext_dart_client_cmd.InfoUpdataFlag = 1;
                }
                else
                    n++; //26
                break;
            case Judge_Robot_Communicate: //机器人信息交互(还有一种写法就是直接case内容ID 不case命令码)
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Commute))
                {
                    memcpy(&Robot_Commute, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[26]));
                    n += JudgeLength_Robot_Commute;
                }
                else
                    n++;
                break;
            default:
                n++;
                break;
            }
        }
        else
            n++;
    }
    //JudgeSystem.InfoUpdateFrame++;
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
_Bool Color;
_Bool is_red_or_blue(void)
{
    Judge_Self_ID = ext_game_robot_state.data.robot_id; //读取当前机器人ID

    if (ext_game_robot_state.data.robot_id > 10)
    {
        return 0; //蓝方 (blue)
    }
    else
    {
        return 1; //红方 (red)
    }
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
    Color = is_red_or_blue();
    if (Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0100 + ext_game_robot_state.data.robot_id; //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + ext_game_robot_state.data.robot_id; //计算客户端ID
    }
}

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
#define send_max_len 300 //200
unsigned char CliendTxBuffer[send_max_len];


// MY PART -------------------------------------

void init_referee_struct_data(void)
{
    memset(&ext_game_robot_state, 0, sizeof(ext_game_robot_status_t));
}
uint8_t get_robot_id(void)
{
    return ext_game_robot_state.data.robot_id;
}
uint8_t get_remain_hp(void)
{
    return ext_game_robot_state.data.current_HP;
}


// Code from CSDN
// https://blog.csdn.net/zhang1079528541/article/details/115435696

#define MAX_SIZE          128    //上传数据最大的长度
#define frameheader_len  5       //帧头长度
#define cmd_len          2       //命令码长度
#define crc_len          2       //CRC16校验
uint8_t seq=0;


void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len, BufferedSerial* b)
{
    
	unsigned char i=i;
	uint8_t tx_buff[MAX_SIZE];

	uint16_t frame_length = frameheader_len + cmd_len + len + crc_len;   //数据帧长度	

	memset(tx_buff,0,frame_length);  //存储数据的数组清零
	
	/*****帧头打包*****/
	tx_buff[0] = sof;//数据帧起始字节
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));//数据帧中data的长度
	tx_buff[3] = seq;//包序号
	Append_CRC8_Check_Sum(tx_buff,frameheader_len);  //帧头校验CRC8

	/*****命令码打包*****/
	memcpy(&tx_buff[frameheader_len],(uint8_t*)&cmd_id, cmd_len);
	
	/*****数据打包*****/
	memcpy(&tx_buff[frameheader_len+cmd_len], p_data, len);
	Append_CRC16_Check_Sum(tx_buff,frame_length);  //一帧数据校验CRC16

    if (seq == 0xff) seq=0;
    else
     seq++;
	
	/*****数据上传*****/
	// USART_ClearFlag(UART4,USART_FLAG_TC);
    LL_USART_ClearFlag_TC(USART3);

    /*
	for(i=0;i<frame_length;i++)
	{
	//   USART_SendData(UART4,tx_buff[i]);
        LL_USART_TransmitData8(USART3, tx_buff[i]);                        //
    // HAL_UART_Transmit(huart4, tx_buff[i], sizeof(tx_buff[i]), 10);     

	//   while (USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET); //等待之前的字符发送完成
		while(LL_USART_IsActiveFlag_TC(USART3) == RESET);
    }
    */
   
    b->write(tx_buff, frame_length);
}