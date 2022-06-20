
#include "ref_serial.hpp"

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

uint16_t DMA_Counter;
void JudgeSystem_Handler(UART_HandleTypeDef *huart) // not used
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    DMA_Counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    Judge_GetMessage(JUDGESYSTEM_PACKSIZE - DMA_Counter);

    __HAL_DMA_SET_COUNTER(huart->hdmarx, JUDGESYSTEM_PACKSIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);

    //RM_Judge.InfoUpdateFrame++;
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
ext_SendClientData_t ShowData; //客户端信息
ext_ShowCrossHair_t Ex_ShowData;
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
            // case Judge_Robot_Communicate: //机器人信息交互(还有一种写法就是直接case内容ID 不case命令码)
            //     if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Commute))
            //     {
            //         memcpy(&Robot_Commute, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[26]));
            //         n += JudgeLength_Robot_Commute;
            //     }
            //     else
            //         n++;
            //     break;
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
        return 0; //蓝方
    }
    else
    {
        return 1; //红方
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
/*显示准心*/
void Standard_CrossHair(BufferedSerial* b)
{
    static uint8_t datalength;

    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID

    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;                           //客户端绘制一个图形
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[0].graphic_name[0] = 1;
    ShowData.cilentData[0].graphic_tpye = 0;
    ShowData.cilentData[0].operate_tpye = 1;
    ShowData.cilentData[0].layer = 0;
    ShowData.cilentData[0].color = 2;
    ShowData.cilentData[0].start_angle = 0;
    ShowData.cilentData[0].end_angle = 0;
    ShowData.cilentData[0].width = 1;
    ShowData.cilentData[0].start_x = 920; //原840
    ShowData.cilentData[0].start_y = 680;
    ShowData.cilentData[0].radius = 0;
    ShowData.cilentData[0].end_x = 1000; //原1080
    ShowData.cilentData[0].end_y = 680;

    ShowData.cilentData[1].graphic_name[0] = 2;
    ShowData.cilentData[1].graphic_tpye = 0;
    ShowData.cilentData[1].operate_tpye = 1;
    ShowData.cilentData[1].layer = 0;
    ShowData.cilentData[1].color = 2;
    ShowData.cilentData[1].start_angle = 0;
    ShowData.cilentData[1].end_angle = 0;
    ShowData.cilentData[1].width = 1;
    ShowData.cilentData[1].start_x = 580; //原 860
    ShowData.cilentData[1].start_y = 540; //原：560
    ShowData.cilentData[1].radius = 0;
    ShowData.cilentData[1].end_x = 1340; //原1060
    ShowData.cilentData[1].end_y = 540;

    ShowData.cilentData[2].graphic_name[0] = 3;
    ShowData.cilentData[2].graphic_tpye = 0;
    ShowData.cilentData[2].operate_tpye = 1;
    ShowData.cilentData[2].layer = 0;
    ShowData.cilentData[2].color = 2;
    ShowData.cilentData[2].start_angle = 0;
    ShowData.cilentData[2].end_angle = 0;
    ShowData.cilentData[2].width = 1;
    ShowData.cilentData[2].start_x = 880; //原890
    ShowData.cilentData[2].start_y = 440;
    ShowData.cilentData[2].radius = 0;
    ShowData.cilentData[2].end_x = 1040; //原1030
    ShowData.cilentData[2].end_y = 440;

    ShowData.cilentData[3].graphic_name[0] = 4;
    ShowData.cilentData[3].graphic_tpye = 0;
    ShowData.cilentData[3].operate_tpye = 1;
    ShowData.cilentData[3].layer = 0;
    ShowData.cilentData[3].color = 2;
    ShowData.cilentData[3].start_angle = 0;
    ShowData.cilentData[3].end_angle = 0;
    ShowData.cilentData[3].width = 1;
    ShowData.cilentData[3].start_x = 960;
    ShowData.cilentData[3].start_y = 700;
    ShowData.cilentData[3].radius = 0;
    ShowData.cilentData[3].end_x = 960;
    ShowData.cilentData[3].end_y = 250;

    //打包写入数据库
    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    datalength = sizeof(ShowData);
    //HAL_UART_Transmit(&huart3, CliendTxBuffer, datalength, 0xFF);
    b->write(CliendTxBuffer,datalength);
}
unsigned char CrossHairBuffer[90];
void Ex_CrossHair(BufferedSerial* b)
{
    static uint8_t crosslength;

    Ex_ShowData.txFrameHeader.SOF = 0xA5;
    Ex_ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 5 * sizeof(ext_client_custom_graphic_single_t);
    Ex_ShowData.txFrameHeader.Seq = 0;
    memcpy(CrossHairBuffer, &Ex_ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CrossHairBuffer, sizeof(xFrameHeader));              //写入帧头CRC8校验码

    Ex_ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID

    //ID已经是自动读取的了
    Ex_ShowData.dataFrameHeader.data_cmd_id = 0x0103;                           //客户端绘制两个个图形
    Ex_ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    Ex_ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    Ex_ShowData.cilentData[0].graphic_name[1] = 9;
    Ex_ShowData.cilentData[0].graphic_tpye = 0;
    Ex_ShowData.cilentData[0].operate_tpye = 1;
    Ex_ShowData.cilentData[0].layer = 0;
    Ex_ShowData.cilentData[0].color = 2;
    Ex_ShowData.cilentData[0].start_angle = 0;
    Ex_ShowData.cilentData[0].end_angle = 0;
    Ex_ShowData.cilentData[0].width = 1;
    Ex_ShowData.cilentData[0].start_x = 900; //原840
    Ex_ShowData.cilentData[0].start_y = 400;
    Ex_ShowData.cilentData[0].radius = 0;
    Ex_ShowData.cilentData[0].end_x = 1020; //原1080
    Ex_ShowData.cilentData[0].end_y = 400;

    Ex_ShowData.cilentData[1].graphic_name[1] = 10;
    Ex_ShowData.cilentData[1].graphic_tpye = 0;
    Ex_ShowData.cilentData[1].operate_tpye = 1;
    Ex_ShowData.cilentData[1].layer = 0;
    Ex_ShowData.cilentData[1].color = 2;
    Ex_ShowData.cilentData[1].start_angle = 0;
    Ex_ShowData.cilentData[1].end_angle = 0;
    Ex_ShowData.cilentData[1].width = 1;
    Ex_ShowData.cilentData[1].start_x = 920; //原 860
    Ex_ShowData.cilentData[1].start_y = 360; //原：560
    Ex_ShowData.cilentData[1].radius = 0;
    Ex_ShowData.cilentData[1].end_x = 1000; //原1060
    Ex_ShowData.cilentData[1].end_y = 360;

    Ex_ShowData.cilentData[2].graphic_name[1] = 11;
    Ex_ShowData.cilentData[2].graphic_tpye = 0;
    Ex_ShowData.cilentData[2].operate_tpye = 1;
    Ex_ShowData.cilentData[2].layer = 0;
    Ex_ShowData.cilentData[2].color = 2;
    Ex_ShowData.cilentData[2].start_angle = 0;
    Ex_ShowData.cilentData[2].end_angle = 0;
    Ex_ShowData.cilentData[2].width = 1;
    Ex_ShowData.cilentData[2].start_x = 940; //原 860
    Ex_ShowData.cilentData[2].start_y = 310; //原：560
    Ex_ShowData.cilentData[2].radius = 0;
    Ex_ShowData.cilentData[2].end_x = 980; //原1060
    Ex_ShowData.cilentData[2].end_y = 310;

    Ex_ShowData.cilentData[2].graphic_name[1] = 10;
    Ex_ShowData.cilentData[2].graphic_tpye = 0;
    Ex_ShowData.cilentData[2].operate_tpye = 1;
    Ex_ShowData.cilentData[2].layer = 0;
    Ex_ShowData.cilentData[2].color = 2;
    Ex_ShowData.cilentData[2].start_angle = 0;
    Ex_ShowData.cilentData[2].end_angle = 0;
    Ex_ShowData.cilentData[2].width = 1;
    Ex_ShowData.cilentData[2].start_x = 945; //原 860
    Ex_ShowData.cilentData[2].start_y = 280; //原：560
    Ex_ShowData.cilentData[2].radius = 0;
    Ex_ShowData.cilentData[2].end_x = 975; //原1060
    Ex_ShowData.cilentData[2].end_y = 280;

    //打包写入数据库
    memcpy(CrossHairBuffer + 5,
           (uint8_t *)&Ex_ShowData.CmdID,
           (sizeof(Ex_ShowData.CmdID) + sizeof(Ex_ShowData.dataFrameHeader) + sizeof(Ex_ShowData.cilentData)));
    Append_CRC16_Check_Sum(CrossHairBuffer, sizeof(Ex_ShowData)); //写入数据段CRC16校验码
    crosslength = sizeof(Ex_ShowData);
    //HAL_UART_Transmit(&huart3, CrossHairBuffer, crosslength, 0xFF);
    b->write(CrossHairBuffer, crosslength);
}
void Show_CrossHair(BufferedSerial* b)
{
    Standard_CrossHair(b);
    Ex_CrossHair(b);
}


void Judge_sendPC(BufferedSerial* b)
{
    static uint8_t datalength;

    determine_ID(); //判断发送者ID和其对应的客户端ID

    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301;

    ShowData.dataFrameHeader.data_cmd_id = 0xD180; //发给客户端的cmd,官方固定
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.sender_ID = Judge_Self_ID;           //发送者的ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID; //客户端的ID，只能为发送者机器人对应的客户端

    /*- 自定义内容 -*/
    //	ShowData.clientData.data1 = (float)Capvoltage_Percent();//电容剩余电量
    //	ShowData.clientData.data2 = (float)Base_Angle_Measure();//吊射角度测
    //	ShowData.clientData.data3 = GIMBAL_PITCH_Judge_Angle();//云台抬头角度

    //打包写入数据段
    memcpy(
        CliendTxBuffer + 5,
        (uint8_t *)&ShowData.CmdID,
        (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));

    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码

    datalength = sizeof(ShowData);

    b->write(CliendTxBuffer, datalength);
    //HAL_UART_Transmit(&huart3, CliendTxBuffer, datalength, 0xFF);
}

/*机器人状态指示*/
uint8_t Status_Datalength = 0;
void RobotStatus_LEDYellow(BufferedSerial* b)
{
    ShowData.txFrameHeader.SOF = 0xA5;
    ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + 7 * sizeof(ext_client_custom_graphic_single_t);
    ShowData.txFrameHeader.Seq = 0;
    memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader)); //写入帧头数据
    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));           //写入帧头CRC8校验码

    ShowData.CmdID = 0x0301; //机器人通信协议

    determine_ID(); //判断发送者ID和其对应的客户端ID
    //ID已经是自动读取的了
    ShowData.dataFrameHeader.data_cmd_id = 0x0104;
    ShowData.dataFrameHeader.sender_ID = ext_game_robot_state.data.robot_id; //发送者ID
    ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;              //客户端ID

    ShowData.cilentData[4].graphic_name[0] = 9;
    ShowData.cilentData[4].graphic_tpye = 2;
    ShowData.cilentData[4].operate_tpye = 1;
    ShowData.cilentData[4].layer = 1;
    ShowData.cilentData[4].color = 1;
    ShowData.cilentData[4].width = 10;
    ShowData.cilentData[4].start_x = 1900;
    ShowData.cilentData[4].start_y = 800;
    ShowData.cilentData[4].radius = 10;

    memcpy(CliendTxBuffer + 5,
           (uint8_t *)&ShowData.CmdID,
           (sizeof(ShowData.CmdID) + sizeof(ShowData.dataFrameHeader) + sizeof(ShowData.cilentData)));
    Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(ShowData)); //写入数据段CRC16校验码
    Status_Datalength = sizeof(ShowData);
    //HAL_UART_Transmit(&huart3, CliendTxBuffer, Status_Datalength, 0xFF);
    b->write(CliendTxBuffer, Status_Datalength);
    //b->write(CliendTxBuffer, sizeof(CliendTxBuffer));
}


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
    return ext_game_robot_state.data.remain_HP;
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
    LL_USART_ClearFlag_TC(UART4);
	for(i=0;i<frame_length;i++)
	{
    
	//   USART_SendData(UART4,tx_buff[i]);
    LL_USART_TransmitData8(UART4, tx_buff[i]);                        //
    // HAL_UART_Transmit(huart4, tx_buff[i], sizeof(tx_buff[i]), 10);     
    // b->write(tx_buff, frame_length);

	//   while (USART_GetFlagStatus(UART4,USART_FLAG_TC) == RESET); //等待之前的字符发送完成
		while(LL_USART_IsActiveFlag_TC(UART4) == RESET);
    }
}