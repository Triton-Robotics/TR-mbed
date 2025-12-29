
#include "ref_serial.h"

// -------------------------------------
// From South China University of Technology 华南理工大学广州学院-野狼战队-步兵代码 ----------------------------------
// https://github.com/wuzjun/2021RM_Infantry/blob/master/Devices/Devices.c/RM_JudgeSystem.c
// 2021 section


int Referee::JudgeSystem_USART_Receive_DMA() // modified
{
    // b->enable_input(TRUE);
    // memset(JudgeSystem_rxBuff, 0, sizeof(JudgeSystem_rxBuff));
    ref.enable_output(true);
    //memset (JudgeSystem_rxBuff,0,JUDGESYSTEM_PACKSIZE);
    return ref.read(JudgeSystem_rxBuff_priv, JUDGESYSTEM_PACKSIZE);
}


void Referee::Judge_GetMessage(uint16_t Data_Length)
{
    #if REF_DEBUG
    // for (int i = 0; i < Data_Length;i++)
    // {
    //     printf("%x ", JudgeSystem_rxBuff[i]);
    // }
    // printf("\n");
    // printf("[%d]\n",Data_Length);
    #endif
    for (int n = 0; n < Data_Length;)
    {
        
        if (JudgeSystem_rxBuff[n] == JUDGE_FRAME_HEADER)
        {
            switch (JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8)
            {
            case Judge_Game_StatusData: //比赛状态数据
                // printf("ST[%d]\n", n);
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_StatusData))
                {
                    #if REF_DEBUG
                    printf("GS[%d]\n", n);
                    #endif
                    memcpy(&game_status, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Game_StatusData - JUDGE_EXTRA]));
                    n += JudgeLength_Game_StatusData;
                    game_status.infoUpdateFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("GS_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Game_ResultData: //比赛结果
                // printf("GR[%d]\n", n);
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Game_ResultData))
                {
                    #if REF_DEBUG
                    printf("GR[%d]\n", n);
                    #endif
                    memcpy(&game_result, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Game_ResultData - JUDGE_EXTRA]));
                    n += JudgeLength_Game_ResultData;
                    game_result.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("GR_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Robot_HP: //机器人血量数据
                // printf("HP[%d]\n", n);
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_HP))
                {
                    #if REF_DEBUG
                    printf("HP[%d]\n", n);
                    #endif
                    memcpy(&ext_game_robot_HP.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_HP - JUDGE_EXTRA]));
                    n += JudgeLength_Robot_HP;
                    ext_game_robot_HP.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("HP_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Dart_Launch: //飞镖发射状态

                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Launch))
                {
                    #if REF_DEBUG
                    printf("DL[%d]\n", n);
                    #endif
                    memcpy(&ext_dart_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Dart_Launch - JUDGE_EXTRA]));
                    n += JudgeLength_Dart_Launch;
                    ext_dart_status.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("DL_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_AI_ChallengeBuff: //AI加成与惩罚
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_AI_ChallengeBuff))
                {
                    #if REF_DEBUG
                    printf("AI[%d]\n", n);
                    #endif
                    memcpy(&ext_ICRA_buff_debuff_zone_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[11]));
                    n += JudgeLength_AI_ChallengeBuff;
                    ext_ICRA_buff_debuff_zone_status.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("AI_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Event_Data: //场地事件数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Event_Data))
                {
                    #if REF_DEBUG
                    printf("EV[%d]\n", n);
                    #endif
                    memcpy(&ext_even_data.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[4]));
                    n += JudgeLength_Event_Data;
                    ext_even_data.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("EV_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Supply_Station: //补给站动作标识
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Supply_Station))
                {
                    #if REF_DEBUG
                    printf("SS[%d]\n", n);
                    #endif
                    memcpy(&ext_supply_projectile_action.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Supply_Station - JUDGE_EXTRA]));
                    n += JudgeLength_Supply_Station;
                    ext_supply_projectile_action.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("SS_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Referee_Warning: //裁判系统警告信息
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Referee_Warning))
                {
                    #if REF_DEBUG
                    printf("RF[%d]\n", n);
                    #endif
                    memcpy(&referee_warning, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Referee_Warning - JUDGE_EXTRA]));
                    n += JudgeLength_Referee_Warning;
                    referee_warning.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("RF_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Dart_Countdown: //飞镖发射口倒计时
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Countdown))
                {
                    #if REF_DEBUG
                    printf("DT[%d]\n", n);
                    #endif
                    memcpy(&ext_dart_remaining_time.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Dart_Countdown - JUDGE_EXTRA]));
                    n += JudgeLength_Dart_Countdown;
                    ext_dart_remaining_time.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("DT_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Robot_State: //比赛机器人状态
                // for(int i = 0; i < JudgeLength_Robot_State; i ++){
                //     printf("|%2x", (uint8_t)*(JudgeSystem_rxBuff+n+i));
                // }
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_State))
                {
                    #if REF_DEBUG
                    printf("RS[%d]\n", n);
                    robot_status_t* gamering = (robot_status_t*)&JudgeSystem_rxBuff[n + 7];
                    // printf("G%u %u %u %u %u %u %u %x\n", 
                    //     gamering->robot_id,
                    //     gamering->robot_level,
                    //     gamering->current_HP,
                    //     gamering->maximum_HP,    
                    //     gamering->shooter_barrel_cooling_value,
                    //     gamering->shooter_barrel_heat_limit,
                    //     gamering->chassis_power_limit,
                    //     gamering->power_management_output);
                    #endif
                    memcpy(&robot_status, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_State - JUDGE_EXTRA]));

                    n += JudgeLength_Robot_State;
                    robot_status.InfoUpdataFlag = 1;

                    #if REF_DEBUG
                    // printf("R%u %u %u %u %u %u %u %x\n", 
                    //     robot_status.robot_id,
                    //     robot_status.robot_level,
                    //     robot_status.current_HP,
                    //     robot_status.maximum_HP,
                    //     robot_status.shooter_barrel_cooling_value,
                    //     robot_status.shooter_barrel_heat_limit,
                    //     robot_status.chassis_power_limit,
                    //     robot_status.power_management_output);
                    #endif
                }
                else{
                    #if REF_DEBUG
                    printf("RS_NO[%d|", n);
                    for(int i = 0; i < JudgeLength_Robot_State; i ++){
                       printf("%2x|", (uint8_t)*(JudgeSystem_rxBuff+n+i));
                    }
                    #endif
                    n++;
                }
                break;
            case Judge_Power_Heat: //实时功率热量
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Power_Heat))
                {
                    #if REF_DEBUG
                    printf("PH[%d]\n", n);
                    #endif
                    //WIERD BUG BYPASS
                    power_heat_data_t preData = power_heat_data;
                    memcpy(&power_heat_data, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Power_Heat - JUDGE_EXTRA]));
                    
                    if(power_heat_data.shooter_17mm_1_barrel_heat == 0 && preData.shooter_17mm_1_barrel_heat != 0){
                        power_heat_data.shooter_17mm_1_barrel_heat = preData.shooter_17mm_1_barrel_heat;
                    }
                    if(power_heat_data.shooter_17mm_2_barrel_heat == 0 && preData.shooter_17mm_2_barrel_heat != 0){
                        power_heat_data.shooter_17mm_2_barrel_heat = preData.shooter_17mm_2_barrel_heat;
                    }
                    if(power_heat_data.shooter_42mm_barrel_heat == 0 && preData.shooter_42mm_barrel_heat != 0){
                        power_heat_data.shooter_42mm_barrel_heat = preData.shooter_42mm_barrel_heat;
                    }
                    
                    n += JudgeLength_Power_Heat;
                    power_heat_data.InfoUpdataFlag = 1;
                    #if REF_DEBUG
                    //printf("%u %u %u %u\n", power_heat_data.buffer_energy, power_heat_data.shooter_17mm_1_barrel_heat, power_heat_data.shooter_17mm_2_barrel_heat, power_heat_data.shooter_42mm_barrel_heat);
                    #endif
                }
                else{
                    #if REF_DEBUG
                    printf("PH_NO[%d|", n);
                    // for(int i = 0; i < JudgeLength_Power_Heat; i ++){
                    //    printf("%2x|", (uint8_t)*(JudgeSystem_rxBuff+n+i));
                    // }
                    #endif
                    n++;
                }
                break;
            case Judge_Robot_Position: //机器人位置
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Position))
                {
                    #if REF_DEBUG
                    printf("RP[%d]\n", n);
                    #endif
                    memcpy(&robot_pos, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_Position - JUDGE_EXTRA]));
                    n += JudgeLength_Robot_Position;
                    robot_pos.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("RP_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Robot_Buff: //机器人增益
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Buff))
                {
                    #if REF_DEBUG
                    printf("BF[%d]\n", n);
                    #endif
                    memcpy(&buff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_Buff - JUDGE_EXTRA]));
                    n += JudgeLength_Robot_Buff;
                    Buff.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("BF_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Aerial_Energy: //空中机器人能量状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Aerial_Energy))
                {
                    #if REF_DEBUG
                    printf("AE[%d]\n", n);
                    #endif
                    memcpy(&aerial_robot_energy.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Aerial_Energy - JUDGE_EXTRA]));
                    n += JudgeLength_Aerial_Energy;
                    aerial_robot_energy.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("AE_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Injury_State: //伤害状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Injury_State))
                {
                    #if REF_DEBUG
                    printf("IN[%d]\n", n);
                    #endif
                    memcpy(&hurt_data, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Injury_State - JUDGE_EXTRA]));
                    n += JudgeLength_Injury_State;
                    hurt_data.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("IN_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_RealTime_Shoot: //实时射击数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_RealTime_Shoot))
                {
                    #if REF_DEBUG
                    printf("RE[%d]\n", n);
                    #endif
                    memcpy(&shoot_data, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_RealTime_Shoot - JUDGE_EXTRA]));
                    n += JudgeLength_RealTime_Shoot;
                    shoot_data.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("RE_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Remaining_Rounds: //子弹剩余数
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Remaining_Rounds))
                {
                    #if REF_DEBUG
                    printf("RR[%d]\n", n);
                    #endif
                    memcpy(&projectile_allowance, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Remaining_Rounds - JUDGE_EXTRA]));
                    n += JudgeLength_Remaining_Rounds;
                    projectile_allowance.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("RR_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Robot_RFID: //机器人RFID状态
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_RFID))
                {
                    #if REF_DEBUG
                    printf("ID[%d]\n", n);
                    #endif
                    memcpy(&ext_rfid_status.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[JudgeLength_Robot_RFID - JUDGE_EXTRA]));
                    n += JudgeLength_Robot_RFID;
                    ext_rfid_status.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("ID_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            case Judge_Dart_Client: //飞镖机器人客户端指令数据
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Dart_Client))
                {
                    #if REF_DEBUG
                    printf("RC[%d]\n", n);
                    #endif
                    memcpy(&ext_dart_client_cmd.data.dataBuff, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[12]));
                    n += JudgeLength_Dart_Client;
                    ext_dart_client_cmd.InfoUpdataFlag = 1;
                }
                else{
                    #if REF_DEBUG
                    printf("DC_NO[%d]\n", n);
                    #endif
                    n++;
                } //26
                break;
            case Judge_Robot_Communicate: //机器人信息交互(还有一种写法就是直接case内容ID 不case命令码)
                if (Verify_CRC16_Check_Sum(JudgeSystem_rxBuff + n, JudgeLength_Robot_Commute))
                {
                    memcpy(&Robot_Commute, &JudgeSystem_rxBuff[n + 7], sizeof(uint8_t[26]));
                    n += JudgeLength_Robot_Commute;
                }
                else{
                    #if REF_DEBUG
                    printf("RC_NO[%d]\n", n);
                    #endif
                    n++;
                }
                break;
            default:
                // printf("[none %x]\n", JudgeSystem_rxBuff[n + 5] | JudgeSystem_rxBuff[n + 6] << 8);
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
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void Referee::determine_ID()
{
    bool Color = is_red_or_blue();
    if (Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0100 + robot_status.robot_id; //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + robot_status.robot_id; //计算客户端ID
    }
}

/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */


// MY PART -------------------------------------

void Referee::init_referee_struct_data()
{
    memset(&robot_status, 0, sizeof(robot_status_t));
}


// Code from CSDN
// https://blog.csdn.net/zhang1079528541/article/details/115435696
void Referee::referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    
	unsigned char i=i;
	uint8_t tx_buff[MAX_SIZE];

    mutex_write_.lock();
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
    mutex_write_.unlock();
    
    ref.write(tx_buff, frame_length);
}