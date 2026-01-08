#ifndef _REFEREE_HPP
#define _REFEREE_HPP

#pragma once
#include "mbed.h"
#include <string>
#include "util/communications/StmIO.h"
#include "crc.h"
#include "ref_constants.h"


class Referee: public StmIO {
public:
    Referee(PinName pin_tx, PinName pin_rx);
    
    //BufferedSerial getRef();
    bool readable();

    // Old referee thread code
    void refereeThread();

    void readThread();

    void writeThread();

    void read();

    // TODO: figure out how to write ui or interobot comms 
    void write();

    // Return 1 if robot is in red team, 0 if blue.
    bool is_red_or_blue();

    // Return robot ID.
    uint8_t get_robot_id();

    // Return robot remaining HP.
    uint8_t get_remain_hp();

    // Return game progress
    uint8_t get_game_progress();

    
    robot_status_t                          robot_status;
private:
    BufferedSerial ref;
    Mutex mutex_write_;
    Mutex mutex_read_;
    Thread readThread_;
    Thread writeThread_;
    bool enablePrintRefData = 0;

    uint8_t JudgeSystem_rxBuff_priv[JUDGESYSTEM_PACKSIZE];
    uint8_t JudgeSystem_rxBuff[JUDGESYSTEM_PACKSIZE]; //接收buff
    uint8_t Judge_Self_ID;        //当前机器人ID
    uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID


    /** 
     * @brief Receive data from the referee system serial port
    */
    int JudgeSystem_USART_Receive_DMA();
    void Judge_GetMessage(uint16_t Data_Length);
    void Judge_sendPC();
    void RobotStatus_LEDYellow();


    // Draw text on the UI
    void ui_graph_characters(int operation_type, string str, int x, int y, char name);

    // Delete a layer of UI drawing
    void ui_delete_layer(int layer);
    

    // ------------------------------
    // Extra stuff
    
    game_status_t 		                    game_status;
    game_result_t                           game_result;
    ext_game_robot_HP_t                     ext_game_robot_HP;
    ext_dart_status_t                       ext_dart_status;
    ext_ICRA_buff_debuff_zone_status_t      ext_ICRA_buff_debuff_zone_status;
    ext_event_data_t                        ext_even_data;
    ext_supply_projectile_action_t          ext_supply_projectile_action;
    referee_warning_t                       referee_warning;
    ext_dart_remaining_time_t               ext_dart_remaining_time;
    power_heat_data_t                       power_heat_data;
    robot_pos_t                             robot_pos;
    ext_buff_t                              Buff;
    buff_t                                  buff;
    aerial_robot_energy_t                   aerial_robot_energy;
    hurt_data_t                             hurt_data;
    shoot_data_t                            shoot_data;
    projectile_allowance_t                  projectile_allowance;
    ext_rfid_status_t                       ext_rfid_status;
    ext_dart_client_cmd_t                   ext_dart_client_cmd;

    uint8_t Robot_Commute[26];

    uint8_t seq=0;
    unsigned char CliendTxBuffer[send_max_len];

    /** @brief Send data to the referee system serial port
        @param sof SOF (0xA5)
        @param cmd_id Command ID
        @param p_data Pointer to data
        @param len Length
    */
    void referee_data_pack_handle(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len);

    void determine_ID();

    void init_referee_struct_data();
};

#endif