/*
Notes:
- ui_g.c & ui_g.h can be replaced all the time
- use bismark to create ui and generate the DYNAMIC version of the code 
*/

#include "string.h"
#include "ui_g.h"

/** 
 * @brief Sets up robot id and sending function
 * @param robot_id is the robot id to send the UI to
 * @param send_func is the function used to send the data
*/
UI::UI(uint16_t robot_id, std::function<void(uint8_t*, uint16_t )> send_func) {
    ui_self_id = robot_id;
    send_packet_func = send_func;
}

/** 
 * @brief Setup up all figures & strings
*/
void UI::ui_init_g() {
    ui_g_Ungroup_Bayblade->figure_type = 7;
    ui_g_Ungroup_Bayblade->operate_type = 1;
    ui_g_Ungroup_Bayblade->layer = 0;
    ui_g_Ungroup_Bayblade->color = 2;
    ui_g_Ungroup_Bayblade->start_x = 688;
    ui_g_Ungroup_Bayblade->start_y = 839;
    ui_g_Ungroup_Bayblade->width = 8;
    ui_g_Ungroup_Bayblade->font_size = 80;
    ui_g_Ungroup_Bayblade->str_length = 7;
    memset(ui_g_Ungroup_Bayblade->string, 0, sizeof(ui_g_Ungroup_Bayblade->string));
    strcpy(ui_g_Ungroup_Bayblade->string, "SPIN!!!");

    ui_g_Ungroup_Flywheel->figure_type = 7;
    ui_g_Ungroup_Flywheel->operate_type = 1;
    ui_g_Ungroup_Flywheel->layer = 0;
    ui_g_Ungroup_Flywheel->color = 6;
    ui_g_Ungroup_Flywheel->start_x = 758;
    ui_g_Ungroup_Flywheel->start_y = 370;
    ui_g_Ungroup_Flywheel->width = 6;
    ui_g_Ungroup_Flywheel->font_size = 60;
    ui_g_Ungroup_Flywheel->str_length = 7;
    memset(ui_g_Ungroup_Flywheel->string, 0, sizeof(ui_g_Ungroup_Flywheel->string));
    strcpy(ui_g_Ungroup_Flywheel->string, "FLY-OFF");

    ui_g_Ungroup_CV->figure_type = 7;
    ui_g_Ungroup_CV->operate_type = 1;
    ui_g_Ungroup_CV->layer = 0;
    ui_g_Ungroup_CV->color = 0;
    ui_g_Ungroup_CV->start_x = 42;
    ui_g_Ungroup_CV->start_y = 751;
    ui_g_Ungroup_CV->width = 5;
    ui_g_Ungroup_CV->font_size = 50;
    ui_g_Ungroup_CV->str_length = 6;
    memset(ui_g_Ungroup_CV->string, 0, sizeof(ui_g_Ungroup_CV->string));
    strcpy(ui_g_Ungroup_CV->string, "CV-OFF");

    ui_g_Ungroup_Alignment->figure_type = 7;
    ui_g_Ungroup_Alignment->operate_type = 1;
    ui_g_Ungroup_Alignment->layer = 0;
    ui_g_Ungroup_Alignment->color = 3;
    ui_g_Ungroup_Alignment->start_x = 48;
    ui_g_Ungroup_Alignment->start_y = 607;
    ui_g_Ungroup_Alignment->width = 6;
    ui_g_Ungroup_Alignment->font_size = 60;
    ui_g_Ungroup_Alignment->str_length = 2;
    memset(ui_g_Ungroup_Alignment->string, 0, sizeof(ui_g_Ungroup_Alignment->string));
    strcpy(ui_g_Ungroup_Alignment->string, "AL");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
        ui_g_dirty_string[i] = 1;
        idx++;
    }
    
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
        ui_g_dirty_figure[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

/**
 * Tells server to re-add stuff
 */
void UI::ui_reinit_g() {
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 1;
        ui_g_dirty_string[i] = 1;
    }
    
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 1;
        ui_g_dirty_figure[i] = 1;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

/**
    @brief Checks what figures/strings has been updated and only sends those that have changes
        to the server
*/
void UI::ui_update_g() {
    SCAN_AND_SEND();
}

/** 
 * @brief prints message to serial monitor
 * @param message pointer to the packets
 * @param length length of message
*/
void UI::print_message(const uint8_t *message, const int length) {
    for (int i = 0; i < length; i++) {
        printf("%2d ", i);
    }
    printf("\n");
    
    for (int i = 0; i < length; i++) {
        printf("%02x ", message[i]);
    }
    printf("\n\n");
}

/**
 * @brief sends the message to the server (make sure the variable, send_packet_func,
 *      is set to the function that sends data before calling this function)
 * @param message pointer to the packets
 * @param length length of message
 */
void UI::send_message(uint8_t* message, uint16_t length) {
    print_message(message, length);
    if(send_packet_func != NULL) {
        send_packet_func(message, length);
    }
}

/**
 * @brief calculates and returns the proper crc8 for a given message
 */
unsigned char UI::calc_crc8(unsigned char *pchMessage, unsigned int dwLength) {
    unsigned char ucCRC8 = 0xff;
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/**
 * @brief calculates and returns the proper crc16 for a given message
 */
uint16_t UI::calc_crc16(uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wCRC = 0xffff;
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/**
 * @brief sets up the entire packet for a message of 1 figure
 */
void UI::ui_proc_1_frame(ui_1_frame_t *msg) {
    uint16_t num = 1;
    uint16_t id = 0x0101;

    msg->header.SOF = 0xA5;                                 
    msg->header.length = 6 + 15 * num; //15 for the length of the frame                  
    msg->header.seq = seq++;                                
    msg->header.crc8 = calc_crc8((uint8_t*)msg, 4);        
    msg->header.cmd_id = 0x0301;                            
    msg->header.sub_id = id;                                
    msg->header.send_id = ui_self_id;                       
    msg->header.recv_id = ui_self_id + 256;                 
    msg->crc16 = calc_crc16((uint8_t*)msg, 13 + 15 * num); 
}

/**
 * @brief sets up the entire packet for a message of 2 figures
 */
void UI::ui_proc_2_frame(ui_2_frame_t *msg) {
    uint16_t num = 2;
    uint16_t id = 0x0102;
    
    msg->header.SOF = 0xA5;                                 
    msg->header.length = 6 + 15 * num;                      
    msg->header.seq = seq++;                                
    msg->header.crc8 = calc_crc8((uint8_t*)msg, 4);        
    msg->header.cmd_id = 0x0301;                            
    msg->header.sub_id = id;                                
    msg->header.send_id = ui_self_id;                       
    msg->header.recv_id = ui_self_id + 256;                 
    msg->crc16 = calc_crc16((uint8_t*)msg, 13 + 15 * num); 
}

/**
 * @brief sets up the entire packet for a message of 5 figures
 */
void UI::ui_proc_5_frame(ui_5_frame_t *msg) {
    uint16_t num = 5;
    uint16_t id = 0x0103;
    msg->header.SOF = 0xA5;                                 
    msg->header.length = 6 + 15 * num;                      
    msg->header.seq = seq++;                                
    msg->header.crc8 = calc_crc8((uint8_t*)msg, 4);        
    msg->header.cmd_id = 0x0301;                            
    msg->header.sub_id = id;                                
    msg->header.send_id = ui_self_id;                       
    msg->header.recv_id = ui_self_id + 256;                 
    msg->crc16 = calc_crc16((uint8_t*)msg, 13 + 15 * num); 
}

/**
 * @brief sets up the entire packet for a message of 7 figures
 */
void UI::ui_proc_7_frame(ui_7_frame_t *msg) {
    uint16_t num = 7;
    uint16_t id = 0x0104;

    msg->header.SOF = 0xA5;                                 
    msg->header.length = 6 + 15 * num;                      
    msg->header.seq = seq++;                                
    msg->header.crc8 = calc_crc8((uint8_t*)msg, 4);        
    msg->header.cmd_id = 0x0301;                            
    msg->header.sub_id = id;                                
    msg->header.send_id = ui_self_id;                       
    msg->header.recv_id = ui_self_id + 256;                 
    msg->crc16 = calc_crc16((uint8_t*)msg, 13 + 15 * num); 
}

/**
 * @brief sets up the entire packet for a message containing a string
 */
void UI::ui_proc_string_frame(ui_string_frame_t *msg) {
    msg->header.SOF = 0xA5;
    msg->header.length = 51;
    msg->header.seq = seq++;
    msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
    msg->header.cmd_id = 0x0301;
    msg->header.sub_id = 0x0110;
    msg->header.send_id = ui_self_id;
    msg->header.recv_id = ui_self_id + 256;
    msg->crc16 = calc_crc16((uint8_t *) msg, sizeof(msg->header) + sizeof(msg->option));
}

/**
 * @brief sets up the entire packet for deleting an etnrie frame
 */
void UI::ui_proc_delete_frame(ui_delete_frame_t *msg) {
    msg->header.SOF = 0xA5;
    msg->header.length = 8;
    msg->header.seq = seq++;
    msg->header.crc8 = calc_crc8((uint8_t *) msg, 4);
    msg->header.cmd_id = 0x0301;
    msg->header.sub_id = 0x0100;
    msg->header.send_id = ui_self_id;
    msg->header.recv_id = ui_self_id + 256;
    msg->crc16 = calc_crc16((uint8_t *) msg, 15);
}

/**
 * @brief sets up the entire packet for deleting an entire layer
 */
void UI::ui_delete_layer(const uint8_t delete_type, const uint8_t layer) {
    ui_delete_frame.delete_type = delete_type;
    ui_delete_frame.layer = layer;
    ui_proc_delete_frame(&ui_delete_frame);
    send_message((uint8_t *) &ui_delete_frame, sizeof(ui_delete_frame));
}

/**
    @brief Packs figures and strings into appropriately sized packets and then 
        sends to server
    @param ui_now_figures: list of figures to be proccessed and sent
    @param ui_dirty_figure: list whose indexes correspond to ui_now_figures, any 
        indexes greater than 0 indicates that the figure needs to be proccessed 
        and sent
    @param ui_now_strings: list strings to be proccessed and sent
    @param ui_dirty_string: list whose indexes correspond to ui_now_strings, any 
        indexes greater than 0 indicates that the string needs to be proccessed 
        and sent
    @param total_figures: maximum amount of figures that might be sent (usually the
        size of ui_now_figures)
    @param total_strings: maximum amount of strings that might be sent (usually the
        size of ui_now_strings)
     
*/
void UI::ui_scan_and_send(const ui_interface_figure_t *ui_now_figures, uint8_t *ui_dirty_figure,
                        const ui_interface_string_t *ui_now_strings, uint8_t *ui_dirty_string, 
                        const int total_figures, const int total_strings) {
    // Counts the amount of UI figures that need to be sent
    if (total_figures > 0) {
        int dirty_count = 0; // Amount of actual figures that need to be sent
        int dirty_indices[total_figures];
        for (int i = 0; i < total_figures; i++) {
            if (ui_dirty_figure[i] > 0) {
                dirty_indices[dirty_count] = i;
                dirty_count++;
            }
        }

        // Puts each figure into a packet
        for (int now_cap = 0, pack_size = 0; now_cap < dirty_count; now_cap++) {
            // Determines which index to be packed into
            const int now_idx = now_cap % 7;
            const int i = dirty_indices[now_cap];

            // If now_idx is 0, we are on a new packet. Thus we determine
            // what size this new packet is
            if (now_idx == 0) {
                const int remain_size = dirty_count - now_cap;
                if (remain_size > 5) {
                    pack_size = 7;
                } else if (remain_size > 2) {
                    pack_size = 5;
                } else if (remain_size > 1) {
                    pack_size = 2;
                } else {
                    pack_size = 1;
                }
            }

            // Assigns our figure to the appropiate index in the appropiate packet
            if (pack_size == 7) {
                _ui_7_frame.data[now_idx] = ui_now_figures[i];
            } else if (pack_size == 5) {
                _ui_5_frame.data[now_idx] = ui_now_figures[i];
            } else if (pack_size == 2) {
                _ui_2_frame.data[now_idx] = ui_now_figures[i];
            } else {
                _ui_1_frame.data[now_idx] = ui_now_figures[i];
            }

            // Checks if this figure is at the end of our current packet or 
            // this is the the last packet in in the function
            if (now_idx + 1 == pack_size || now_cap + 1 == dirty_count) {
                // Fills all remaining packet indexes with the "no operation" command
                // just in case there is left-over data from previous itterations 
                for (int j = now_idx + 1; j < pack_size + 1; j++) {
                    if (pack_size == 7) {
                        _ui_7_frame.data[j].operate_type = 0;
                    } else if (pack_size == 5) {
                        _ui_5_frame.data[j].operate_type = 0;
                    } else if (pack_size == 2) {
                        _ui_2_frame.data[j].operate_type = 0;
                    } else {
                        _ui_1_frame.data[j].operate_type = 0;
                    }
                }

                // Send message because we're complete with our packet
                if (pack_size == 7) {
                    ui_proc_7_frame(&_ui_7_frame);
                    send_message((uint8_t *) &_ui_7_frame, sizeof(_ui_7_frame));
                } else if (pack_size == 5) {
                    ui_proc_5_frame(&_ui_5_frame);
                    send_message((uint8_t *) &_ui_5_frame, sizeof(_ui_5_frame));
                } else if (pack_size == 2) {
                    ui_proc_2_frame(&_ui_2_frame);
                    send_message((uint8_t *) &_ui_2_frame, sizeof(_ui_2_frame));
                } else {
                    ui_proc_1_frame(&_ui_1_frame);
                    send_message((uint8_t *) &_ui_1_frame, sizeof(_ui_1_frame));
                }
            }
            // Shifts which index of the packet we're putting the next figure into
            now_cap++;
            // Decreases it to signal that the figure has been proccessed once
            ui_dirty_figure[i]--;
        }
    }
    
    // Proccesses and sends all the strings
    if (total_strings > 0) {
        for (int i = 0; i < total_strings; i++) {
            bool should_send = false;
            if (ui_dirty_string[i] > 0 && ui_self_id != 0) {
                _ui_string_frame.option = ui_now_strings[i];
                ui_proc_string_frame(&_ui_string_frame);
                ui_dirty_string[i]--;
                should_send = true;
            }
            
            if(should_send) {
                send_message((uint8_t *) &_ui_string_frame, sizeof(_ui_string_frame));
            }
        }
    }
}

void UI::SCAN_AND_SEND() {
    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);
}

void UI::set_spin_ui(bool is_spinning) {
    ui_lock.lock();
    if(is_spinning) {
        strcpy(ui_g_Ungroup_Bayblade->string, "SPIN!!!");
        ui_g_Ungroup_Bayblade->str_length = 7;
    } else {
        strcpy(ui_g_Ungroup_Bayblade->string, "STOP");
        ui_g_Ungroup_Bayblade->str_length = 4;
    }
    *ui_g_Ungroup_Bayblade_send_count = ui_g_Ungroup_Bayblade_max_send_count;
    ui_lock.unlock();
}

void UI::set_flywheel_ui(bool is_flywheel_on) {
    ui_lock.lock();
    if(is_flywheel_on) {
        strcpy(ui_g_Ungroup_Flywheel->string, "FLY-ON");
        ui_g_Ungroup_Flywheel->str_length = 6;
    } else {
        strcpy(ui_g_Ungroup_Flywheel->string, "FLY-OFF");
        ui_g_Ungroup_Flywheel->str_length = 7;
    }
    *ui_g_Ungroup_Flywheel_send_count = ui_g_Ungroup_Flywheel_max_send_count;
    ui_lock.unlock();
}

void UI::set_cv_ui(bool is_cv_on) {
    ui_lock.lock();
    if(!is_cv_on) {
        strcpy(ui_g_Ungroup_CV->string, "CV-OFF");
        ui_g_Ungroup_CV->str_length = 6;
    } else {
        strcpy(ui_g_Ungroup_CV->string, "CV-ON");
        ui_g_Ungroup_CV->str_length = 5;
    }
    *ui_g_Ungroup_CV_send_count = ui_g_Ungroup_CV_max_send_count;
    ui_lock.unlock();
}

void UI::set_alignment_ui(bool is_aligned) {
    ui_lock.lock();
    if(is_aligned) {
        strcpy(ui_g_Ungroup_Alignment->string, "AL");
        ui_g_Ungroup_Alignment->str_length = 2;
    } else {
        strcpy(ui_g_Ungroup_Alignment->string, "NO-AL");
        ui_g_Ungroup_Alignment->str_length = 5;
    }
    *ui_g_Ungroup_Alignment_send_count = ui_g_Ungroup_Alignment_max_send_count;
    ui_lock.unlock();
}