//
// Created by bismarckkk on 2026/3/22.
// Dynamic Edition
//

#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "ui_types.h"
#include <functional>

extern int ui_self_id;
extern std::function<void(uint8_t*, uint16_t )> send_packet_func;

void print_message(const uint8_t* message, int length);

// User Code Begin
// #define SEND_MESSAGE(message, length) print_message(message, length)
void send_message(uint8_t* message, uint16_t);
// User Code End

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void ui_proc_delete_frame(ui_delete_frame_t *msg);

void ui_delete_layer(const uint8_t delete_type, const uint8_t layer);

void ui_scan_and_send(const ui_interface_figure_t* ui_now_figures, uint8_t* ui_dirty_figure, const ui_interface_string_t* ui_now_strings, uint8_t* ui_dirty_string, int total_figures, int total_strings);

#endif //UI_INTERFACE_H
