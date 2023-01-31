#ifndef _REF_UI_HPP
#define _REF_UI_HPP

#include "mbed.h"
#include "ref_serial.h"
#include <string>

// Screen resolution of your Robomaster Client
#define SCREEN_LENGTH 1920
#define SCREEN_WIDTH 1080

void ui_graph_character(BufferedSerial* serial, int operation_type, string char_data, int x, int y, int name);
void ui_delete_all(BufferedSerial* serial);

#endif