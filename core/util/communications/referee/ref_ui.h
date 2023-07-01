#ifndef _REF_UI_HPP
#define _REF_UI_HPP

#include "mbed.h"
#include "ref_serial.h"
#include <string>

// Screen resolution of your Robomaster Client
#define SCREEN_LENGTH 1920
#define SCREEN_WIDTH 1080

// Draw text on the UI
void ui_graph_characters(BufferedSerial* serial, int operation_type, string str, int x, int y, char name);

// Delete a layer of UI drawing
void ui_delete_layer(BufferedSerial* serial, int layer);

#endif