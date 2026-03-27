//
// Created by bismarckkk on 2026/3/22.
// Dynamic Edition
//

#ifndef UI_TYPES_H
#define UI_TYPES_H

// User Code Begin

// Not 100% sure what MANUAL_DIRTY is but guessing it is for more control over
//      handling the variable named "dirty" in ui_g. This allows for more control
//      of when to update
// #define MANUAL_DIRTY

#if defined(__GNUC__) || defined(__CC_ARM)
#define MESSAGE_PACKED __attribute__((packed))
#include <stdint.h>
#else
#error "MESSAGE_PACKED not defined for this compiler"
#endif

// User Code End

// Structures for various figures/numbers/strings
typedef struct {
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t _a:9;
uint32_t _b:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t _c:10;
uint32_t _d:11;
uint32_t _e:11;
} ui_interface_figure_t;

typedef struct {
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t _a:9;
uint32_t _b:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t _c:10;
uint32_t end_x:11;
uint32_t end_y:11;
} ui_interface_line_t;

typedef struct {
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t _a:9;
uint32_t _b:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t _c:10;
uint32_t end_x:11;
uint32_t end_y:11;
} ui_interface_rect_t;

typedef struct {
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t _a:9;
uint32_t _b:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t r:10;
uint32_t _d:11;
uint32_t _e:11;
} ui_interface_round_t;

typedef struct {
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t _a:9;
uint32_t _b:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t _c:10;
uint32_t rx:11;
uint32_t ry:11;
} ui_interface_ellipse_t;

typedef struct {
uint8_t figure_name[3];
uint32_t operate_type:3;
uint32_t figure_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t _c:10;
uint32_t rx:11;
uint32_t ry:11;
} ui_interface_arc_t;

typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t _b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    int32_t number;
} MESSAGE_PACKED ui_interface_number_t;

typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t font_size: 9;
    uint32_t str_length: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t _c: 10;
    uint32_t _d: 11;
    uint32_t _e: 11;
    char string[30];
} MESSAGE_PACKED ui_interface_string_t;

// Structure for frame header
typedef struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
    uint16_t cmd_id, sub_id;
    uint16_t send_id, recv_id;
} MESSAGE_PACKED ui_frame_header_t;

// Structure for sending figure data in various sized packets
typedef struct {
ui_frame_header_t header;
ui_interface_figure_t data[1];
uint16_t crc16;
} MESSAGE_PACKED ui_1_frame_t;

typedef struct {
ui_frame_header_t header;
ui_interface_figure_t data[2];
uint16_t crc16;
} MESSAGE_PACKED ui_2_frame_t;

typedef struct {
ui_frame_header_t header;
ui_interface_figure_t data[5];
uint16_t crc16;
} MESSAGE_PACKED ui_5_frame_t;

typedef struct {
ui_frame_header_t header;
ui_interface_figure_t data[7];
uint16_t crc16;
} MESSAGE_PACKED ui_7_frame_t;

// Structure for sending string data in packets
typedef struct {
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t crc16;
} MESSAGE_PACKED ui_string_frame_t;

// Structure for deleting layers
typedef struct {
    ui_frame_header_t header;
    uint8_t delete_type;
    uint8_t layer;
    uint16_t crc16;
} MESSAGE_PACKED ui_delete_frame_t;

// Grabs these variables from another file
extern ui_string_frame_t _ui_string_frame;
extern ui_1_frame_t _ui_1_frame;
extern ui_2_frame_t _ui_2_frame;
extern ui_5_frame_t _ui_5_frame;
extern ui_7_frame_t _ui_7_frame;

#endif //UI_TYPES_H
