//
// Created by bismarckkk on 2026/3/22.
// Dynamic Edition
//

#ifndef UI_TYPES_H
#define UI_TYPES_H

// User Code Begin

// Defines whether or not we want to manually handling updtaing or not (comment out if we want automatic handling)
// #define MANUAL_DIRTY

#if defined(__GNUC__) || defined(__CC_ARM)
#define MESSAGE_PACKED __attribute__((packed))
#include <stdint.h>
#else
#error "MESSAGE_PACKED not defined for this compiler"
#endif

// User Code End

// Structures for various figures/numbers/strings

typedef struct __attribute__((__packed__)) {
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

typedef struct __attribute__((__packed__)) {
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

#define PRIMITIVE_CAT(x, y) x ## y
#define CAT(x, y) PRIMITIVE_CAT(x, y)

// Creates generic figure interface struct (good for inheritance related functions)
#define DEFINE_MESSAGE(name, p_a, p_b, p_c, p_d, p_e)   \
typedef struct  __attribute__((__packed__)) {           \
    uint8_t figure_name[3];                             \
    uint32_t operate_type:3;                            \
    uint32_t figure_type:3;                             \
    uint32_t layer:4;                                   \
    uint32_t color:4;                                   \
    uint32_t PRIMITIVE_CAT(,p_a) :9;                    \
    uint32_t PRIMITIVE_CAT(,p_b):9;                     \
    uint32_t width:10;                                  \
    uint32_t start_x:11;                                \
    uint32_t start_y:11;                                \
    uint32_t PRIMITIVE_CAT(,p_c):10;                    \
    uint32_t PRIMITIVE_CAT(,p_d):11;                    \
    uint32_t PRIMITIVE_CAT(,p_e):11;                    \
} MESSAGE_PACKED ui_interface_ ## name ##_t             

// Explicitly define figure types with right names
DEFINE_MESSAGE(figure, _a, _b, _c, _d, _e);
DEFINE_MESSAGE(line, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(rect, _a, _b, _c, end_x, end_y);
DEFINE_MESSAGE(round, _a, _b, r, _d, _e);
DEFINE_MESSAGE(ellipse, _a, _b, _c, rx, ry);
DEFINE_MESSAGE(arc, start_angle, end_angle, _c, rx, ry);

// Structure for frame header
typedef struct __attribute__((__packed__)) {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
    uint16_t cmd_id, sub_id;
    uint16_t send_id, recv_id;
} MESSAGE_PACKED ui_frame_header_t;

// Structure for sending figure data in various sized packets
typedef struct __attribute__((__packed__)) {
ui_frame_header_t header;
ui_interface_figure_t data[1];
uint16_t crc16;
} MESSAGE_PACKED ui_1_frame_t;

typedef struct __attribute__((__packed__)) {
ui_frame_header_t header;
ui_interface_figure_t data[2];
uint16_t crc16;
} MESSAGE_PACKED ui_2_frame_t;

typedef struct __attribute__((__packed__)) {
ui_frame_header_t header;
ui_interface_figure_t data[5];
uint16_t crc16;
} MESSAGE_PACKED ui_5_frame_t;

typedef struct __attribute__((__packed__)) {
ui_frame_header_t header;
ui_interface_figure_t data[7];
uint16_t crc16;
} MESSAGE_PACKED ui_7_frame_t;

// Structure for sending string data in packets
typedef struct __attribute__((__packed__)) {
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t crc16;
} MESSAGE_PACKED ui_string_frame_t;

// Structure for deleting layers
typedef struct __attribute__((__packed__)) {
    ui_frame_header_t header;
    uint8_t delete_type;
    uint8_t layer;
    uint16_t crc16;
} MESSAGE_PACKED ui_delete_frame_t;

#endif //UI_TYPES_H
