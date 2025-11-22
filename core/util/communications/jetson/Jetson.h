#pragma once
#include <cstring>
#include <stdint.h>
// #include "communications/SerialCommunication.h"

#include "mbed.h"

struct Jetson_send_data {
  float chassis_x_velocity;
  float chassis_y_velocity;
  float chassis_rotation;
  float pitch_angle_rads;
  float yaw_angle_rads;
  float pitch_velocity;
  float yaw_velocity;
  int size = 28;
};

struct Jetson_send_data_buf {
    char chassis_x_velocity_char[4];
    char chassis_y_velocity_char[4];
    char chassis_rotation_char[4];
    char yaw_angle_char[4];
    char yaw_velocity_char[4];
    char pitch_angle_char[4];
    char pitch_velocity_char[4];
    int size = 28;
};

struct Jetson_send_ref {
    int8_t game_state;
    int16_t robot_hp;
    int size = 3;
};

struct Jetson_send_ref_buf {
    char game_state[1];
    char robot_hp[2];
    int size = 3;
};

struct Jetson_read_data {
    float requested_pitch_rads;
    float requested_yaw_rads;
    char shoot_status;
    int size = 9;
};

struct Jetson_read_odom {
    float x_vel;
    float y_vel;
    float rotation;
    char calibration;
    int size = 13;
};

// BufferedSerial bcJetson(PC_12, PD_2, 115200);

// we want to be able to send both struct types, so we could have 4 args:
// bcJetson, &jetson_send_data = nullptr, &jetson_send_ref = nullptr, ref_data = False
void jetson_send_feedback(BufferedSerial &bcJetson, const Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type = 0);

// same w receiving
// bcJetson, &jetson_read_data = nullptr, &jetson_read_new_odom = nullptr
// here we return num bytes to determine which type? or we can give a bool to know which one is updated
// well either way we look at both :D
ssize_t jetson_read_values(BufferedSerial &bcJetson, Jetson_read_data& read_data, Jetson_read_odom& odom_data);


// spi bs
void jetson_send_spi(SPISlave &spiJetson, const Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type = 0);

ssize_t jetson_read_spi(SPISlave &spiJetson, Jetson_read_data& read_data, Jetson_read_odom& odom_data);