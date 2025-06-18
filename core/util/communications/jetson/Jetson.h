#pragma once
#include <cstring>
#include <stdint.h>
// #include "communications/SerialCommunication.h"

#include "mbed.h"

struct Jetson_send_data {
  float chassis_x_velocity;
  float chassis_y_velocity;
  float pitch_angle_rads;
  float yaw_angle_rads;
  float pitch_velocity;
  float yaw_velocity;
};

struct Jetson_read_data {
  float requested_pitch_rads;
  float requested_yaw_rads;
  char shoot_status;
};

// BufferedSerial bcJetson(PC_12, PD_2, 115200);

void jetson_send_feedback(BufferedSerial &bcJetson, const Jetson_send_data& data);
ssize_t jetson_read_values(BufferedSerial &bcJetson, Jetson_read_data& read_data);