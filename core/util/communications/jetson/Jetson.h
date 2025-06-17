#include <cstring>
#include <stdint.h>
#include "communications/SerialCommunication.h"

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

void jetson_send_feedback(BufferedSerial jetson_serial, const Jetson_send_data& data);
ssize_t jetson_read_values(BufferedSerial jetson_serial, Jetson_read_data& read_data);
