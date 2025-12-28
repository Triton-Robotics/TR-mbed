#pragma once
#include "mbed.h"
#include "util/communications/StmIO.h"

#include <memory>
#include <vector>

#include "util/communications/jetson/Packet.h"

// TODO Constant definition here?
#define DATA_HEADER 0xAA
#define REF_HEADER 0xBB
#define AIM_HEADER 0xCC
#define ODOM_HEADER 0xDD
#define JETSON_READ_BUFF_SIZE 500
#define JETSON_READ_MSG_SIZE 11 // old code
#define JETSON_MAX_PACKET_SIZE 117

class Jetson {
  public:
    struct WriteState {
        float chassis_x_velocity;
        float chassis_y_velocity;
        float chassis_rotation;
        float pitch_angle_rads;
        float yaw_angle_rads;
        float pitch_velocity;
        float yaw_velocity;

        int8_t game_state;
        int16_t robot_hp;
    };

    struct ReadState {
        float desired_pitch_rads;
        float desired_yaw_rads;
        char shoot_status;

        float desired_x_vel;
        float desired_y_vel;
        float desired_angular_vel;
        char localization_calibration;
    };

    // Init Jetson with a BufferedSerial Object
    Jetson(BufferedSerial &UARTJetson);

    // Init Jetson with a SPISlave Object
    Jetson(SPISlave &SPIJetson);

    static uint8_t calculateLRC(const char *data, size_t length);

    Jetson::ReadState read();
    void write(Jetson::WriteState &to_write);

  private:
    std::vector<std::unique_ptr<WritePacket>> write_packets_;
    std::vector<std::unique_ptr<ReadPacket>> read_packets_;

    ReadState read_state_;
    WriteState write_state_;

    Thread write_thread_;
    Thread read_thread_;
    Mutex mutex_read_;
    Mutex mutex_write_;
    void writeThread();
    void readThread();

    int writeIO(char *buff, int write_size);
    int readIO(char *buff, int buff_size);
    int readIOReadable();

    BufferedSerial *bcJetson;
    SPISlave *spiJetson;

    static constexpr unsigned long WRITE_THREAD_LOOP_DT_MS = 1;
};

// we want to be able to send both struct types, so we could have 4 args:
// bcJetson, &jetson_send_data = nullptr, &jetson_send_ref = nullptr, ref_data =
// False void jetson_send_feedback(BufferedSerial &bcJetson, const
// Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type = 0);

// same w receiving
// bcJetson, &jetson_read_data = nullptr, &jetson_read_new_odom = nullptr
// here we return num bytes to determine which type? or we can give a bool to
// know which one is updated well either way we look at both :D ssize_t
// jetson_read_values(BufferedSerial &bcJetson, Jetson_read_data& read_data,
// Jetson_read_odom& odom_data);

// spi bs
// ssize_t jetson_send_read_spi(SPI &spiJetson, const Jetson_send_data& input,
// Jetson_read_data& output);