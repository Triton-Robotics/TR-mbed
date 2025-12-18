#pragma once
#include "mbed.h"
#include <cstring>
#include <stdint.h>
#include "util/communications/StmIO.h"

// TODO Constant definition here?
#define DATA_HEADER 0xAA
#define REF_HEADER 0xBB
#define AIM_HEADER 0xCC
#define ODOM_HEADER 0xDD
#define JETSON_READ_BUFF_SIZE 500
#define JETSON_READ_MSG_SIZE 11 // old code
#define JETSON_MAX_PACKET_SIZE 117

class Jetson: public StmIO {
  public:
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

    struct Jetson_send_ref {
        int8_t game_state;
        int16_t robot_hp;
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
        // static constexpr int size = 28 + 3;
    };

    struct ReadState {
        float desired_pitch_rads;
        float desired_yaw_rads;
        char shoot_status;

        float desired_x_vel;
        float desired_y_vel;
        float desired_angular_vel;
        char localization_calibration;

        // static constexpr int size = 13 + 9;
    };

    Jetson();

    // Init Jetson with a BufferedSerial Object
    Jetson(BufferedSerial &UARTJetson);

    // Init Jetson with a SPISlave Object
    Jetson(SPISlave &SPIJetson);

    /**
     * @brief Send state data to Jetson through UART.
     *
     * @param ref_data: Referee system data: game_state and robot_hp
     * @param data: Robot data: Chassis vX,vY,vOmega, YawPos, YawVel, PitchPos, PitchVel
     */
    void jetson_send_feedback(const Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type = 0);

    /**
     * @brief Read state data from Jetson through UART.
     *
     * @return Number of bytes read
     */
    ssize_t jetson_read_values();

    /**
     * @brief Send state data to Jetson through SPI.
     *
     * @param ref_data: Referee system data: game_state and robot_hp
     * @param data: Robot data: Chassis vX,vY,vOmega, YawPos, YawVel, PitchPos, PitchVel
     */
    void jetson_send_spi(const Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type = 0);

    /**
     * @brief Read data from the Jetson via SPI.
     *
     * @return Number of bytes read
     */
    ssize_t jetson_read_spi();

    // TODO potentially useless?
    void write_robot_state(Jetson_send_data *curr_robot_state);
    void write_ref_state(Jetson_send_ref *curr_ref_state);

    // StmIO things
    void read(ReadState *jetson_state = nullptr);
    void write(WriteState *stm_state = nullptr);

  private:
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

    struct Jetson_send_ref_buf {
        char game_state[1];
        char robot_hp[2];
        int size = 3;
    };

    Mutex mutex_;
    void write_thread();
    void read_thread();

    BufferedSerial* bcJetson;
    SPISlave* spiJetson;

    Jetson_read_data read_data;
    Jetson_read_odom odom_data;

    Jetson_send_data robot_state;
    Jetson_send_ref ref_state;

    char nucleo_value[50] = {0};
    char jetson_read_buff[JETSON_READ_BUFF_SIZE] = {0};
    int jetson_read_buff_pos = 0;
    bool jetson_data_ready = false;

    // Helper Functions
    void getBytesFromFloat(char* byteArr, float value);

    void getBytesFromInt16(char* byteArr, int16_t value);

    void getBytesFromInt8(char* byteArr, int8_t value);

    void decode_toSTM32(char *read_buf, float &received_one, float &received_two, char &received_three, uint8_t &checksum);

    void copy4Char(char* srcBuf, char* destBuf, int offset);

    static uint8_t calculateLRC(const char* data, size_t length);
};


// we want to be able to send both struct types, so we could have 4 args:
// bcJetson, &jetson_send_data = nullptr, &jetson_send_ref = nullptr, ref_data = False
// void jetson_send_feedback(BufferedSerial &bcJetson, const Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type = 0);

// same w receiving
// bcJetson, &jetson_read_data = nullptr, &jetson_read_new_odom = nullptr
// here we return num bytes to determine which type? or we can give a bool to know which one is updated
// well either way we look at both :D
// ssize_t jetson_read_values(BufferedSerial &bcJetson, Jetson_read_data& read_data, Jetson_read_odom& odom_data);


// spi bs
// ssize_t jetson_send_read_spi(SPI &spiJetson, const Jetson_send_data& input, Jetson_read_data& output);