#pragma once
#include "mbed.h"

#include <memory>
#include <vector>

class WritePacket;
class ReadPacket;

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

// ------
// Write packets
// ------

class WritePacket {
  public:
    WritePacket(char header, int payload_size)
        : header_(header), payload_size_(payload_size) {};

    virtual ~WritePacket() = default;

    const char header_;
    const int payload_size_;

    int write_data_to_buff(Jetson::WriteState &write_state, char *buff,
                           int buff_size) {

        // buff must be able to hold payload + header + checksum
        if (buff_size < payload_size_ + 2) {
            return -1;
        }

        // write payload into buff, leaving 1 byte for the header
        get_packed_payload(write_state, &buff[1]);
        buff[0] = header_;

        uint8_t checksum = Jetson::calculateLRC(&buff[1], payload_size_);
        buff[payload_size_ + 1] = checksum;

        // payload + 1 header byte + 1 checksum byte
        return payload_size_ + 2;
    }

  private:
    virtual void get_packed_payload(Jetson::WriteState &write_state,
                                    char *buff) = 0;
};

class RefWritePacket : public WritePacket {
  public:
    static int constexpr HEADER = 0xBB;
    static int constexpr PAYLOAD_SIZE = 3;
    RefWritePacket() : WritePacket(HEADER, PAYLOAD_SIZE) {};

  private:
    void get_packed_payload(Jetson::WriteState &write_state,
                            char *buff) override {

        std::memcpy(&buff[0], &write_state.game_state, sizeof(uint8_t));
        std::memcpy(&buff[1], &write_state.robot_hp, sizeof(int16_t));
    }
};

class RobotStateWritePacket : public WritePacket {
  public:
    static int constexpr HEADER = 0xAA;
    static int constexpr PAYLOAD_SIZE = 28;

    RobotStateWritePacket() : WritePacket(HEADER, PAYLOAD_SIZE) {};

  private:
    void get_packed_payload(Jetson::WriteState &write_state,
                            char *buff) override {

        std::memcpy(&buff[0], &write_state.yaw_angle_rads, sizeof(float));
        std::memcpy(&buff[4], &write_state.pitch_angle_rads, sizeof(float));

        std::memcpy(&buff[8], &write_state.yaw_velocity, sizeof(float));
        std::memcpy(&buff[12], &write_state.pitch_velocity, sizeof(float));
                              
        std::memcpy(&buff[16], &write_state.chassis_x_velocity, sizeof(float));
        std::memcpy(&buff[20], &write_state.chassis_y_velocity, sizeof(float));
        std::memcpy(&buff[24], &write_state.chassis_rotation, sizeof(float));
    }
};

// --------------
//  READ PACKETS
// --------------

class ReadPacket {
  public:
    const char header_;
    const int payload_size_;
    ReadPacket(char header, int payload_size)
        : header_(header), payload_size_{payload_size} {};

    virtual ~ReadPacket() = default;

    // returns the amount of bytes consumed or -1 if invalid
    int parse_buff(char *buff, int buff_size, Jetson::ReadState &read_state) {
        if (buff[0] != header_) {
            return -1;
        }
        if (buff_size < payload_size_ + 2) {
            return -1;
        }

        char checksum = buff[payload_size_ + 1];
        // calculate checksum without header
        char calculated_checksum =
            Jetson::calculateLRC(&buff[1], payload_size_);

        if (checksum != calculated_checksum) {
            return -1;
        }

        extract_payload(&buff[1], read_state);

        // consumed header + payload + checksum bytes
        return payload_size_ + 2;
    }

  private:
    virtual void extract_payload(char *buff, Jetson::ReadState &read_state) = 0;
};

class TurretPacket : public ReadPacket {
  public:
    static char constexpr HEADER = 0xCC;
    static int constexpr PAYLOAD_SIZE = 9;
    TurretPacket() : ReadPacket(HEADER, PAYLOAD_SIZE) {};

  private:
    void extract_payload(char *buff, Jetson::ReadState &read_state) override {

        // 4 byte pitch, 4 byte yaw, 1 byte shoot
        std::memcpy(&read_state.desired_yaw_rads, &buff[0], sizeof(float));
        std::memcpy(&read_state.desired_pitch_rads, &buff[4], sizeof(float));
        std::memcpy(&read_state.shoot_status, &buff[8], sizeof(char));
    }
};

class ChassisReadPacket : public ReadPacket {
  public:
    static char constexpr HEADER = 0xDD;
    static int constexpr PAYLOAD_SIZE = 13;

    ChassisReadPacket() : ReadPacket(HEADER, PAYLOAD_SIZE) {};

  private:
    void extract_payload(char *buff, Jetson::ReadState &read_state) override {

        // 4 x vel bytes, 4 y vel bytes, 4 rotation vel bytes, 1 localization
        // calibration byte
        std::memcpy(&read_state.desired_x_vel, &buff[0], sizeof(float));
        std::memcpy(&read_state.desired_y_vel, &buff[4], sizeof(float));
        std::memcpy(&read_state.desired_angular_vel, &buff[8], sizeof(float));
        std::memcpy(&read_state.localization_calibration, &buff[12],
                    sizeof(uint8_t));
    }
};
