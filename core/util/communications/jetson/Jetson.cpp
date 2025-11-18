#include "Jetson.h"
#define DATA_HEADER 0xAA
#define REF_HEADER 0xBB
#define AIM_HEADER 0xCC
#define ODOM_HEADER 0xDD
#define JETSON_READ_BUFF_SIZE 500
#define JETSON_READ_MSG_SIZE 11 // old code
#define JETSON_MAX_PACKET_SIZE 117
char nucleo_value[50] = {0};
char jetson_read_buff[JETSON_READ_BUFF_SIZE] = {0};
int jetson_read_buff_pos = 0;

/**
 * Copy float `value` bytes into single bytes in `byteArr` array
 * @param byteArr destination char array for value individual bytes
 * @param value float value to copy into byteArr
 */
void getBytesFromFloat(char* byteArr, float value) {
  std::memcpy(byteArr, &value, sizeof(float));
}

void getBytesFromInt16(char* byteArr, int16_t value) {
  std::memcpy(byteArr, &value, sizeof(int16_t));
}

void getBytesFromInt8(char* byteArr, int8_t value) {
  std::memcpy(byteArr, &value, sizeof(int8_t));
}

/**
* Writes 9 bytes of read_buf into received_one and received_two as floats for pitch & yaw positions
* Used for receiving desired position data from CV in read_buf, write out as 
* floats to received_one/two.
* @param read_buf - Source data
* @param received_one - Destination buffer
*/
void decode_toSTM32(char *read_buf, float &received_one, float &received_two, char &received_three, uint8_t &checksum){
  memcpy(&received_one, read_buf, sizeof(float));
  memcpy(&received_two, read_buf + 4, sizeof(float));
  memcpy(&received_three, read_buf + 8, sizeof(char)); // shooting indicator
  checksum = read_buf[9];
}

/**
* Copies 4 bytes from srcBuf[0] into destBuf[offset]
* @param srcBuf source buffer
* @param destBuf destination buffer
* @param offset the starting position into destBuf
*/
void copy4Char(char* srcBuf, char* destBuf, int offset){
  for(int i = 0; i < 4; i ++){
      destBuf[offset+i] = srcBuf[i];
  }
}

/**
* Performs a Longitudinal Redundancy Check
* @param data the data to compute the checksum on
* @param length the length of data
*/
static uint8_t calculateLRC(const char* data, size_t length) {
  unsigned char lrc = 0;
  for (size_t i = 0; i < length; ++i) {
      lrc += data[i];
      lrc &= 0xff;
  }
  lrc = ((lrc ^ 0xff) + 1) & 0xff;
  return lrc;
}

/**
 * Read motor values and send to CV
 */
void jetson_send_feedback(BufferedSerial &bcJetson, const Jetson_send_ref& ref_data, const Jetson_send_data& data, int msg_type) {
    // Implementation for sending both Jetson_send_ref and Jetson_send_data
    if (msg_type == 1) {
        // Send ref data
        Jetson_send_ref_buf data_buf;

        getBytesFromInt8(data_buf.game_state, ref_data.game_state);
        getBytesFromInt16(data_buf.robot_hp, ref_data.robot_hp);

        // 0  1 2 3 4    - 5 total bytes
        // EF g h h checksum
        //put the data into temp
        int startPositions[2] = {1, 2};
        nucleo_value[0] = REF_HEADER;
        copy4Char(data_buf.game_state, nucleo_value, startPositions[0]);
        copy4Char(data_buf.robot_hp, nucleo_value, startPositions[1]);

        uint8_t lrc = calculateLRC(nucleo_value + 1, data_buf.size); //exclude header byte
        char lrc_char = static_cast<uint8_t>(lrc);
        nucleo_value[data_buf.size + 1] = lrc_char;

        bcJetson.sync();
        bcJetson.write(nucleo_value, (data_buf.size + 2));
    }
    else if (msg_type == 2) {
        // Send data
        Jetson_send_data_buf data_buf;
        
        getBytesFromFloat(data_buf.chassis_x_velocity_char, data.chassis_x_velocity);
        getBytesFromFloat(data_buf.chassis_y_velocity_char, data.chassis_y_velocity);
        getBytesFromFloat(data_buf.chassis_rotation_char, data.chassis_rotation);
        getBytesFromFloat(data_buf.yaw_angle_char, data.yaw_angle_rads);
        getBytesFromFloat(data_buf.yaw_velocity_char, data.yaw_velocity);
        getBytesFromFloat(data_buf.pitch_angle_char, data.pitch_angle_rads);
        getBytesFromFloat(data_buf.pitch_velocity_char, data.pitch_velocity);

        // 0  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29    - 30 total bytes
        // EE x x x x y y y y r r  r  r  p  p  p  p  y  y  y  y  pv pv pv pv yv yv yv yv checksum
        //put the data into temp
        int startPositions[7] = {1, 5, 9, 13, 17, 21, 25};
        nucleo_value[0] = DATA_HEADER;
        copy4Char(data_buf.chassis_x_velocity_char, nucleo_value, startPositions[0]);
        copy4Char(data_buf.chassis_y_velocity_char, nucleo_value, startPositions[1]);
        copy4Char(data_buf.chassis_rotation_char, nucleo_value, startPositions[2]);
        copy4Char(data_buf.pitch_angle_char, nucleo_value, startPositions[3]);
        copy4Char(data_buf.yaw_angle_char, nucleo_value, startPositions[4]);
        copy4Char(data_buf.pitch_velocity_char, nucleo_value, startPositions[5]);
        copy4Char(data_buf.yaw_velocity_char, nucleo_value, startPositions[6]);


        uint8_t lrc = calculateLRC(nucleo_value + 1, data_buf.size); //exclude header byte
        char lrc_char = static_cast<uint8_t>(lrc);
        nucleo_value[data_buf.size + 1] = lrc_char;

        bcJetson.sync();
        bcJetson.write(nucleo_value, (data_buf.size + 2));
    }
    else {
        Jetson_send_ref_buf data_buf;

        getBytesFromInt8(data_buf.game_state, ref_data.game_state);
        getBytesFromInt16(data_buf.robot_hp, ref_data.robot_hp);

        // 0  1 2 3 4    - 5 total bytes
        // EF g h h checksum
        //put the data into temp
        int startPositions[2] = {1, 2};
        nucleo_value[0] = REF_HEADER;
        copy4Char(data_buf.game_state, nucleo_value, startPositions[0]);
        copy4Char(data_buf.robot_hp, nucleo_value, startPositions[1]);

        uint8_t lrc = calculateLRC(nucleo_value + 1, data_buf.size); //exclude header byte
        char lrc_char = static_cast<uint8_t>(lrc);
        nucleo_value[data_buf.size + 1] = lrc_char;

        Jetson_send_data_buf data_buf2;

        getBytesFromFloat(data_buf2.chassis_x_velocity_char, data.chassis_x_velocity);
        getBytesFromFloat(data_buf2.chassis_y_velocity_char, data.chassis_y_velocity);
        getBytesFromFloat(data_buf2.chassis_rotation_char, data.chassis_rotation);
        getBytesFromFloat(data_buf2.yaw_angle_char, data.yaw_angle_rads);
        getBytesFromFloat(data_buf2.yaw_velocity_char, data.yaw_velocity);
        getBytesFromFloat(data_buf2.pitch_angle_char, data.pitch_angle_rads);
        getBytesFromFloat(data_buf2.pitch_velocity_char, data.pitch_velocity);

        // 0  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29    - 30 total bytes
        // EE x x x x y y y y r r  r  r  p  p  p  p  y  y  y  y  pv pv pv pv yv yv yv yv checksum
        //put the data into temp
        int offset = data_buf.size + 2;
        int startPositions2[7] = {offset + 1, offset + 5, offset + 9, offset + 13, offset + 17, offset + 21, offset + 25};
        nucleo_value[offset] = DATA_HEADER;
        copy4Char(data_buf2.chassis_x_velocity_char, nucleo_value, startPositions2[0]);
        copy4Char(data_buf2.chassis_y_velocity_char, nucleo_value, startPositions2[1]);
        copy4Char(data_buf2.chassis_rotation_char, nucleo_value, startPositions2[2]);
        copy4Char(data_buf2.pitch_angle_char, nucleo_value, startPositions2[3]);
        copy4Char(data_buf2.yaw_angle_char, nucleo_value, startPositions2[4]);
        copy4Char(data_buf2.pitch_velocity_char, nucleo_value, startPositions2[5]);
        copy4Char(data_buf2.yaw_velocity_char, nucleo_value, startPositions2[6]);
        uint8_t lrc2 = calculateLRC(nucleo_value + offset + 1, data_buf2.size); //exclude header byte
        char lrc_char2 = static_cast<uint8_t>(lrc2);
        nucleo_value[offset + data_buf2.size + 1] = lrc_char2;

        bcJetson.sync();
        bcJetson.write(nucleo_value, (data_buf.size + 2) + (data_buf2.size + 2));
    }
}
/**
* Read desired pitch and yaw position data from Jetson
* 
* @param pitch_move buffer to store desired pitch position
* @param yaw_move buffer to store desired yaw position
*/
//TODO: remove printf's
ssize_t jetson_read_values(BufferedSerial &bcJetson, Jetson_read_data& read_data, Jetson_read_odom& odom_data) {
    if(!bcJetson.readable()) {
        return -1;
    }

    int available_space = JETSON_READ_BUFF_SIZE - jetson_read_buff_pos;
    if (available_space < JETSON_MAX_PACKET_SIZE) {
        jetson_read_buff_pos = 0;
        available_space = JETSON_MAX_PACKET_SIZE;
    }
    else if (available_space > JETSON_MAX_PACKET_SIZE) {
        available_space = JETSON_MAX_PACKET_SIZE;
    }

    ssize_t bytes_read = bcJetson.read(jetson_read_buff + jetson_read_buff_pos, available_space);

    if(bytes_read == -EAGAIN) {
        return -EAGAIN;
    }

    //error other than no data to read 
    if (bytes_read <= 0) {
        jetson_read_buff_pos = 0; //reset buffer
        return bytes_read; //return error code
    }

    if (bytes_read < JETSON_READ_MSG_SIZE) {
        jetson_read_buff_pos += bytes_read;
        return -1;
    }

    for (int i = 0; i < bytes_read; ++i) {
        if (jetson_read_buff[jetson_read_buff_pos + i] == AIM_HEADER) {
            // process aim data
            if (jetson_read_buff_pos + i + 10 <= JETSON_READ_BUFF_SIZE) {
                uint8_t checksum = jetson_read_buff[jetson_read_buff_pos + i + 10];
                uint8_t calculated_checksum = calculateLRC(&jetson_read_buff[jetson_read_buff_pos + i + 1], 9);

                if (checksum != calculated_checksum) {
                    // checksum mismatch, skip this packet
                    continue;
                }
                
                memcpy(&read_data.requested_pitch_rads, &jetson_read_buff[jetson_read_buff_pos + i + 1], sizeof(float));
                memcpy(&read_data.requested_yaw_rads, &jetson_read_buff[jetson_read_buff_pos + i + 5], sizeof(float));
                memcpy(&read_data.shoot_status, &jetson_read_buff[jetson_read_buff_pos + i + 9], sizeof(char));
                jetson_read_buff_pos += bytes_read;

                return 1;
            }
        }
        else if (jetson_read_buff[jetson_read_buff_pos + i] == ODOM_HEADER) {
            // process odom data
            if (jetson_read_buff_pos + i + 13 <= JETSON_READ_BUFF_SIZE) {
                uint8_t checksum = jetson_read_buff[jetson_read_buff_pos + i + 14];
                uint8_t calculated_checksum = calculateLRC(&jetson_read_buff[jetson_read_buff_pos + i + 1], 13);

                if (checksum != calculated_checksum) {
                    // checksum mismatch, skip this packet
                    continue;
                }

                memcpy(&odom_data.x_vel, &jetson_read_buff[jetson_read_buff_pos + i + 1], sizeof(float));
                memcpy(&odom_data.y_vel, &jetson_read_buff[jetson_read_buff_pos + i + 5], sizeof(float));
                memcpy(&odom_data.rotation, &jetson_read_buff[jetson_read_buff_pos + i + 9], sizeof(float));
                memcpy(&odom_data.calibration, &jetson_read_buff[jetson_read_buff_pos + i + 13], sizeof(char));
                jetson_read_buff_pos += bytes_read;
                
                return 2;
            }
        }
    }

    jetson_read_buff_pos += bytes_read;

    return -1;
}


/**
 * Read packets from Jetson for aiming and odometry,
 * as well as writing to Jetson from Chassis and turret data.
 * 
 * @param spiJetson SPI object for read/write
 * @param ref_data Jetson_send_ref object; ref info
 * @param data Jetson_send_data object; chassis info
 * @param read_data Jetson_read_data object; desired turret position
 * @param odom_data Jetson_read_odom object; desired chassis position
 * 
 * @return number of bytes read, -1 if fail
 */
ssize_t jetson_send_read_spi(SPI &spiJetson, const Jetson_send_ref& ref_data, const Jetson_send_data& data, Jetson_read_data& read_data, Jetson_read_odom& odom_data) {
    Jetson_send_ref_buf data_buf;

    getBytesFromInt8(data_buf.game_state, ref_data.game_state);
    getBytesFromInt16(data_buf.robot_hp, ref_data.robot_hp);

    // 0  1 2 3 4    - 5 total bytes
    // EF g h h checksum
    //put the data into temp
    int startPositions[2] = {1, 2};
    nucleo_value[0] = REF_HEADER;
    copy4Char(data_buf.game_state, nucleo_value, startPositions[0]);
    copy4Char(data_buf.robot_hp, nucleo_value, startPositions[1]);

    uint8_t lrc = calculateLRC(nucleo_value + 1, data_buf.size); //exclude header byte
    char lrc_char = static_cast<uint8_t>(lrc);
    nucleo_value[data_buf.size + 1] = lrc_char;

    Jetson_send_data_buf data_buf2;

    getBytesFromFloat(data_buf2.chassis_x_velocity_char, data.chassis_x_velocity);
    getBytesFromFloat(data_buf2.chassis_y_velocity_char, data.chassis_y_velocity);
    getBytesFromFloat(data_buf2.chassis_rotation_char, data.chassis_rotation);
    getBytesFromFloat(data_buf2.yaw_angle_char, data.yaw_angle_rads);
    getBytesFromFloat(data_buf2.yaw_velocity_char, data.yaw_velocity);
    getBytesFromFloat(data_buf2.pitch_angle_char, data.pitch_angle_rads);
    getBytesFromFloat(data_buf2.pitch_velocity_char, data.pitch_velocity);

    // 0  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29    - 30 total bytes
    // EE x x x x y y y y r r  r  r  p  p  p  p  y  y  y  y  pv pv pv pv yv yv yv yv checksum
    //put the data into temp
    int offset = data_buf.size + 2;
    int startPositions2[7] = {offset + 1, offset + 5, offset + 9, offset + 13, offset + 17, offset + 21, offset + 25};
    nucleo_value[offset] = DATA_HEADER;
    copy4Char(data_buf2.chassis_x_velocity_char, nucleo_value, startPositions2[0]);
    copy4Char(data_buf2.chassis_y_velocity_char, nucleo_value, startPositions2[1]);
    copy4Char(data_buf2.chassis_rotation_char, nucleo_value, startPositions2[2]);
    copy4Char(data_buf2.pitch_angle_char, nucleo_value, startPositions2[3]);
    copy4Char(data_buf2.yaw_angle_char, nucleo_value, startPositions2[4]);
    copy4Char(data_buf2.pitch_velocity_char, nucleo_value, startPositions2[5]);
    copy4Char(data_buf2.yaw_velocity_char, nucleo_value, startPositions2[6]);
    uint8_t lrc2 = calculateLRC(nucleo_value + offset + 1, data_buf2.size); //exclude header byte
    char lrc_char2 = static_cast<uint8_t>(lrc2);
    nucleo_value[offset + data_buf2.size + 1] = lrc_char2;

    int available_space = JETSON_READ_BUFF_SIZE - jetson_read_buff_pos;
    if (available_space < JETSON_MAX_PACKET_SIZE) {
        jetson_read_buff_pos = 0;
        available_space = JETSON_MAX_PACKET_SIZE;
    }
    else if (available_space > JETSON_MAX_PACKET_SIZE) {
        available_space = JETSON_MAX_PACKET_SIZE;
    }

    ssize_t bytes_read = spiJetson.write(nucleo_value, (ref_data.size + data.size + 4), jetson_read_buff + jetson_read_buff_pos, available_space);

    if(bytes_read == -EAGAIN) {
        return -EAGAIN;
    }

    //error other than no data to read 
    if (bytes_read <= 0) {
        jetson_read_buff_pos = 0; //reset buffer
        return bytes_read; //return error code
    }

    if (bytes_read < JETSON_READ_MSG_SIZE) {
        jetson_read_buff_pos += bytes_read;
        return -1;
    }

    for (int i = 0; i < bytes_read; ++i) {
        if (jetson_read_buff[jetson_read_buff_pos + i] == AIM_HEADER) {
            // process aim data
            if (jetson_read_buff_pos + i + 10 <= JETSON_READ_BUFF_SIZE) {
                uint8_t checksum = jetson_read_buff[jetson_read_buff_pos + i + 10];
                uint8_t calculated_checksum = calculateLRC(&jetson_read_buff[jetson_read_buff_pos + i + 1], 9);

                if (checksum != calculated_checksum) {
                    // checksum mismatch, skip this packet
                    continue;
                }
                
                memcpy(&read_data.requested_pitch_rads, &jetson_read_buff[jetson_read_buff_pos + i + 1], sizeof(float));
                memcpy(&read_data.requested_yaw_rads, &jetson_read_buff[jetson_read_buff_pos + i + 5], sizeof(float));
                memcpy(&read_data.shoot_status, &jetson_read_buff[jetson_read_buff_pos + i + 9], sizeof(char));
                jetson_read_buff_pos += bytes_read;

                return 1;
            }
        }
        else if (jetson_read_buff[jetson_read_buff_pos + i] == ODOM_HEADER) {
            // process odom data
            if (jetson_read_buff_pos + i + 13 <= JETSON_READ_BUFF_SIZE) {
                uint8_t checksum = jetson_read_buff[jetson_read_buff_pos + i + 14];
                uint8_t calculated_checksum = calculateLRC(&jetson_read_buff[jetson_read_buff_pos + i + 1], 13);

                if (checksum != calculated_checksum) {
                    // checksum mismatch, skip this packet
                    continue;
                }

                memcpy(&odom_data.x_vel, &jetson_read_buff[jetson_read_buff_pos + i + 1], sizeof(float));
                memcpy(&odom_data.y_vel, &jetson_read_buff[jetson_read_buff_pos + i + 5], sizeof(float));
                memcpy(&odom_data.rotation, &jetson_read_buff[jetson_read_buff_pos + i + 9], sizeof(float));
                memcpy(&odom_data.calibration, &jetson_read_buff[jetson_read_buff_pos + i + 13], sizeof(char));
                jetson_read_buff_pos += bytes_read;
                
                return 2;
            }
        }
    }

    jetson_read_buff_pos += bytes_read;

    return -1;
}