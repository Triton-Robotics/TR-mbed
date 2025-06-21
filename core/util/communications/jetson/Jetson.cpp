#include "Jetson.h"
#define MAGICBYTE 0xEE
#define JETSON_READ_BUFF_SIZE 500
#define JETSON_READ_MSG_SIZE 11
char nucleo_value[30] = {0};
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
void jetson_send_feedback(BufferedSerial &bcJetson, const Jetson_send_data& data) {
  char chassis_x_velocity_char[4];
  char chassis_y_velocity_char[4];
  char yaw_angle_char[4];
  char yaw_velocity_char[4];
  char pitch_angle_char[4];
  char pitch_velocity_char[4];

  getBytesFromFloat(chassis_x_velocity_char, data.chassis_x_velocity);
  getBytesFromFloat(chassis_y_velocity_char, data.chassis_y_velocity);
  getBytesFromFloat(yaw_angle_char, data.yaw_angle_rads);
  getBytesFromFloat(yaw_velocity_char, data.yaw_velocity);
  getBytesFromFloat(pitch_angle_char, data.pitch_angle_rads);
  getBytesFromFloat(pitch_velocity_char, data.pitch_velocity);

  // 0  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25    - 26 total bytes
  // EE x x x x y y y y p p  p  p  y  y  y  y  pv pv pv pv yv yv yv yv checksum
  //put the data into temp
  int startPositions[6] = {1, 5, 9, 13, 17, 21};
  nucleo_value[0] = MAGICBYTE;
  copy4Char(chassis_x_velocity_char, nucleo_value, startPositions[0]);
  copy4Char(chassis_y_velocity_char, nucleo_value, startPositions[1]);
  copy4Char(pitch_angle_char, nucleo_value, startPositions[2]);
  copy4Char(yaw_angle_char, nucleo_value, startPositions[3]);
  copy4Char(pitch_velocity_char, nucleo_value, startPositions[4]);
  copy4Char(yaw_velocity_char, nucleo_value, startPositions[5]);


  uint8_t lrc = calculateLRC(nucleo_value + 1, 24); //exclude header byte
  char lrc_char = static_cast<uint8_t>(lrc);
  nucleo_value[25] = lrc_char;

  //TODO: do we need this?
  bcJetson.set_blocking(false);
  bcJetson.write(nucleo_value, 26); //changed from 30
}


/**
* Read desired pitch and yaw position data from Jetson
* 
* @param pitch_move buffer to store desired pitch position
* @param yaw_move buffer to store desired yaw position
*/
//TODO: remove printf's
ssize_t jetson_read_values(BufferedSerial &bcJetson, Jetson_read_data& read_data) {
  //TODO: do we need this?
  bcJetson.set_blocking(false);

  if(!bcJetson.readable()){
    return -EAGAIN;
  }

  if(jetson_read_buff_pos > (JETSON_READ_BUFF_SIZE - JETSON_READ_MSG_SIZE)){
    printf("WARN: jetson read buffer overflow. Resetting buffer to 0\n");
    jetson_read_buff_pos = 0;
  }

  //TODO: keep a persistent buffer where if no matches are found we keep appending to the buffer until we find a match
  int available_space = JETSON_READ_BUFF_SIZE - jetson_read_buff_pos;
  ssize_t bytes_read = bcJetson.read(jetson_read_buff + jetson_read_buff_pos, available_space);
  if(bytes_read == -EAGAIN){
    return -EAGAIN;
  }

  //error other than no data to read 
  if(bytes_read <= 0){
    jetson_read_buff_pos = 0; //reset buffer
    return bytes_read; //return error code
  }

  jetson_read_buff_pos += bytes_read;

  if(jetson_read_buff_pos < JETSON_READ_MSG_SIZE){
    return -1;
      }

  //starting at the very last index of jetson_values (numbytes - 1)
  //i - 10 is the magic byte, and we will use next 9 bytes for checksum which is at i; exclude magic byte in checksum
  //if we meet the match conditions, decode values, clear buffer, return last amount of bytes read
  //if no match found, print no match
  //if buffer empty, print empty
  for(int i = jetson_read_buff_pos - 1 ; i >= 10 ; --i) {
    //calculating checksum without magic header bytes
    //check for magic byte, check checksum != 0, check calculated checksum matches message checksum
    if ((jetson_read_buff[i-10] == MAGICBYTE) &&
        (jetson_read_buff[i] != 0) && 
        (calculateLRC(&jetson_read_buff[i - 9], 9) == jetson_read_buff[i])){

        uint8_t checkSum;
        decode_toSTM32(&jetson_read_buff[i-9], 
          read_data.requested_pitch_rads, 
          read_data.requested_yaw_rads, 
          read_data.shoot_status, 
          checkSum);
        //TODO: as an optimization we can clear onto the message we extracted. Leaving any potential partial messages in the buffer
        jetson_read_buff_pos = 0;
        return 1;
    }
  }

  return -1;
}