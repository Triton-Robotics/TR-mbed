#include "DJIRemote2.h"

#include <cstring>
#include <algorithm>
#include <us_ticker_api.h>
static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}









DJIRemote2::DJIRemote2(PinName tx, PinName rx, int baud)
    : serial_(tx, rx, baud),
      streamCount_(0),
      validFrame_(false),
      lastFrameTimeUs_(0),
      currentFrameTimeUs_(0),
      framePeriodUs_(0)
{
    serial_.set_blocking(false);
    std::memset(streamBuffer_, 0, sizeof(streamBuffer_));
}

bool DJIRemote2::update()
{
    readIncomingBytes();
    return tryParseFrame();
}

void DJIRemote2::clear()
{
    streamCount_ = 0;
    validFrame_ = false;
    framePeriodUs_ = 0;
    lastFrameTimeUs_ = 0;
    currentFrameTimeUs_ = 0;
    std::memset(streamBuffer_, 0, sizeof(streamBuffer_));
    data_ = VTMInput{};
}

const VTMInput& DJIRemote2::getData() const
{
    return data_;
}

bool DJIRemote2::hasValidFrame() const
{
    return validFrame_;
}

uint64_t DJIRemote2::getLastFrameTimeUs() const
{
    return currentFrameTimeUs_;
}

uint64_t DJIRemote2::getFramePeriodUs() const
{
    return framePeriodUs_;
}

double DJIRemote2::getFrameRateHz() const
{
    if (framePeriodUs_ == 0) {
        return 0.0;
    }
    return 1000000.0 / static_cast<double>(framePeriodUs_);
}

void DJIRemote2::readIncomingBytes()
{
    uint8_t temp[32];

    while (serial_.readable()) {
        ssize_t bytesRead = serial_.read(temp, sizeof(temp));
        if (bytesRead <= 0) {
            break;
        }

        size_t n = static_cast<size_t>(bytesRead);

        // If incoming data is larger than buffer, keep only the newest bytes
        if (n >= STREAM_BUFFER_SIZE) {
            std::memcpy(streamBuffer_, temp + (n - STREAM_BUFFER_SIZE), STREAM_BUFFER_SIZE);
            streamCount_ = STREAM_BUFFER_SIZE;
            continue;
        }

        // Make room if needed by dropping the oldest bytes
        if (streamCount_ + n > STREAM_BUFFER_SIZE) {
            size_t overflow = (streamCount_ + n) - STREAM_BUFFER_SIZE;
            shiftLeft(overflow);
        }

        std::memcpy(streamBuffer_ + streamCount_, temp, n);
        streamCount_ += n;
    }
}

bool DJIRemote2::tryParseFrame()
{
    while (streamCount_ >= 2) {
        int headerIndex = findHeader();

        // No header found, keep only last byte in case next byte completes header
        if (headerIndex < 0) {
            if (streamCount_ > 1) {
                streamBuffer_[0] = streamBuffer_[streamCount_ - 1];
                streamCount_ = 1;
            }
            return false;
        }

        // Drop junk before header
        if (headerIndex > 0) {
            shiftLeft(static_cast<size_t>(headerIndex));
        }

        // Need a full frame
        if (streamCount_ < FRAME_SIZE) {
            return false;
        }

        decodeFrame(streamBuffer_);

        // TODO: make this a real test and not just a print
        // printf("CRC: %d\n", verify_crc16_check_sum(streamBuffer_,21));

        currentFrameTimeUs_ = static_cast<uint64_t>(us_ticker_read());
        if (lastFrameTimeUs_ != 0) {
            framePeriodUs_ = currentFrameTimeUs_ - lastFrameTimeUs_;
        }
        lastFrameTimeUs_ = currentFrameTimeUs_;
        validFrame_ = true;

        // Remove parsed frame
        shiftLeft(FRAME_SIZE);
        return true;
    }

    return false;
}

int DJIRemote2::findHeader() const
{
    for (size_t i = 0; i + 1 < streamCount_; i++) {
        if (streamBuffer_[i] == HEADER_BYTE_0 && streamBuffer_[i + 1] == HEADER_BYTE_1) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void DJIRemote2::decodeFrame(const uint8_t* frame)
{
    data_.ch0 = static_cast<uint16_t>(frame[2])
              | (static_cast<uint16_t>(frame[3] & 0x07) << 8);

    data_.ch1 = ((static_cast<uint16_t>(frame[3]) >> 3) & 0x1F)
              | ((static_cast<uint16_t>(frame[4]) & 0x3F) << 5);

    data_.ch2 = ((static_cast<uint16_t>(frame[4]) >> 6) & 0x03)
              | (static_cast<uint16_t>(frame[5]) << 2)
              | ((static_cast<uint16_t>(frame[6]) & 0x01) << 10);

    data_.ch3 = ((static_cast<uint16_t>(frame[6]) >> 1) & 0x7F)
              | ((static_cast<uint16_t>(frame[7]) & 0x0F) << 7);

    data_.mode  = (frame[7] >> 4) & 0x03;
    data_.pause = (frame[7] >> 6) & 0x01;
    data_.btnL  = (frame[7] >> 7) & 0x01;
    data_.btnR  = frame[8] & 0x01;

    data_.dial = ((static_cast<uint16_t>(frame[8]) >> 1) & 0x7F)
               | ((static_cast<uint16_t>(frame[9]) & 0x0F) << 7);

    data_.trigger = (frame[9] >> 4) & 0x01;

    data_.mouseX = static_cast<int16_t>(
          ((static_cast<uint16_t>(frame[9])  >> 5) & 0x07)
        | ( static_cast<uint16_t>(frame[10])       << 3)
        | ((static_cast<uint16_t>(frame[11]) & 0x1F) << 11)
    );

    data_.mouseY = static_cast<int16_t>(
          ((static_cast<uint16_t>(frame[11]) >> 5) & 0x07)
        | ( static_cast<uint16_t>(frame[12])       << 3)
        | ((static_cast<uint16_t>(frame[13]) & 0x1F) << 11)
    );

    data_.mouseZ = static_cast<int16_t>(
          ((static_cast<uint16_t>(frame[13]) >> 5) & 0x07)
        | ( static_cast<uint16_t>(frame[14])       << 3)
        | ((static_cast<uint16_t>(frame[15]) & 0x1F) << 11)
    );

    data_.mouseL = (frame[15] >> 5) & 0x03;

    data_.mouseR = ((static_cast<uint16_t>(frame[15]) >> 7) & 0x01)
                 | ((static_cast<uint16_t>(frame[16]) & 0x01) << 1);

    data_.mouseM = (frame[16] >> 1) & 0x03;

    data_.keyboard = ((static_cast<uint16_t>(frame[16]) >> 3) & 0x1F)
                   | ( static_cast<uint16_t>(frame[17])       << 5)
                   | ((static_cast<uint16_t>(frame[18]) & 0x07) << 13);

    data_.CRC_in = ((static_cast<uint16_t>(frame[18]) >> 3) & 0x1F)
                 | ( static_cast<uint16_t>(frame[19])       << 5)
                 | ((static_cast<uint16_t>(frame[20]) & 0x07) << 13);
}

void DJIRemote2::shiftLeft(size_t count)
{
    if (count >= streamCount_) {
        streamCount_ = 0;
        return;
    }

    std::memmove(streamBuffer_, streamBuffer_ + count, streamCount_ - count);
    streamCount_ -= count;
}