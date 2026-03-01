// #include "DJIRemote2.h"

// #include <cstring>
// #include <algorithm>
// #include <us_ticker_api.h>

// DJIRemote2::DJIRemote2(PinName tx, PinName rx, int baud)
//     : serial_(tx, rx, baud),
//       streamCount_(0),
//       validFrame_(false),
//       lastFrameTimeUs_(0),
//       currentFrameTimeUs_(0),
//       framePeriodUs_(0)
// {
//     serial_.set_blocking(false);
//     std::memset(streamBuffer_, 0, sizeof(streamBuffer_));
// }

// bool DJIRemote2::update()
// {
//     readIncomingBytes();
//     return tryParseFrame();
// }

// void DJIRemote2::clear()
// {
//     streamCount_ = 0;
//     validFrame_ = false;
//     framePeriodUs_ = 0;
//     lastFrameTimeUs_ = 0;
//     currentFrameTimeUs_ = 0;
//     std::memset(streamBuffer_, 0, sizeof(streamBuffer_));
//     data_ = VTMInput{};
// }

// const VTMInput& DJIRemote2::getData() const
// {
//     return data_;
// }

// bool DJIRemote2::hasValidFrame() const
// {
//     return validFrame_;
// }

// uint64_t DJIRemote2::getLastFrameTimeUs() const
// {
//     return currentFrameTimeUs_;
// }

// uint64_t DJIRemote2::getFramePeriodUs() const
// {
//     return framePeriodUs_;
// }

// double DJIRemote2::getFrameRateHz() const
// {
//     if (framePeriodUs_ == 0) {
//         return 0.0;
//     }
//     return 1000000.0 / static_cast<double>(framePeriodUs_);
// }

// void DJIRemote2::readIncomingBytes()
// {
//     uint8_t temp[32];

//     while (serial_.readable()) {
//         ssize_t bytesRead = serial_.read(temp, sizeof(temp));
//         if (bytesRead <= 0) {
//             break;
//         }

//         size_t n = static_cast<size_t>(bytesRead);

//         // If incoming data is larger than buffer, keep only the newest bytes
//         if (n >= STREAM_BUFFER_SIZE) {
//             std::memcpy(streamBuffer_, temp + (n - STREAM_BUFFER_SIZE), STREAM_BUFFER_SIZE);
//             streamCount_ = STREAM_BUFFER_SIZE;
//             continue;
//         }

//         // Make room if needed by dropping the oldest bytes
//         if (streamCount_ + n > STREAM_BUFFER_SIZE) {
//             size_t overflow = (streamCount_ + n) - STREAM_BUFFER_SIZE;
//             shiftLeft(overflow);
//         }

//         std::memcpy(streamBuffer_ + streamCount_, temp, n);
//         streamCount_ += n;
//     }
// }

// bool DJIRemote2::tryParseFrame()
// {
//     while (streamCount_ >= 2) {
//         int headerIndex = findHeader();

//         // No header found, keep only last byte in case next byte completes header
//         if (headerIndex < 0) {
//             if (streamCount_ > 1) {
//                 streamBuffer_[0] = streamBuffer_[streamCount_ - 1];
//                 streamCount_ = 1;
//             }
//             return false;
//         }

//         // Drop junk before header
//         if (headerIndex > 0) {
//             shiftLeft(static_cast<size_t>(headerIndex));
//         }

//         // Need a full frame
//         if (streamCount_ < FRAME_SIZE) {
//             return false;
//         }

//         decodeFrame(streamBuffer_);

//         currentFrameTimeUs_ = static_cast<uint64_t>(us_ticker_read());
//         if (lastFrameTimeUs_ != 0) {
//             framePeriodUs_ = currentFrameTimeUs_ - lastFrameTimeUs_;
//         }
//         lastFrameTimeUs_ = currentFrameTimeUs_;
//         validFrame_ = true;

//         // Remove parsed frame
//         shiftLeft(FRAME_SIZE);
//         return true;
//     }

//     return false;
// }

// int DJIRemote2::findHeader() const
// {
//     for (size_t i = 0; i + 1 < streamCount_; i++) {
//         if (streamBuffer_[i] == HEADER_BYTE_0 && streamBuffer_[i + 1] == HEADER_BYTE_1) {
//             return static_cast<int>(i);
//         }
//     }
//     return -1;
// }

// void DJIRemote2::decodeFrame(const uint8_t* frame)
// {
//     data_.ch0 = static_cast<uint16_t>(frame[2])
//               | (static_cast<uint16_t>(frame[3] & 0x07) << 8);

//     data_.ch1 = ((static_cast<uint16_t>(frame[3]) >> 3) & 0x1F)
//               | ((static_cast<uint16_t>(frame[4]) & 0x3F) << 5);

//     data_.ch2 = ((static_cast<uint16_t>(frame[4]) >> 6) & 0x03)
//               | (static_cast<uint16_t>(frame[5]) << 2)
//               | ((static_cast<uint16_t>(frame[6]) & 0x01) << 10);

//     data_.ch3 = ((static_cast<uint16_t>(frame[6]) >> 1) & 0x7F)
//               | ((static_cast<uint16_t>(frame[7]) & 0x0F) << 7);

//     data_.mode  = (frame[7] >> 4) & 0x03;
//     data_.pause = (frame[7] >> 6) & 0x01;
//     data_.btnL  = (frame[7] >> 7) & 0x01;
//     data_.btnR  = frame[8] & 0x01;

//     data_.dial = ((static_cast<uint16_t>(frame[8]) >> 1) & 0x7F)
//                | ((static_cast<uint16_t>(frame[9]) & 0x0F) << 7);

//     data_.trigger = (frame[9] >> 4) & 0x01;

//     data_.mouseX = static_cast<int16_t>(
//           ((static_cast<uint16_t>(frame[9])  >> 5) & 0x07)
//         | ( static_cast<uint16_t>(frame[10])       << 3)
//         | ((static_cast<uint16_t>(frame[11]) & 0x1F) << 11)
//     );

//     data_.mouseY = static_cast<int16_t>(
//           ((static_cast<uint16_t>(frame[11]) >> 5) & 0x07)
//         | ( static_cast<uint16_t>(frame[12])       << 3)
//         | ((static_cast<uint16_t>(frame[13]) & 0x1F) << 11)
//     );

//     data_.mouseZ = static_cast<int16_t>(
//           ((static_cast<uint16_t>(frame[13]) >> 5) & 0x07)
//         | ( static_cast<uint16_t>(frame[14])       << 3)
//         | ((static_cast<uint16_t>(frame[15]) & 0x1F) << 11)
//     );

//     data_.mouseL = (frame[15] >> 5) & 0x03;

//     data_.mouseR = ((static_cast<uint16_t>(frame[15]) >> 7) & 0x01)
//                  | ((static_cast<uint16_t>(frame[16]) & 0x01) << 1);

//     data_.mouseM = (frame[16] >> 1) & 0x03;

//     data_.keyboard = ((static_cast<uint16_t>(frame[16]) >> 3) & 0x1F)
//                    | ( static_cast<uint16_t>(frame[17])       << 5)
//                    | ((static_cast<uint16_t>(frame[18]) & 0x07) << 13);

//     data_.CRC_in = ((static_cast<uint16_t>(frame[18]) >> 3) & 0x1F)
//                  | ( static_cast<uint16_t>(frame[19])       << 5)
//                  | ((static_cast<uint16_t>(frame[20]) & 0x07) << 13);
// }

// void DJIRemote2::shiftLeft(size_t count)
// {
//     if (count >= streamCount_) {
//         streamCount_ = 0;
//         return;
//     }

//     std::memmove(streamBuffer_, streamBuffer_ + count, streamCount_ - count);
//     streamCount_ -= count;
// }

#include "DJIRemote2.h"

#include <cstring>
#include <algorithm>
#include <us_ticker_api.h>

DJIRemote2::DJIRemote2(PinName tx, PinName rx, int baud)
    : serial_(tx, rx, baud),
      streamCount_(0),
      validFrame_(false),
      lastFrameTimeUs_(0),
      currentFrameTimeUs_(0),
      framePeriodUs_(0),
      crcValid_(false),
      receivedCRC_(0),
      computedCRC_(0)
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

    crcValid_ = false;
    receivedCRC_ = 0;
    computedCRC_ = 0;

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

bool DJIRemote2::isCRCValid() const
{
    return crcValid_;
}

uint16_t DJIRemote2::getReceivedCRC() const
{
    return receivedCRC_;
}

uint16_t DJIRemote2::getComputedCRC() const
{
    return computedCRC_;
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

        if (n >= STREAM_BUFFER_SIZE) {
            std::memcpy(streamBuffer_, temp + (n - STREAM_BUFFER_SIZE), STREAM_BUFFER_SIZE);
            streamCount_ = STREAM_BUFFER_SIZE;
            continue;
        }

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

        if (headerIndex < 0) {
            if (streamCount_ > 1) {
                streamBuffer_[0] = streamBuffer_[streamCount_ - 1];
                streamCount_ = 1;
            }
            validFrame_ = false;
            return false;
        }

        if (headerIndex > 0) {
            shiftLeft(static_cast<size_t>(headerIndex));
        }

        if (streamCount_ < FRAME_SIZE) {
            validFrame_ = false;
            return false;
        }

        // Verify, but do not reject yet while testing
        verifyCRC(streamBuffer_);

        decodeFrame(streamBuffer_);

        currentFrameTimeUs_ = static_cast<uint64_t>(us_ticker_read());
        if (lastFrameTimeUs_ != 0) {
            framePeriodUs_ = currentFrameTimeUs_ - lastFrameTimeUs_;
        }
        lastFrameTimeUs_ = currentFrameTimeUs_;
        validFrame_ = true;

        shiftLeft(FRAME_SIZE);
        return true;
    }

    validFrame_ = false;
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

uint16_t DJIRemote2::extractBitsLSB(const uint8_t* bytes, size_t startBit, size_t bitCount) const
{
    uint16_t value = 0;

    for (size_t i = 0; i < bitCount; i++) {
        size_t bitPos = startBit + i;
        uint8_t bit = (bytes[bitPos / 8] >> (bitPos % 8)) & 0x01;
        value |= static_cast<uint16_t>(bit) << i;
    }

    return value;
}

uint16_t DJIRemote2::computeCRC16CCITTFALSE(const uint8_t* bytes, size_t startBit, size_t bitCount) const
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < bitCount; i++) {
        size_t bitPos = startBit + i;

        uint8_t dataBit = (bytes[bitPos / 8] >> (bitPos % 8)) & 0x01;

        uint8_t crcMsb = (crc >> 15) & 0x01;
        crc = static_cast<uint16_t>(crc << 1);

        if (crcMsb ^ dataBit) {
            crc ^= 0x1021;
        }
    }

    return crc; // no XOR-out for CCITT-FALSE
}

bool DJIRemote2::verifyCRC(const uint8_t* frame)
{
    receivedCRC_ = extractBitsLSB(frame, CRC_FIELD_START_BIT, CRC_FIELD_BIT_COUNT);
    computedCRC_ = computeCRC16CCITTFALSE(frame, CRC_DATA_START_BIT, CRC_DATA_BIT_COUNT);
    crcValid_ = (receivedCRC_ == computedCRC_);
    return crcValid_;
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