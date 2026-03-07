// #ifndef VTM_RECEIVER_H
// #define VTM_RECEIVER_H

// #include "mbed.h"
// #include <cstdint>
// #include <cstddef>

// struct VTMInput {
//     uint16_t ch0 = 0;
//     uint16_t ch1 = 0;
//     uint16_t ch2 = 0;
//     uint16_t ch3 = 0;

//     uint8_t mode = 0;
//     uint8_t pause = 0;
//     uint8_t btnL = 0;
//     uint8_t btnR = 0;

//     uint16_t dial = 0;
//     uint8_t trigger = 0;

//     int16_t mouseX = 0;
//     int16_t mouseY = 0;
//     int16_t mouseZ = 0;

//     uint8_t mouseL = 0;
//     uint8_t mouseR = 0;
//     uint8_t mouseM = 0;

//     uint16_t keyboard = 0;
//     uint16_t CRC_in = 0;
// };

// class DJIRemote2 {
// public:
//     static constexpr uint8_t HEADER_BYTE_0 = 0xA9;
//     static constexpr uint8_t HEADER_BYTE_1 = 0x53;
//     static constexpr size_t FRAME_SIZE = 21;
//     static constexpr size_t STREAM_BUFFER_SIZE = 64;

//     DJIRemote2(PinName tx, PinName rx, int baud = 921600);

//     bool update();                      // returns true when a full new frame is decoded
//     void clear();

//     const VTMInput& getData() const;
//     bool hasValidFrame() const;

//     uint64_t getLastFrameTimeUs() const;
//     uint64_t getFramePeriodUs() const;
//     double getFrameRateHz() const;

// private:
//     BufferedSerial serial_;

//     uint8_t streamBuffer_[STREAM_BUFFER_SIZE];
//     size_t streamCount_;

//     VTMInput data_;
//     bool validFrame_;

//     uint64_t lastFrameTimeUs_;
//     uint64_t currentFrameTimeUs_;
//     uint64_t framePeriodUs_;

//     void readIncomingBytes();
//     bool tryParseFrame();
//     int findHeader() const;
//     void decodeFrame(const uint8_t* frame);
//     void shiftLeft(size_t count);
// };

// #endif

#ifndef VTM_RECEIVER_H
#define VTM_RECEIVER_H

#include "mbed.h"
#include <cstdint>
#include <cstddef>

struct VTMInput {
    uint16_t ch0 = 0;
    uint16_t ch1 = 0;
    uint16_t ch2 = 0;
    uint16_t ch3 = 0;

    uint8_t mode = 0;
    uint8_t pause = 0;
    uint8_t btnL = 0;
    uint8_t btnR = 0;

    uint16_t dial = 0;
    uint8_t trigger = 0;

    int16_t mouseX = 0;
    int16_t mouseY = 0;
    int16_t mouseZ = 0;

    uint8_t mouseL = 0;
    uint8_t mouseR = 0;
    uint8_t mouseM = 0;

    uint16_t keyboard = 0;
    uint16_t CRC_in = 0;
};

class DJIRemote2 {
public:
    static constexpr uint8_t HEADER_BYTE_0 = 0xA9;
    static constexpr uint8_t HEADER_BYTE_1 = 0x53;
    static constexpr size_t FRAME_SIZE = 21;
    static constexpr size_t STREAM_BUFFER_SIZE = 64;

    DJIRemote2(PinName tx, PinName rx, int baud = 921600);

    bool update();                      // returns true when a full new valid frame is decoded
    void clear();

    const VTMInput& getData() const;
    bool hasValidFrame() const;

    uint64_t getLastFrameTimeUs() const;
    uint64_t getFramePeriodUs() const;
    double getFrameRateHz() const;

    bool isCRCValid() const;
    uint16_t getReceivedCRC() const;
    uint16_t getComputedCRC() const;

private:
    BufferedSerial serial_;

    uint8_t streamBuffer_[STREAM_BUFFER_SIZE];
    size_t streamCount_;

    VTMInput data_;
    bool validFrame_;

    uint64_t lastFrameTimeUs_;
    uint64_t currentFrameTimeUs_;
    uint64_t framePeriodUs_;

    bool crcValid_;
    uint16_t receivedCRC_;
    uint16_t computedCRC_;

    // Assumption:
    // CRC field starts at overall bit 147 and is 16 bits long.
    // That matches your current bit extraction:
    // keyboard uses bits up to frame[18] bit2, CRC starts at frame[18] bit3.
    static constexpr size_t CRC_FIELD_START_BIT = 147;
    static constexpr size_t CRC_FIELD_BIT_COUNT = 16;

    // Assumption:
    // CRC is computed over all bits before the CRC field, including the 2-byte header.
    // If your sender excludes the header, change CRC_DATA_START_BIT to 16.
    static constexpr size_t CRC_DATA_START_BIT = 16;
    static constexpr size_t CRC_DATA_BIT_COUNT = CRC_FIELD_START_BIT - CRC_DATA_START_BIT;

    // Default CRC flavor used here:
    // reflected CRC-16/CCITT style
    // If every frame fails, these 3 constants are the first thing to try changing.
    static constexpr uint16_t CRC_POLY = 0x8408;   // reflected form of 0x1021
	static constexpr uint16_t CRC_INIT = 0xFFFF;
	static constexpr uint16_t CRC_XOR_OUT = 0xFFFF;

    void readIncomingBytes();
    bool tryParseFrame();
    int findHeader() const;
    void decodeFrame(const uint8_t* frame);
    void shiftLeft(size_t count);

    bool verifyCRC(const uint8_t* frame);
    uint16_t extractBitsLSB(const uint8_t* bytes, size_t startBit, size_t bitCount) const;
    uint16_t computeCRC16(const uint8_t* bytes, size_t startBit, size_t bitCount) const;
};

#endif