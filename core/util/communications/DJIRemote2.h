#ifndef VTM_RECEIVER_H
#define VTM_RECEIVER_H

#include "mbed.h"
#include <stdint.h>

struct VTMInput
{
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;

    uint8_t mode;
    uint8_t pause;
    uint8_t btnL;
    uint8_t btnR;

    uint16_t dial;
    uint8_t trigger;

    int16_t mouseX;
    int16_t mouseY;
    int16_t mouseZ;

    uint8_t mouseL;
    uint8_t mouseR;
    uint8_t mouseM;

    uint16_t keyboard;
    uint16_t CRC_in;
};

class DJIRemote2
{
public:
    enum
    {
        FRAME_SIZE = 21,
        STREAM_BUFFER_SIZE = 64,
        READ_CHUNK_SIZE = 32
    };

    DJIRemote2(PinName tx, PinName rx, int baud = 921600);

    bool update();
    VTMInput getData() const;
    bool hasFrame() const;

    uint64_t getFramePeriodUs() const;
    float getFrameRateHz() const;

    void clear();

private:
    BufferedSerial serial;

    uint8_t streamBuffer[STREAM_BUFFER_SIZE];
    int streamCount;

    VTMInput data;
    bool validFrame;

    uint64_t lastFrameTimeUs;
    uint64_t currentFrameTimeUs;
    uint64_t framePeriodUs;

    void resetData();
    void readIncomingBytes();
    bool parseOneFrame();
    int findHeader() const;
    void decodeFrame(const uint8_t *frame);
    void shiftLeft(int count);
};

#endif