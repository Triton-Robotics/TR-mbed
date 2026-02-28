#include "DJIRemote2.h"
#include <us_ticker_api.h>

DJIRemote2::DJIRemote2(PinName tx, PinName rx, int baud)
    : serial(tx, rx, baud)
{
    int i;

    streamCount = 0;
    validFrame = false;

    lastFrameTimeUs = 0;
    currentFrameTimeUs = 0;
    framePeriodUs = 0;

    for (i = 0; i < STREAM_BUFFER_SIZE; i++)
    {
        streamBuffer[i] = 0;
    }

    resetData();
}

void DJIRemote2::resetData()
{
    data.ch0 = 0;
    data.ch1 = 0;
    data.ch2 = 0;
    data.ch3 = 0;

    data.mode = 0;
    data.pause = 0;
    data.btnL = 0;
    data.btnR = 0;

    data.dial = 0;
    data.trigger = 0;

    data.mouseX = 0;
    data.mouseY = 0;
    data.mouseZ = 0;

    data.mouseL = 0;
    data.mouseR = 0;
    data.mouseM = 0;

    data.keyboard = 0;
    data.CRC_in = 0;
}

void DJIRemote2::clear()
{
    int i;

    streamCount = 0;
    validFrame = false;

    lastFrameTimeUs = 0;
    currentFrameTimeUs = 0;
    framePeriodUs = 0;

    for (i = 0; i < STREAM_BUFFER_SIZE; i++)
    {
        streamBuffer[i] = 0;
    }

    resetData();
}

bool DJIRemote2::update()
{
    readIncomingBytes();
    return parseOneFrame();
}

VTMInput DJIRemote2::getData() const
{
    return data;
}

bool DJIRemote2::hasFrame() const
{
    return validFrame;
}

uint64_t DJIRemote2::getFramePeriodUs() const
{
    return framePeriodUs;
}

float DJIRemote2::getFrameRateHz() const
{
    if (framePeriodUs == 0)
    {
        return 0.0f;
    }

    return 1000000.0f / (float)framePeriodUs;
}

void DJIRemote2::readIncomingBytes()
{
    char temp[READ_CHUNK_SIZE];

    while (serial.readable())
    {
        int bytesRead = (int)serial.read(temp, READ_CHUNK_SIZE);

        if (bytesRead <= 0)
        {
            break;
        }

        if (bytesRead >= STREAM_BUFFER_SIZE)
        {
            int startIndex = bytesRead - STREAM_BUFFER_SIZE;
            int i;

            for (i = 0; i < STREAM_BUFFER_SIZE; i++)
            {
                streamBuffer[i] = (uint8_t)temp[startIndex + i];
            }

            streamCount = STREAM_BUFFER_SIZE;
            continue;
        }

        if (streamCount + bytesRead > STREAM_BUFFER_SIZE)
        {
            int overflow = (streamCount + bytesRead) - STREAM_BUFFER_SIZE;
            shiftLeft(overflow);
        }

        for (int i = 0; i < bytesRead; i++)
        {
            streamBuffer[streamCount + i] = (uint8_t)temp[i];
        }

        streamCount += bytesRead;
    }
}

bool DJIRemote2::parseOneFrame()
{
    while (streamCount >= 2)
    {
        int headerIndex = findHeader();

        if (headerIndex < 0)
        {
            if (streamCount > 1)
            {
                streamBuffer[0] = streamBuffer[streamCount - 1];
                streamCount = 1;
            }
            return false;
        }

        if (headerIndex > 0)
        {
            shiftLeft(headerIndex);
        }

        if (streamCount < FRAME_SIZE)
        {
            return false;
        }

        decodeFrame(streamBuffer);

        currentFrameTimeUs = (uint64_t)us_ticker_read();

        if (lastFrameTimeUs != 0)
        {
            framePeriodUs = currentFrameTimeUs - lastFrameTimeUs;
        }

        lastFrameTimeUs = currentFrameTimeUs;
        validFrame = true;

        shiftLeft(FRAME_SIZE);
        return true;
    }

    return false;
}

int DJIRemote2::findHeader() const
{
    int i;

    for (i = 0; i < streamCount - 1; i++)
    {
        if (streamBuffer[i] == 0xA9 && streamBuffer[i + 1] == 0x53)
        {
            return i;
        }
    }

    return -1;
}

void DJIRemote2::decodeFrame(const uint8_t *frame)
{
    data.ch0 = ((uint16_t)frame[2]) | (((uint16_t)(frame[3] & 0x07)) << 8);

    data.ch1 = ((((uint16_t)frame[3]) >> 3) & 0x1F)
             | ((((uint16_t)frame[4]) & 0x3F) << 5);

    data.ch2 = ((((uint16_t)frame[4]) >> 6) & 0x03)
             | (((uint16_t)frame[5]) << 2)
             | ((((uint16_t)frame[6]) & 0x01) << 10);

    data.ch3 = ((((uint16_t)frame[6]) >> 1) & 0x7F)
             | ((((uint16_t)frame[7]) & 0x0F) << 7);

    data.mode  = (frame[7] >> 4) & 0x03;
    data.pause = (frame[7] >> 6) & 0x01;
    data.btnL  = (frame[7] >> 7) & 0x01;
    data.btnR  = frame[8] & 0x01;

    data.dial = ((((uint16_t)frame[8]) >> 1) & 0x7F)
              | ((((uint16_t)frame[9]) & 0x0F) << 7);

    data.trigger = (frame[9] >> 4) & 0x01;

    data.mouseX = (int16_t)(
          ((((uint16_t)frame[9])  >> 5) & 0x07)
        | (((uint16_t)frame[10]) << 3)
        | ((((uint16_t)frame[11]) & 0x1F) << 11)
    );

    data.mouseY = (int16_t)(
          ((((uint16_t)frame[11]) >> 5) & 0x07)
        | (((uint16_t)frame[12]) << 3)
        | ((((uint16_t)frame[13]) & 0x1F) << 11)
    );

    data.mouseZ = (int16_t)(
          ((((uint16_t)frame[13]) >> 5) & 0x07)
        | (((uint16_t)frame[14]) << 3)
        | ((((uint16_t)frame[15]) & 0x1F) << 11)
    );

    data.mouseL = (frame[15] >> 5) & 0x03;

    data.mouseR = ((((uint16_t)frame[15]) >> 7) & 0x01)
                | ((((uint16_t)frame[16]) & 0x01) << 1);

    data.mouseM = (frame[16] >> 1) & 0x03;

    data.keyboard = ((((uint16_t)frame[16]) >> 3) & 0x1F)
                  | (((uint16_t)frame[17]) << 5)
                  | ((((uint16_t)frame[18]) & 0x07) << 13);

    data.CRC_in = ((((uint16_t)frame[18]) >> 3) & 0x1F)
                | (((uint16_t)frame[19]) << 5)
                | ((((uint16_t)frame[20]) & 0x07) << 13);
}

void DJIRemote2::shiftLeft(int count)
{
    if (count >= streamCount)
    {
        streamCount = 0;
        return;
    }

    int newCount = streamCount - count;
    int i;

    for (i = 0; i < newCount; i++)
    {
        streamBuffer[i] = streamBuffer[i + count];
    }

    for (i = newCount; i < streamCount; i++)
    {
        streamBuffer[i] = 0;
    }

    streamCount = newCount;
}