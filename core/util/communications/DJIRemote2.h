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
	static constexpr float STICK_MAX_VALUE = 1684.0f;

    DJIRemote2(PinName tx, PinName rx, int baud = 921600);

    bool update();                      // returns true when a full new frame is decoded
    void clear();

    const VTMInput& getData() const;
    bool hasValidFrame() const;

    uint64_t getLastFrameTimeUs() const;
    uint64_t getFramePeriodUs() const;
    double getFrameRateHz() const;

	// specifies a particular joystick
	enum class Joystick
    {
        RIGHT_HORIZONTAL,
        RIGHT_VERTICAL,
        LEFT_HORIZONTAL,
        LEFT_VERTICAL
    };

	enum class Button
	{
		TRGR = 0,
		CUSTL,
		CUSTR,
		PAUSE
	};

	// specifies a particular mode 
	enum class ModeSwitch
    {
        MODE_C,
        MODE_N,
		MODE_S
    };

	// A list of the particular keys to interact with, in bit order.
	enum class Key
    {
        W = 0,
        S,
        A,
        D,
        SHIFT,
        CTRL,
        Q,
        E,
        R,
        F,
        G,
        Z,
        X,
        C,
        V,
        B
    };

private:
    BufferedSerial serial_;

    uint8_t streamBuffer_[STREAM_BUFFER_SIZE];
    size_t streamCount_;

    VTMInput data_;
    bool validFrame_;

    uint64_t lastFrameTimeUs_;
    uint64_t currentFrameTimeUs_;
    uint64_t framePeriodUs_;

	ModeSwitch getMode() const;

    void readIncomingBytes();
    bool tryParseFrame();
    int findHeader() const;
    void decodeFrame(const uint8_t* frame);
    void shiftLeft(size_t count);
	float getJoystickValue(Joystick joy) const;
	float apply_deadzone(float value) const;
	float getDialValue() const;
};

#endif