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

	// check if trigger is pressed
	bool TriggerPressed() const;
	// check if custom left button is pressed
	bool CUSTLPressed() const;
	// check if custom right button is pressed
	bool CUSTRPressed() const;
	// check if pause button is pressed
	bool PAUSEPressed() const;
	
    /**
     * @return The current mouse x value.
     */
    int16_t getMouseX() const;

    /**
     * @return The current mouse y value.
     */
    int16_t getMouseY() const;

    /**
     * @return The current mouse z value.
     */
    int16_t getMouseZ() const;

    /**
     * @return The current mouse l value.
     */
    bool getMouseL() const;

    /**
     * @return The current mouse r value.
     */
    bool getMouseR() const;

    /**
     * @return `true` if the given `key` is pressed, `false` otherwise.
     */
    bool keyPressed(Key key) const;

    /**
     * @return the value of the wheel, between `[-STICK_MAX_VALUE, STICK_MAX_VALUE]`.
     */
    int16_t getWheel() const;

	bool CUSTLToggled() const;
	bool CUSTRToggled() const;
	bool PAUSEToggled() const;

	float getJoystickValue(Joystick joy) const;
	float apply_deadzone(float value) const;
	float getDialValue() const;
	ModeSwitch getMode() const;

private:
    BufferedSerial serial_;

    uint8_t streamBuffer_[STREAM_BUFFER_SIZE];
    size_t streamCount_;

    VTMInput data_;
    bool validFrame_;

    uint64_t lastFrameTimeUs_;
    uint64_t currentFrameTimeUs_;
    uint64_t framePeriodUs_;


    void readIncomingBytes();
    bool tryParseFrame();
    int findHeader() const;
    void decodeFrame(const uint8_t* frame);
    void shiftLeft(size_t count);
};

#endif