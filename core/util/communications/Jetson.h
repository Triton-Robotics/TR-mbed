//
// Created by ankit on 5/6/23.
//

#ifndef TR_MBED_JETSON_H
#define TR_MBED_JETSON_H

#include <mbed.h>
#include <atomic>

/**
 * Serial parser settings
 */
#define JETSON_BUF_LEN          64      // bytes
#define JETSON_TOPIC_ID         0x69
#define JETSON_COBS_HEADER_LEN  5       // bytes
#define JETSON_READ_TIMEOUT     250     // ms
#define JETSON_READ_RATE        100     // ms per packet

class Jetson {

private:
    UnbufferedSerial jetson;

    Timer readTimer;
    time_t lastRead = 0;

    uint8_t rxBuffer[JETSON_BUF_LEN]{0};
    uint8_t cobsBuffer[JETSON_BUF_LEN]{0};

    int currentBufferIndex = 0;

    atomic<float> cvX = 0, cvY = 0, cvZ = 0;

    void parse();
    void clear();
    void reset();

    template <typename T>
    T find(uint8_t* buf, size_t len, int index);

public:
    Jetson(PinName tx, PinName rx);

    enum cv{
        X,
        Y,
        Z
    };

    enum JetsonStatus {
        CONNECTED,
        DISCONNECTED
    };

    void read();
    float get(cv axis);

    atomic<JetsonStatus> status = DISCONNECTED;
};


#endif //TR_MBED_JETSON_H
