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
#define JETSON_READ_TIMEOUT     1000    // ms
#define JETSON_READ_RATE        500     // ms per packet

/**
 * @brief A class for interfacing with the Jetson TX2 and receiving computer vision data over UART
 *
 * A full end to end example with threading can be seen here:
 *
 * ```
 * #include "mbed.h"
 * #include "communications/Jetson.h"
 *
 * Jetson jetson(PC_12, PD_2);
 * Thread cv;
 *
 * int main() {
 *   cv.start([]() {
 *     while (true) {
 *       jetson.read();
 *     }
 *   });
 *
 *   DigitalOut led(LED1);
 *   while (true) {
 *     led = !led;
 *     printf("Data: %f, %f, %f\n", jetson.get(Jetson::cv::X), jetson.get(Jetson::cv::Y), jetson.get(Jetson::cv::Z));
 *     printf("Status: %d\n", jetson.status.load());
 *     ThisThread::sleep_for(500);
 *   }
 *   return 0;
 * }
 * ```
 */
class Jetson {

private:
    UnbufferedSerial jetson;

    time_t lastRead = 0;

    uint8_t rxBuffer[JETSON_BUF_LEN]{0};
    uint8_t cobsBuffer[JETSON_BUF_LEN]{0};

    int currentBufferIndex = 0;

    atomic<float> cvX, cvY, cvZ;

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

    Timer readTimer;
    atomic<JetsonStatus> status;
};


#endif //TR_MBED_JETSON_H
