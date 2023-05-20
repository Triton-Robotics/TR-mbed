//
// Created by ankit on 5/6/23.
//

#include "Jetson.h"

size_t cobs_unstuff_data(const uint8_t *ptr, size_t length, uint8_t *dst)
{
    const uint8_t *start = dst;
    const uint8_t *end = ptr + length;
    uint8_t code = 0xff;
    uint8_t copy = 0;

    for (; ptr < end; copy--) {
        if (copy != 0) {
            *dst++ = *ptr++;
        }
        else {
            if (code != 0xff) {
                *dst++ = 0;
            }
            copy = code = *ptr++;
            if (code == 0) {
                break;  // source length too long
            }
        }
    }

    return dst - start;
}

Jetson::Jetson(PinName tx, PinName rx) : jetson(tx, rx, 115200) {
    jetson.set_blocking(false);
    readTimer.start();
    printf("Jetson init\n");

    cvX = 0; cvY = 0; cvZ = 0;
    status = DISCONNECTED;
}

void Jetson::read()
{
    // Read next byte if available and more needed for the current packet
    // Check disconnect timeout

    //printf("Time since last: %d\n", std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count() - lastRead);

    if (std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count() - lastRead > JETSON_READ_TIMEOUT) {
        status = DISCONNECTED;  // Jetson no longer connected
        reset();                // Reset current remote values
    }
    uint8_t data;  // Next byte to be read
    // Read next byte if available and more needed for the current packet

    while (jetson.read(&data, 1) && currentBufferIndex < JETSON_BUF_LEN) {
        //printf("%x \t", data);
        rxBuffer[currentBufferIndex] = data;
        currentBufferIndex++;
        lastRead = std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count();
    }
    //printf("\n");

    // Check read timeout
    if (std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count() - lastRead > JETSON_READ_RATE) {
        clear();
    }
    // Parse buffer if COBS packet delimiter is found
    if (data == 0) {
        status = CONNECTED;
        parse();
        clear();
    }

}

void Jetson::parse(){
    cobs_unstuff_data(rxBuffer, currentBufferIndex + 1, cobsBuffer);

    if (cobsBuffer[0] == JETSON_TOPIC_ID) {
        cvX = find<float>(cobsBuffer, JETSON_BUF_LEN, 0);
        cvY = find<float>(cobsBuffer, JETSON_BUF_LEN, 1);
        cvZ = find<float>(cobsBuffer, JETSON_BUF_LEN, 2);
    }
}

void Jetson::clear()
{
    // Reset bytes read counter
    currentBufferIndex = 0;
    // Clear remote rxBuffer
    for (int i = 0; i < JETSON_BUF_LEN; i++) {
        rxBuffer[i] = 0;
        cobsBuffer[i] = 0;
    }
    // Clear rxBuffer
    jetson.sync();
}

void Jetson::reset() {
    cvX = 0; cvY = 0; cvZ = 0;
    clear();
}

float Jetson::get(Jetson::cv axis) {
    switch(axis){
        case Jetson::cv::X:
            return cvX;
        case Jetson::cv::Y:
            return cvY;
        case Jetson::cv::Z:
            return cvZ;
        default:
            return 0;
    }
}

template<typename T>
T Jetson::find(uint8_t *buf, size_t len, int index) {
    //printf("finding index %d\n", index);
    if (index < 0 || JETSON_COBS_HEADER_LEN + (index * sizeof(T)) > len) {
        return 0;
    }
    T val;
    memcpy(&val, &cobsBuffer[JETSON_COBS_HEADER_LEN + (index * sizeof(T))], sizeof(T));
    return val;
}