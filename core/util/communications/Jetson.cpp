#include "Jetson.h"

/**
 * A helper function to find the values in a buffer of bytes. The
 * buffer must be in the form of a COBS packet.
 *
 * @see https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 *
 * @param ptr A pointer to the buffer to search
 * @param length The length of the buffer
 * @param dst A pointer to the destination, in which the unpacked data will be stored
 * @return The number of bytes unpacked
 */
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

size_t cobs_stuff_data(const uint8_t *input, size_t length, uint8_t *output)
{
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length)
    {
        if (input[read_index] == 0)
        {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        }
        else
        {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFF)
            {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }

    output[code_index] = code;

    return write_index;
}

/**
 * Constructor for the Jetson class. The current implementation of the Jetson
 * class uses UART serial communication.
 *
 * ```
 * // Example usage
 * Jetson jetson(PA_9, PA_10);  // Baud rate is automatically initialized to 115200
 * ```
 *
 * @param tx The transmit pin
 * @param rx The receive pin
 */
Jetson::Jetson(PinName tx, PinName rx) : jetson(tx, rx, 115200) {
    jetson.set_blocking(false);
    readTimer.start();

    cvX = 0; cvY = 0; cvZ = 0;
    status = DISCONNECTED;
}

/**
 * Should be called once per loop. Reads the next set of data from the serial port.
 *
 * ```
 * // Example usage
 * Jetson jetson(PA_9, PA_10);
 *
 * while (true) {
 *   jetson.read();
 *   printf("X: %f, Y: %f, Z: %f\n", jetson.get(Jetson::X), jetson.get(Jetson::Y), jetson.get(Jetson::Z));
 * }
 * ```
 *
 * Note that it is often preferred to do this in a separate thread. See the class header for
 * a full example.
 */
void Jetson::read()
{
    // Read next byte if available and more needed for the current packet
    // Check disconnect timeout

    if (std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count() - lastRead > JETSON_READ_TIMEOUT) {
        status = DISCONNECTED;  // Jetson no longer connected
        reset();                // Reset current remote values
    }
    uint8_t data;  // Next byte to be read
    // Read next byte if available and more needed for the current packet

    while (jetson.read(&data, 1) && currentBufferIndex < JETSON_BUF_LEN) {
        printf("%x \t", data);
        rxBuffer[currentBufferIndex] = data;
        currentBufferIndex++;
        lastRead = std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count();
    }
    printf("\n");

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

uint16_t Jetson::crc16_byte(uint16_t crc, uint8_t data)
{
    return static_cast<uint8_t>(crc >> 8U) ^ crc16_table[static_cast<uint8_t>(crc ^ data)];
}

uint16_t Jetson::crc16(uint8_t const *buffer, size_t len)
{
    uint16_t crc = 0;

    while ((len--) != 0)
    {
        crc = crc16_byte(crc, *buffer++);
    }

    return crc;
}


void Jetson::send(float x, float y, float z) {
    // Make a buffer to store the unstuffed data
    float float_array[3] = {x, y, z};

    uint8_t* unstuffed_data;
    unstuffed_data = reinterpret_cast<uint8_t*>(&float_array);

    // Find the CRC of the data
    uint16_t crc = crc16(unstuffed_data, 3 * sizeof(float));

    uint16_t length = 3 * sizeof(float);
    uint8_t buffer[JETSON_BUF_LEN - 1];
    buffer[0] = JETSON_TOPIC_ID_SEND;
    // buffer[1] should be the high byte of the length
    buffer[1] = (length >> 8);
    // buffer[2] should be the low byte of the lengthlength & 0xFF;
    buffer[2] = length & 0x00FF;

    buffer[3] = (crc >> 8);
    buffer[4] = crc & 0x00FF;

    pack<float>(buffer, x, 0);
    pack<float>(buffer, y, 1);
    pack<float>(buffer, z, 2);

    uint8_t txBuffer[19];
    cobs_stuff_data(buffer, 17, txBuffer);
    txBuffer[18] = 0;


    // unstuff the data as a test
    uint8_t test_unstuffed[18];
    cobs_unstuff_data(txBuffer, 18, test_unstuffed);


//    for (unsigned char i : txBuffer) {
//        printf("%x \t", i);
//    }
//    printf("\n");
    jetson.write(txBuffer, 19);
}

/**
 * Parses the current buffer and updates the current values of the remote.
 * This function should not be called directly. It is called via the read() function.
 */
void Jetson::parse(){
    cobs_unstuff_data(rxBuffer, currentBufferIndex + 1, cobsBuffer);

    if (cobsBuffer[0] == JETSON_TOPIC_ID) {
        cvX = find<float>(cobsBuffer, JETSON_BUF_LEN, 0);
        cvY = find<float>(cobsBuffer, JETSON_BUF_LEN, 1);
        cvZ = find<float>(cobsBuffer, JETSON_BUF_LEN, 2);
    }
}

/**
 * Clears the current buffer and resets the bytes read counter.
 * This function should rarely be called directly. It is called via the read() function.
 */
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

/**
 * Resets the current values of the CV data to 0.
 */
void Jetson::reset() {
    cvX = 0; cvY = 0; cvZ = 0;
    clear();
}

/**
 * Returns the current value of the specified axis.
 *
 * @param axis The axis to get the value of
 * @return The current value of the specified axis
 *
 * For example usage, see the example for the read() function.
 */
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

/**
 * Returns a value from an unpacked COBS buffer. This function should not be called directly.
 *
 * @tparam T The type of the value to return
 * @param buf Unpacked COBS buffer
 * @param len Length of the buffer
 * @param index Which value to return
 * @return The desired value
 *
 * ```
 * // Example usage (from the parse() function)
 * cvX = find<float>(cobsBuffer, JETSON_BUF_LEN, 0);
 * cvY = find<float>(cobsBuffer, JETSON_BUF_LEN, 1);
 * cvZ = find<float>(cobsBuffer, JETSON_BUF_LEN, 2);
 * ```
 *
 */
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

template<typename T>
T Jetson::pack(uint8_t *buf, T value, int index) {
    memcpy(&buf[JETSON_COBS_HEADER_LEN + (index * sizeof(T))], &value, sizeof(T));
    return value;
}