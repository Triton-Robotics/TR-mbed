#include "Jetson.h"

// #define JETSON_DEBUG // Uncomment this line to enable debug print statements

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

/**
 * A helper function to pack a buffer of bytes into a COBS packet. The
 * buffer must be in the form of a COBS packet.
 *
 * @see https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 *
 * @param input     The input buffer, which should be in the form |topic_ID|len_high|len_low|CRC_high|CRC_low|payload_start...payload_end|
 * @param length    The length of the input buffer
 * @param output    The output buffer, which will be a stuffed COBS packet
 * @return          The length of the output buffer
 */
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
 * **Example usage:**
 * ```
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
 * **Example usage:**
 * ```
 * Jetson jetson(PA_9, PA_10);
 *
 * while (true) {
 *   jetson.read();
 *   printf("X: %f, Y: %f, Z: %f\n", jetson.get(Jetson::cv::X), jetson.get(Jetson::cv::Y), jetson.get(Jetson::cv::Z));
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
        #ifdef JETSON_DEBUG
            printf("%x \t", data);
        #endif
        rxBuffer[currentBufferIndex] = data;
        currentBufferIndex++;
        lastRead = std::chrono::duration_cast<std::chrono::milliseconds>(readTimer.elapsed_time()).count();
    }
    #ifdef JETSON_DEBUG
        printf("\n");
    #endif

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

/**
 * Finds the CRC value for a singular byte
 *
 * @param crc The current CRC value
 * @param data Input byte
 * @return The new CRC value
 */
uint16_t Jetson::crc16_byte(uint16_t crc, uint8_t data)
{
    return static_cast<uint8_t>(crc >> 8U) ^ crc16_table[static_cast<uint8_t>(crc ^ data)];
}

/**
 * Finds the CRC value for a buffer of bytes
 *
 * @param buffer The buffer of bytes
 * @param len   The length of the buffer
 * @return    The CRC value
 */
uint16_t Jetson::crc16(uint8_t const *buffer, size_t len)
{
    uint16_t crc = 0;

    while ((len--) != 0)
    {
        crc = crc16_byte(crc, *buffer++);
    }

    return crc;
}

/**
 * Should be called once per loop. Writes the next set of data to the serial port.
 *
 * **Example usage**
 * ```
 * while (true) {
 *  jetson.set(Jetson::cv::X, 0.5);
 *  jetson.set(Jetson::cv::Y, 69);
 *  jetson.set(Jetson::cv::Z, 420);
 *  jetson.write();
 * }
 * ```
 *
 * Note that it is often preferred to do this in a separate thread. See the class header for
 * a full example.
 */
void Jetson::write() {
    // Get the current values
    float x = turretX.load();
    float y = turretY.load();
    float z = turretZ.load();

    // Make a buffer to store the unstuffed data
    float float_array[3] = {x, y, z};

    // Cast the float array to an array of bytes
    uint8_t* unstuffed_data;
    unstuffed_data = reinterpret_cast<uint8_t*>(&float_array);

    // Find the CRC of the data
    uint16_t crc = crc16(unstuffed_data, 3 * sizeof(float));

    // The length of the data is 3 floats
    uint16_t length = 3 * sizeof(float);

    // Create a buffer that we will pack using COBS later
    uint8_t buffer[JETSON_BUF_LEN - 1];

    // First value of the buffer is the send ID
    buffer[0] = JETSON_TOPIC_ID_SEND;
    // buffer[1] should be the high byte of the length
    buffer[1] = (length >> 8);
    // buffer[2] should be the low byte of the length
    buffer[2] = length & 0x00FF;
    // buffer[3] should be the high byte of the CRC
    buffer[3] = (crc >> 8);
    // buffer[4] should be the low byte of the CRC
    buffer[4] = crc & 0x00FF;

    // Populate the rest of the buffer with the payload
    pack<float>(buffer, x, 0);
    pack<float>(buffer, y, 1);
    pack<float>(buffer, z, 2);

    // Stuff the data
    uint8_t txBuffer[19];
    cobs_stuff_data(buffer, 17, txBuffer);

    // COBS packet delimiter is zero
    txBuffer[18] = 0;

    #ifdef JETSON_DEBUG
        for (unsigned char i : txBuffer) {
            printf("%x \t", i);
        }
        printf("\n");
    #endif
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
 * Sets the current value of the specified axis.
 *
 * @param axis The axis to set the value of
 * @param value The value to set the axis to
 *
 * For example usage, see the example for the write() function.
 */
void Jetson::set(Jetson::cv axis, float value) {
    switch(axis){
        case Jetson::cv::X:
            cvX = value;
            break;
        case Jetson::cv::Y:
            cvY = value;
            break;
        case Jetson::cv::Z:
            cvZ = value;
            break;
        default:
            return;
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
 * **Example usage:**
 *
 * ```
 * cvX = find<float>(cobsBuffer, JETSON_BUF_LEN, 0);
 * cvY = find<float>(cobsBuffer, JETSON_BUF_LEN, 1);
 * cvZ = find<float>(cobsBuffer, JETSON_BUF_LEN, 2);
 * ```
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

/**
 * Packs a value into a COBS buffer. This function should not be called directly.
 *
 * @tparam T    The type of the value to pack
 * @param buf   The buffer to pack the value into
 * @param value The value to pack
 * @param index The index of the value to pack
 * @return      The value that was packed
 *
 * **Example usage: (From write function)**
 * ```
 * pack<float>(buffer, x, 0);
 * pack<float>(buffer, y, 1);
 * pack<float>(buffer, z, 2);
 * ```
 */
template<typename T>
T Jetson::pack(uint8_t *buf, T value, int index) {
    memcpy(&buf[JETSON_COBS_HEADER_LEN + (index * sizeof(T))], &value, sizeof(T));
    return value;
}