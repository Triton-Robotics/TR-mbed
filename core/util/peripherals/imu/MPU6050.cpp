#include "MPU6050.h"

/**
 * @brief   Creates an MPU6050 object
 * @note    Default address is 0x68
 *          Default acceleration range 2G
 *          Default gyroscope range is 250 dps (degrees per second)
 *          I2C_SDA and I2C_SCL pins are used for I2C connection by default
 *          Interrupt pin is not connected (NC) by default
 * @param
 * @retval
 */
MPU6050::MPU6050
(
    uint8_t     addr /*= 0x68*/,
    AccelScale  accelScale /*= AFS_2G*/,
    GyroScale   gyroScale /*= GFS_250DPS*/,
    PinName     sdaPin /*= I2C_SDA*/,
    PinName     sclPin /*= I2C_SCL*/,
    PinName     interruptInPin  /*= NC*/
) :
    _addr(addr << 1),
    _accelScale(accelScale),
    _gyroScale(gyroScale),
    _i2c(sdaPin, sclPin),
    _interruptIn(interruptInPin),
    _accelRes(0),
    _gyroRes(0),
    _beta(sqrt(3.0f / 4.0f) * _gyroError),
    _zeta(sqrt(3.0f / 4.0f) * _gyroDrift)
{
    _i2c.frequency(400000); // 400 kHz
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void MPU6050::rise(Callback<void (void)> func)
{
    _interruptIn.rise(func);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
template<typename T>
void MPU6050::rise(T* tptr, void (T:: *mptr) (void))
{
    if ((mptr != NULL) && (tptr != NULL))
        _interruptIn.rise(tptr, &T::mptr);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void MPU6050::fall(Callback<void (void)> func)
{
    _interruptIn.fall(func);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
template<typename T>
void MPU6050::fall(T* tptr, void (T:: *mptr) (void))
{
    if ((mptr != NULL) && (tptr != NULL))
        _interruptIn.fall(tptr, &T::mptr);
}

/**
 * @brief   Writes a byte to register
 * @note
 * @param
 * @retval
 */
void MPU6050::writeReg(uint8_t reg, uint8_t byte)
{
    char    data[2];

    data[0] = reg;
    data[1] = byte;
    _i2c.write(_addr, data, sizeof(data), false);
}

/**
 * @brief   Reads byte from a register
 * @note
 * @param
 * @retval
 */
uint8_t MPU6050::readReg(uint8_t reg)
{
    char    ret;                                    // `ret` will store the register data

    _i2c.write(_addr, (const char*) &reg, 1, 1);    // no stop
    _i2c.read(_addr, &ret, 1, 0);
    return ret;
}

/**
 * @brief   Read multiple bytes from a register
 * @note
 * @param
 * @retval
 */
void MPU6050::readRegBytes(uint8_t reg, uint8_t len, uint8_t* dest)
{
    _i2c.write(_addr, (const char*) &reg, 1, 1);    // no stop
    _i2c.read(_addr, (char*)dest, len, 0);
}

/**
 * @brief   Initializes the MPU6050
 * @note
 * @param
 * @retval
 */
bool MPU6050::init()
{
    uint8_t c;

    writeReg(PWR_MGMT_1, 0x00); // Select internal OSC as clock source, clear sleep mode bit (6), enable all sensors

    // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(100ms);
#else
    wait_ms(100);
#endif
    //

    // get stable time source
    writeReg(PWR_MGMT_1, 0x01); // Select PLL with x-axis gyroscope reference as clock source, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // DLPF_CFG = bits 2:0 = 0x03 (0b010)
    // Sets the sample rate to 1 kHz for Low Pass Filter (DLPF) for both the accelerometers and gyros
    // disables FSYNC and sets accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // Maximum delay is 4.9 ms (= 204.08 Hz) which is just over a 200 Hz maximum rate
    writeReg(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Where gyroscope output rate = 1kHz when the DLPF is disabled (DLPF_CFG = 0 or 7)
    // Use a 200 Hz rate; the same rate set in CONFIG above: 200 = 1000/(1 + 4)
    writeReg(SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    c = readReg(GYRO_CONFIG);   // Read the GYRO_CONFIG register
    c &= 0b11100000;            // Clear self-test bits [7:5]

    // Range selectors FS_SEL is 0 - 3, so left-shift 2-bit value into positions 4:3
    c |= (_gyroScale << 3);     // Set FS_SEL bits [4:3]
    writeReg(GYRO_CONFIG, c);   // Set the full scale range for the gyro

    // Update gyro scale according to the selected gyro full scale range.
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Set DPS/(ADC tick) based on that 2-bit value:
    switch (_gyroScale) {
        case GFS_250DPS:
            _gyroRes = 250.0 / 32768.0;
            break;

        case GFS_500DPS:
            _gyroRes = 500.0 / 32768.0;
            break;

        case GFS_1000DPS:
            _gyroRes = 1000.0 / 32768.0;
            break;

        case GFS_2000DPS:
            _gyroRes = 2000.0 / 32768.0;
            break;
    }

    // Set accelerometer configuration
    c = readReg(ACCEL_CONFIG);  // Read the ACCEL_CONFIG register
    c &= 0b11100000;            // Clear self-test bits [7:5]

    // Range selectors AFS_SEL is 0 - 3, so left-shift 2-bit value into positions 4:3
    c |= (_accelScale << 3);
    writeReg(ACCEL_CONFIG, c);  // Set full scale range for the accelerometer

    // Update acceleration scle to the selected acceleration scale range.
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    switch (_accelScale) {
        case AFS_2G:
            _accelRes = 2.0 / 32768.0;
            break;

        case AFS_4G:
            _accelRes = 4.0 / 32768.0;
            break;

        case AFS_8G:
            _accelRes = 8.0 / 32768.0;
            break;

        case AFS_16G:
            _accelRes = 16.0 / 32768.0;
            break;
    }

    // Configure Interrupts and Bypass Enable
    // When bit [7] (INT_LEVEL) is equal to 0, the logic level for the INT pin is active high.
    // When bit [6] (INT_OPEN) is equal to 0, the INT pin is configured as push-pull.
    // When bit [5] (LATCH_INT_EN) is equal to 1, the INT pin is held high until the interrupt is cleared.
    // When bit [4] (INT_RD_CLEAR) is equal to 0, interrupt status bits are cleared only by reading INT_STATUS
    // When bit [3] (FSYNC_INT_LEVEL) is equal to 0, the logic level for the FSYNC pin
    // (when used as an interrupt to the host processor) is active high.
    // When bit [2] (FSYNC_INT_EN) is equal to 0, this bit disables the FSYNC pin from causing
    // an interrupt to the host processor.
    // When bit [1] (I2C_BYPASS_EN) is equal to 1 and bit [5] (I2C_MST_EN) in the USER_CTRL register) is equal to 0,
    // the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0.
    writeReg(INT_PIN_CFG, 0b00100010);

    // Clear bit [5] in the USER_CTRL register
    c = readReg(USER_CTRL);
    c &= ~(1 << I2C_MST_EN);
    writeReg(USER_CTRL, c);

    // Enable data ready interrupt (bit [0])
    writeReg(INT_ENABLE, 0x01);

    return true;
}

/**
 * @brief   Indicates whether data is available after an interrupt
 * @note    The DATA_RDY_INT bit clears to 0 after the INT_STATUS register has been read.
 * @param
 * @retval
 */
bool MPU6050::dataReady()
{
    return(readReg(INT_STATUS) & (1 << DATA_RDY_INT));
}

/**
 * @brief   Reads acceleration ADC (analog to digital converter)
 * @note
 * @param
 * @retval  Acceleration ADC array of three elements
 */
int16_t* MPU6050::accelADC()
{
    uint8_t rawData[6];                                     // x/y/z accel register data stored here

    readRegBytes(ACCEL_XOUT_H, 6, rawData);                 // Read the six raw data registers into data array
    _accelAdc[0] = int16_t(rawData[0]) << 8 | rawData[1];   // Turn the MSB and LSB into a signed 16-bit value
    _accelAdc[1] = int16_t(rawData[2]) << 8 | rawData[3];
    _accelAdc[2] = int16_t(rawData[4]) << 8 | rawData[5];

    return _accelAdc;
}

/**
 * @brief   Reads gyro ADC (analog to digital converter)
 * @note
 * @param
 * @retval  Gyro ADC array of three elements
 */
int16_t* MPU6050::gyroADC()
{
    uint8_t rawData[6];                                     // x/y/z gyro register data stored here

    readRegBytes(GYRO_XOUT_H, 6, rawData);                  // Read the six raw data registers sequentially into data array
    _gyroAdc[0] = int16_t(rawData[0]) << 8 | rawData[1];    // Turn the MSB and LSB into a signed 16-bit value
    _gyroAdc[1] = int16_t(rawData[2]) << 8 | rawData[3];
    _gyroAdc[2] = int16_t(rawData[4]) << 8 | rawData[5];

    return _gyroAdc;
}

/**
 * @brief   Reads temperature ADC (analog to digital converter)
 * @note
 * @param
 * @retval  Temperature ADC value
 */
int16_t MPU6050::tempADC()
{
    uint8_t rawData[2];                             // x/y/z gyro register data stored here

    readRegBytes(TEMP_OUT_H, 2, &rawData[0]);       // Read the two raw data registers sequentially into data array
    return int16_t(rawData[0]) << 8 | rawData[1];   // Turn the MSB and LSB into a 16-bit value
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
float MPU6050::temp()
{
    return(tempADC() / 340. + 36.53);   // Temperature in degrees Celsius
}

/**
 * @brief   Configures the motion detection control for low power accelerometer mode
 * @note    The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
 *          Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
 *          above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
 *          threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
 *          consideration for these threshold evaluations; otherwise, the flags would be set all the time!
 * @param
 * @retval
 */
void MPU6050::lowPowerAccelOnly()
{
    uint8_t c = readReg(PWR_MGMT_1);
    writeReg(PWR_MGMT_1, c &~0x30);     // Clear sleep and cycle bits [5:6]
    writeReg(PWR_MGMT_1, c | 0x30);     // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running
    c = readReg(PWR_MGMT_2);
    writeReg(PWR_MGMT_2, c &~0x38);     // Clear standby XA, YA, and ZA bits [3:5]
    writeReg(PWR_MGMT_2, c | 0x00);     // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running
    c = readReg(ACCEL_CONFIG);
    writeReg(ACCEL_CONFIG, c &~0x07);   // Clear high-pass filter bits [2:0]

    // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    writeReg(ACCEL_CONFIG, c | 0x00);   // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter
    c = readReg(CONFIG);
    writeReg(CONFIG, c &~0x07);         // Clear low-pass filter bits [2:0]
    writeReg(CONFIG, c | 0x00);         // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate
    c = readReg(INT_ENABLE);
    writeReg(INT_ENABLE, c &~0xFF);     // Clear all interrupts
    writeReg(INT_ENABLE, 0x40);         // Enable motion threshold (bits 5) interrupt only

    // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
    // for at least the counter duration
    writeReg(MOT_THR, 0x80);            // Set motion detection to 0.256 g; LSB = 2 mg
    writeReg(MOT_DUR, 0x01);            // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

    // Add delay for accumulation of samples
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(100ms);
#else
    wait_ms(100);
#endif
    c = readReg(ACCEL_CONFIG);
    writeReg(ACCEL_CONFIG, c &~0x07);   // Clear high-pass filter bits [2:0]
    writeReg(ACCEL_CONFIG, c | 0x07);   // Set ACCEL_HPF to 7; hold the initial accleration value as a referance
    c = readReg(PWR_MGMT_2);
    writeReg(PWR_MGMT_2, c &~0xC7);     // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    writeReg(PWR_MGMT_2, c | 0x47);     // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])
    c = readReg(PWR_MGMT_1);
    writeReg(PWR_MGMT_1, c &~0x20);     // Clear sleep and cycle bit 5
    writeReg(PWR_MGMT_1, c | 0x20);     // Set cycle bit 5 to begin low power accelerometer motion interrupts
}

/**
 * @brief   Resets the MPU6050
 * @note
 * @param
 * @retval
 */
void MPU6050::reset()
{
    // Reset the device

    writeReg(PWR_MGMT_1, 0b10000000);   // Write a one to bit 7 (reset bit);
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(100ms);
#else
    wait_ms(100);
#endif
}

/**
 * @brief   Calibrates the MPU6050
 * @note    Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
 *          of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
 * @param
 * @retval
 */
void MPU6050::calibrate()
{
    uint8_t     data[12];                       // data array to hold accelerometer and gyro x, y, z, data
    uint16_t    packetCount, fifoCount;
    int32_t     gyroBias[3] = { 0, 0, 0 };
    int32_t     accelBias[3] = { 0, 0, 0 };

    // Reset the device, reset all registers, clear gyro and accelerometer bias registers

    reset();

    // Get stable time source: set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeReg(PWR_MGMT_1, 0x01);
    writeReg(PWR_MGMT_2, 0x00);
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(200ms);
#else
    wait_ms(200);
#endif
    //

    // Configure the device for bias calculation
    writeReg(INT_ENABLE, 0x00);                 // Disable all interrupts
    writeReg(FIFO_EN, 0x00);                    // Disable FIFO
    writeReg(PWR_MGMT_1, 0x00);                 // Turn on internal clock source
    writeReg(I2C_MST_CTRL, 0x00);               // Disable I2C master
    writeReg(USER_CTRL, 0x00);                  // Disable FIFO and I2C master modes
    writeReg(USER_CTRL, 0x0C);                  // Reset FIFO and DMP
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(200ms);
#else
    wait_ms(200);
#endif
    // Configure MPU6050 gyro and accelerometer for bias calculation

    writeReg(CONFIG, 0x01);                     // Set low-pass filter to 188 Hz
    writeReg(SMPLRT_DIV, 0x00);                 // Set sample rate to 1 kHz
    writeReg(GYRO_CONFIG, 0x00);                // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeReg(ACCEL_CONFIG, 0x00);               // Set accelerometer full-scale to 2 g, maximum sensitivity
    uint16_t    gyroSensitivity = 131;          // = 131 LSB/degrees/sec
    uint16_t    accelSensitivity = 16384;       // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeReg(USER_CTRL, 0x40);                  // Enable FIFO
    writeReg(FIFO_EN, 0x78);                    // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)

    // accumulate 80 samples in 80 milliseconds = 960 bytes
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(80ms);
#else
    wait_ms(80);
#endif
    //

    // At end of sample accumulation, turn off FIFO sensor read
    writeReg(FIFO_EN, 0x00);                    // Disable gyro and accelerometer sensors for FIFO
    readRegBytes(FIFO_COUNTH, 2, &data[0]);     // read FIFO sample count
    fifoCount = ((uint16_t) data[0] << 8) | data[1];
    packetCount = fifoCount / 12;               // How many sets of full gyro and accelerometer data for averaging
    for (int i = 0; i < packetCount; i++) {
        int16_t accel_temp[3] = { 0, 0, 0 },
        gyro_temp[3] = { 0, 0, 0 };
        readRegBytes(FIFO_R_W, 12, &data[0]);   // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

        accelBias[0] += (int32_t) accel_temp[0];                        // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accelBias[1] += (int32_t) accel_temp[1];
        accelBias[2] += (int32_t) accel_temp[2];
        gyroBias[0] += (int32_t) gyro_temp[0];
        gyroBias[1] += (int32_t) gyro_temp[1];
        gyroBias[2] += (int32_t) gyro_temp[2];
    }

    accelBias[0] /= (int32_t) packetCount;                              // Normalize sums to get average count biases
    accelBias[1] /= (int32_t) packetCount;
    accelBias[2] /= (int32_t) packetCount;
    gyroBias[0] /= (int32_t) packetCount;
    gyroBias[1] /= (int32_t) packetCount;
    gyroBias[2] /= (int32_t) packetCount;

    if (accelBias[2] > 0L) {
        accelBias[2] -= (int32_t) accelSensitivity;
    }                                           // Remove gravity from the z-axis accelerometer bias calculation
    else {
        accelBias[2] += (int32_t) accelSensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyroBias[0] / 4 >> 8) & 0xFF;   // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyroBias[0] / 4) & 0xFF;        // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyroBias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyroBias[1] / 4) & 0xFF;
    data[4] = (-gyroBias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyroBias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    writeReg(XG_OFFS_USRH, data[0]);
    writeReg(XG_OFFS_USRL, data[1]);
    writeReg(YG_OFFS_USRH, data[2]);
    writeReg(YG_OFFS_USRL, data[3]);
    writeReg(ZG_OFFS_USRH, data[4]);
    writeReg(ZG_OFFS_USRL, data[5]);

    _gyroBias[0] = (float)gyroBias[0] / (float)gyroSensitivity; // construct gyro bias in deg/s for later manual subtraction
    _gyroBias[1] = (float)gyroBias[1] / (float)gyroSensitivity;
    _gyroBias[2] = (float)gyroBias[2] / (float)gyroSensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    int32_t accel_bias_reg[3] = { 0, 0, 0 };                    // A place to hold the factory accelerometer trim biases

    readRegBytes(XA_OFFSET_H, 2, &data[0]);                     // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];
    readRegBytes(YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];
    readRegBytes(ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

    uint32_t    mask = 1uL;                                     // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t     mask_bit[3] = { 0, 0, 0 };                      // Define array to hold mask bit for each accelerometer bias axis

    for (int i = 0; i < 3; i++) {
        if (accel_bias_reg[i] & mask)
            mask_bit[i] = 0x01;                                 // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accelBias[0] / 8);                    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accelBias[1] / 8);
    accel_bias_reg[2] -= (accelBias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0];                            // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1];                            // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2];                            // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    //  writeByte(_address, XA_OFFSET_H, data[0]);
    //  writeByte(_address, XA_OFFSET_L_TC, data[1]);
    //  writeByte(_address, YA_OFFSET_H, data[2]);
    //  writeByte(_address, YA_OFFSET_L_TC, data[3]);
    //  writeByte(_address, ZA_OFFSET_H, data[4]);
    //  writeByte(_address, ZA_OFFSET_L_TC, data[5]);
    // Output scaled accelerometer biases for manual subtraction in the main program
    _accelBias[0] = (float)accelBias[0] / (float)accelSensitivity;
    _accelBias[1] = (float)accelBias[1] / (float)accelSensitivity;
    _accelBias[2] = (float)accelBias[2] / (float)accelSensitivity;
}

/**
 * @brief   Performs accelerometer and gyroscope self test
 * @note    Checks calibration wrt factory settings.
 *          Should return percent deviation from factory trim values
 *          +/- 14 or less deviation is a pass
 * @param
 * @retval  true    if passed
 *          false   otherwise
 */
bool MPU6050::selfTestOK()
{
    uint8_t rawData[4] = { 0, 0, 0, 0 };
    uint8_t selfTest[6];
    float   factoryTrim[6];
    //
    // Configure the accelerometer for self-test
    writeReg(ACCEL_CONFIG, 0xF0);                               // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeReg(GYRO_CONFIG, 0xE0);                                // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
#if MBED_MAJOR_VERSION > 5
    ThisThread::sleep_for(250ms);
#else
    wait_ms(250);
#endif
    rawData[0] = readReg(SELF_TEST_X);                          // X-axis self-test results
    rawData[1] = readReg(SELF_TEST_Y);                          // Y-axis self-test results
    rawData[2] = readReg(SELF_TEST_Z);                          // Z-axis self-test results
    rawData[3] = readReg(SELF_TEST_A);                          // Mixed-axis self-test results

    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4; // ZA_TEST result is a five-bit unsigned integer

    // Extract the gyration test results first
    selfTest[3] = rawData[0] & 0x1F;                            // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1] & 0x1F;                            // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2] & 0x1F;                            // ZG_TEST result is a five-bit unsigned integer

    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((selfTest[0] - 1.0f) / 30.0f)));    // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((selfTest[1] - 1.0f) / 30.0f)));    // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f * 0.34f) * (pow((0.92f / 0.34f), ((selfTest[2] - 1.0f) / 30.0f)));    // FT[Za] factory trim calculation
    factoryTrim[3] = (25.0f * 131.0f) * (pow(1.046f, (selfTest[3] - 1.0f)));                        // FT[Xg] factory trim calculation
    factoryTrim[4] = (-25.0f * 131.0f) * (pow(1.046f, (selfTest[4] - 1.0f)));                       // FT[Yg] factory trim calculation
    factoryTrim[5] = (25.0f * 131.0f) * (pow(1.046f, (selfTest[5] - 1.0f)));                        // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);
    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        _selfTest[i] = 100.0f + 100.0f * (selfTest[i] - factoryTrim[i]) / factoryTrim[i];           // Report percent differences
    }

    return
        (
            _selfTest[0] < 1.0f &&
            _selfTest[1] < 1.0f &&
            _selfTest[2] < 1.0f &&
            _selfTest[3] < 1.0f &&
            _selfTest[4] < 1.0f &&
            _selfTest[5] < 1.0f
        );
}

/**
 * @brief   Computes acceleration
 * @note
 * @param
 * @retval
 */
float* MPU6050::accel()
{
    accelADC();

    // Compute actual acceleration values, this depends on used scale
    accelX = (float)_accelAdc[0] * _accelRes - _accelBias[0];
    accelY = (float)_accelAdc[1] * _accelRes - _accelBias[1];
    accelZ = (float)_accelAdc[2] * _accelRes - _accelBias[2];

    return accelData;
}

/**
 * @brief   Computes gyro
 * @note
 * @param
 * @retval
 */
float* MPU6050::gyro()
{
    gyroADC();

    // Compute actual gyro values, this depends on used scale
    gyroX = (float)_gyroAdc[0] * _gyroRes - _gyroBias[0];
    gyroY = (float)_gyroAdc[1] * _gyroRes - _gyroBias[1];
    gyroZ = (float)_gyroAdc[2] * _gyroRes - _gyroBias[2];
    return gyroData;
}

/**
 * @brief   Adjusts filter gain after device is stabilized
 * @note
 * @param
 * @retval
 */
void MPU6050::setGain(float beta, float zeta)
{
    _beta = beta;  // set filter gain
    _zeta = zeta;  // set bias drift gain
}

/**
 * @brief   Madgwick filter
 * @note    Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
 *          (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
 *          which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
 *          device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
 *          The performance of the orientation filter is almost as good as conventional Kalman-based filtering algorithms
 *          but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
 * @param   deltaT  Integration time in seconds.
 * @retval
 */
void MPU6050::madgwickFilter(float deltaT)
{
    // aliases for better readability
    float&  q1 = _q[0];
    float&  q2 = _q[1];
    float&  q3 = _q[2];
    float&  q4 = _q[3];
    float   norm;       // vector norm
    float   f1, f2, f3; // objective funcion elements
    float   j_11or24, j_12or23, j_13or22, j_14or21, j_32, j_33; // objective function Jacobian elements
    float   qDot1, qDot2, qDot3, qDot4;
    float   hatDot1, hatDot2, hatDot3, hatDot4;
    float   ax = accelX;
    float   ay = accelY;
    float   az = accelZ;
    float   gx = gyroX * M_PI / 180.0f;                         // convert to rad/s
    float   gy = gyroY * M_PI / 180.0f;                         // convert to rad/s
    float   gz = gyroZ * M_PI / 180.0f;                         // convert to rad/s
    float   gErrX, gErrY, gErrZ;        // gyro bias errors
    float gBiasX{0}; 
    float gBiasY{0}; 
    float gBiasZ{0};

    // Auxiliary variables to avoid repeated arithmetic
    float   halfq1 = 0.5f * q1;
    float   halfq2 = 0.5f * q2;
    float   halfq3 = 0.5f * q3;
    float   halfq4 = 0.5f * q4;
    float   _2q1 = 2.0f * q1;
    float   _2q2 = 2.0f * q2;
    float   _2q3 = 2.0f * q3;
    float   _2q4 = 2.0f * q4;
    //float _2q1q3 = 2.0f * q1 * q3;
    //float _2q3q4 = 2.0f * q3 * q4;
    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return;                                         // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    j_11or24 = _2q3;
    j_12or23 = _2q4;
    j_13or22 = _2q1;
    j_14or21 = _2q2;
    j_32 = 2.0f * j_14or21;
    j_33 = 2.0f * j_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = j_14or21 * f2 - j_11or24 * f1;
    hatDot2 = j_12or23 * f1 + j_13or22 * f2 - j_32 * f3;
    hatDot3 = j_12or23 * f2 - j_33 * f3 - j_13or22 * f1;
    hatDot4 = j_14or21 * f1 + j_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope errors
    gErrX = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gErrY = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gErrZ = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute gyroscope biases
    gBiasX += gErrX * deltaT * _zeta;
    gBiasY += gErrY * deltaT * _zeta;
    gBiasZ += gErrZ * deltaT * _zeta;

    // Remove gyroscope biases
    gx -= gBiasX;
    gy -= gBiasY;
    gz -= gBiasZ;

    // Compute the quaternion derivative
    qDot1 = -halfq2 * gx - halfq3 * gy - halfq4 * gz;
    qDot2 = halfq1 * gx + halfq3 * gz - halfq4 * gy;
    qDot3 = halfq1 * gy - halfq2 * gz + halfq4 * gx;
    qDot4 = halfq1 * gz + halfq2 * gy - halfq3 * gx;

    // Compute the integrated estimated quaternion derivative
    q1 += (qDot1 - (_beta * hatDot1)) * deltaT;
    q2 += (qDot2 - (_beta * hatDot2)) * deltaT;
    q3 += (qDot3 - (_beta * hatDot3)) * deltaT;
    q4 += (qDot4 - (_beta * hatDot4)) * deltaT;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;
    _q[0] = q1 * norm;
    _q[1] = q2 * norm;
    _q[2] = q3 * norm;
    _q[3] = q4 * norm;
}

/**
 * @brief   Calculates yaw in degree
 * @note
 * @param
 * @retval
 */
float MPU6050::yaw()
{
    float   yawRad = atan2
        (
            2.0f * (_q[1] * _q[2] + _q[0] * _q[3]),
            _q[0] * _q[0] + _q[1] * _q[1] - _q[2] * _q[2] - _q[3] * _q[3]
        );

    return yawRad * 180.f / M_PI;
}

/**
 * @brief   Calculates pitch in degree
 * @note
 * @param
 * @retval
 */
float MPU6050::pitch()
{
    float   pitchRad = -asin(2.0f * (_q[1] * _q[3] - _q[0] * _q[2]));

    return pitchRad * 180.f / M_PI;
}

/**
 * @brief   Claculates roll in degree
 * @note
 * @param
 * @retval
 */
float MPU6050::roll()
{
    float   rollRad = atan2
        (
            2.0f * (_q[0] * _q[1] + _q[2] * _q[3]),
            _q[0] * _q[0] - _q[1] * _q[1] - _q[2] * _q[2] + _q[3] * _q[3]
        );

    return rollRad * 180.0f / M_PI;
}
