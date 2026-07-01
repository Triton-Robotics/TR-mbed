#include "mbed.h"
#include <cstdio>

// ================================
// CHANGE THESE PINS IF NEEDED
// For many STM32 Nucleo boards:
// CAN RD = PA_11
// CAN TD = PA_12
// ================================
CAN can1(PA_11, PA_12);

// Change this if your C620 ID is not 1
const int MOTOR_ID = 1;

// Feedback ID for motor ID 1 is usually 0x201
const int MOTOR_FEEDBACK_ID = 0x200 + MOTOR_ID;

// Current command ID:
// Motors 1 to 4 use 0x200
// Motors 5 to 8 use 0x1FF
const int MOTOR_COMMAND_ID = 0x200;

// Small safe test current
// Do NOT start with 5000 or 10000
const int16_t TEST_CURRENT = 300;

// ================================
// Send current command to motors 1-4
// id1 controls motor ID 1
// id2 controls motor ID 2
// id3 controls motor ID 3
// id4 controls motor ID 4
// ================================
void send_m3508_current(int16_t id1, int16_t id2, int16_t id3, int16_t id4) {
    char data[8];

    data[0] = (id1 >> 8) & 0xFF;
    data[1] = id1 & 0xFF;

    data[2] = (id2 >> 8) & 0xFF;
    data[3] = id2 & 0xFF;

    data[4] = (id3 >> 8) & 0xFF;
    data[5] = id3 & 0xFF;

    data[6] = (id4 >> 8) & 0xFF;
    data[7] = id4 & 0xFF;

    CANMessage msg(MOTOR_COMMAND_ID, data, 8);
    can1.write(msg);
}

// ================================
// Choose which motor gets current
// ================================
void send_test_current(int16_t current) {
    if (MOTOR_ID == 1) {
        send_m3508_current(current, 0, 0, 0);
    } else if (MOTOR_ID == 2) {
        send_m3508_current(0, current, 0, 0);
    } else if (MOTOR_ID == 3) {
        send_m3508_current(0, 0, current, 0);
    } else if (MOTOR_ID == 4) {
        send_m3508_current(0, 0, 0, current);
    } else {
        // This simple test code only supports motor IDs 1-4
        send_m3508_current(0, 0, 0, 0);
    }
}

// ================================
// Read C620 feedback
// ================================
void read_can_feedback() {
    CANMessage rx_msg;

    while (can1.read(rx_msg)) {
        if (rx_msg.id == MOTOR_FEEDBACK_ID && rx_msg.len == 8) {
            uint16_t angle = ((uint8_t)rx_msg.data[0] << 8) | (uint8_t)rx_msg.data[1];

            int16_t speed_rpm = ((uint8_t)rx_msg.data[2] << 8) | (uint8_t)rx_msg.data[3];

            int16_t actual_current = ((uint8_t)rx_msg.data[4] << 8) | (uint8_t)rx_msg.data[5];

            uint8_t temperature = (uint8_t)rx_msg.data[6];

            printf("Feedback received | ID: 0x%03X | angle: %u | speed: %d rpm | current: %d | temp: %u C\n",
                   rx_msg.id,
                   angle,
                   speed_rpm,
                   actual_current,
                   temperature);
        } else {
            printf("Other CAN msg | ID: 0x%03X | len: %d\n", rx_msg.id, rx_msg.len);
        }
    }
}

// ================================
// Send one pulse
// ================================
void motor_pulse(int16_t current, int duration_ms) {
    printf("Sending current: %d\n", current);

    int loops = duration_ms / 10;

    for (int i = 0; i < loops; i++) {
        send_test_current(current);
        read_can_feedback();
        ThisThread::sleep_for(10ms);
    }

    printf("Stopping motor\n");

    for (int i = 0; i < 50; i++) {
        send_test_current(0);
        read_can_feedback();
        ThisThread::sleep_for(10ms);
    }
}

int main() {
    printf("\n\n=== M3508 + C620 Motor Test Started ===\n");

    // C620 CAN is usually 1 Mbps
    can1.frequency(1000000);

    printf("CAN frequency set to 1 Mbps\n");
    printf("Testing motor ID: %d\n", MOTOR_ID);
    printf("Expected feedback ID: 0x%03X\n", MOTOR_FEEDBACK_ID);

    // Always start with zero current
    printf("Sending zero current first...\n");

    for (int i = 0; i < 200; i++) {
        send_test_current(0);
        read_can_feedback();
        ThisThread::sleep_for(10ms);
    }

    printf("If feedback appears above, CAN is working.\n");
    printf("Now sending small test pulses.\n");

    while (true) {
        // Forward small pulse
        motor_pulse(TEST_CURRENT, 500);

        ThisThread::sleep_for(1000ms);

        // Reverse small pulse
        motor_pulse(-TEST_CURRENT, 500);

        ThisThread::sleep_for(2000ms);
    }
}