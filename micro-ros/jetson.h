//
// Created by ankit on 2/2/23.
//

#ifndef TR_EMBEDDED_JETSON_H
#define TR_EMBEDDED_JETSON_H

#define UART_DMA_BUFFER_SIZE 2048

#include "mbed.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);while(1){};}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


class Jetson {

private:

    static void on_rx_interrupt();
    static bool mbed_serial_open(struct uxrCustomTransport * transport);
    static bool mbed_serial_close(struct uxrCustomTransport * transport);
    static size_t mbed_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
    static size_t mbed_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);


public:

    enum TeamColor{
        UNDEFINED,
        RED,
        BLUE
    };

    enum CVDatatype{
        CoordinateX,
        CoordinateY,
        TeamColor
    };

    static void set(CVDatatype type, double val);
    static double get(CVDatatype type);

    static void init();
    static void update();
    static void free();

};


#endif //TR_EMBEDDED_JETSON_H
