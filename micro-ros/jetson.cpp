//
// Created by ankit on 2/2/23.
//

#include "jetson.h"

// --- micro-ROS Timing ---
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp)
{
    (void)unused;
    struct timeval tv;
    gettimeofday(&tv, 0);
    tp->tv_sec = tv.tv_sec;
    tp->tv_nsec = tv.tv_usec * 1000;
    return 0;
}

// --- micro-ROS Transports ---
static UnbufferedSerial serial_port(USBTX, USBRX);
Timer t;

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

// --- micro-ROS App ---
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
std_msgs__msg__String str;

rclc_executor_t executor;
rcl_node_t node;

bool init = false;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != nullptr) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, nullptr));
    }
}


void Jetson::on_rx_interrupt()
{
    if (serial_port.read(&dma_buffer[dma_tail], 1)) {
        dma_tail = (dma_tail + 1) % UART_DMA_BUFFER_SIZE;
        if (dma_tail == dma_head) {
            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        }
    }
}

bool Jetson::mbed_serial_open(struct uxrCustomTransport * transport){
    serial_port.baud(115200);
    serial_port.format(
            /* bits */ 8,
            /* parity */ BufferedSerial::None,
            /* stop bit */ 1
    );
    serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);

    return true;
}

bool Jetson::mbed_serial_close(struct uxrCustomTransport * transport){
    return serial_port.close() == 0;
}

size_t Jetson::mbed_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    ssize_t wrote = serial_port.write(buf, len);
    serial_port.sync();
    return (wrote >= 0) ? wrote : 0;
}

size_t Jetson::mbed_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    t.start();
    std::chrono::microseconds elapsed = std::chrono::microseconds(0);
    std::chrono::microseconds timeout_us = std::chrono::microseconds(timeout*1000);

    size_t wrote = 0;
    while(elapsed < timeout_us && wrote < len){
        if (dma_head != dma_tail){
            buf[wrote] = dma_buffer[dma_head];
            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
            wrote++;
        }
        elapsed = t.elapsed_time();
    }

    return wrote;
}

double Jetson::get(Jetson::CVDatatype type){
    return msg.data;
}

void Jetson::set(CVDatatype type, double val) {
    msg.data = val;
}

DigitalOut led(LED2);
void Jetson::update() {

    if (!init){
        printf("init!\n");

        rmw_uros_set_custom_transport(
                true,
                nullptr,
                mbed_serial_open,
                mbed_serial_close,
                mbed_serial_write,
                mbed_serial_read
        );

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        // create init_options
        RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

        // create node
        RCCHECK(rclc_node_init_default(&node, "mbed_node", "", &support));

        // create publisher
        RCCHECK(rclc_publisher_init_default(
                &publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "mbed_int32_publisher"));

        // create timer,
        rcl_timer_t timer;
        const unsigned int timer_timeout = 1000;
        RCCHECK(rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(timer_timeout),
                timer_callback));

        // create executor
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        msg.data = 0;

        init = true;
    }
    led = !led;
    printf("update!\n");

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
