#pragma once

#include "mbed.h"
#include "PinNames.h"
#include "util/communications/DJIRemote.h"
#include "util/communications/referee/ref_serial.h"
#include "util/motor/DJIMotor.h"
#include <us_ticker_api.h>

class BaseRobot {
  public:
    struct Config {
        PinName remote_pin = PA_10;
        
        PinName referee_tx_pin = PC_10;
        PinName referee_rx_pin = PC_11;

        PinName can1_rx_pin = PA_11;
        PinName can1_tx_pin = PA_12;

        PinName can2_rx_pin = PB_12;
        PinName can2_tx_pin = PB_13;

        PinName led0_pin = PB_0;
        PinName led1_pin = PC_1;
        PinName led2_pin = PC_0;
    };

    Remote remote_;
    Referee referee;

    CANHandler canHandler1_;
    CANHandler canHandler2_;
    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;

    // clang-format off
    BaseRobot(const Config &config)
        : remote_(config.remote_pin),
          referee(config.referee_tx_pin, config.referee_rx_pin), //TODO make sure this is right
          canHandler1_(config.can1_rx_pin, config.can1_tx_pin),
          canHandler2_(config.can2_rx_pin, config.can2_tx_pin),
          led0_(config.led0_pin),
          led1_(config.led1_pin),
          led2_(config.led2_pin)
    {};
    // clang-format on

    virtual ~BaseRobot() = default;

    virtual void init() = 0;
    virtual void periodic(const unsigned long dt_us) = 0;
    virtual void end_of_loop() {};

    // default 1000hz main loop. Can be overriden
    virtual unsigned int main_loop_dt_ms() { return 1; };

    virtual void main_loop() {
        unsigned long loop_clock_us = us_ticker_read();
        unsigned long prev_loop_time_us = loop_clock_us;
        unsigned long prev_remote_time_us = loop_clock_us;

        unsigned long main_loop_dt_ms = this->main_loop_dt_ms();

        // Init all constants, subsystems, sensors, IO, etc.
        // Each can message has an id and data, and djimotor ids start from 0x201(m3508 id 1) and end at 0x20D (gm6020 id 8), there is an overlap of 4 motors (M3508 id 5-8 and gm6020 id 1-4), so that is why we have 12 values only
        canHandler1_.registerCallback(0x201, 0x20D, DJIMotor::getCan1Feedback);
        canHandler2_.registerCallback(0x201, 0x20D, DJIMotor::getCan2Feedback);
        DJIMotor::setCanHandlers(&canHandler1_, &canHandler2_);
        
        init();
        
        while (true) {
            // StmIO comms (Ref and Jetson)
            // TODO Mutex referee class and make it a good class
            // TODO update stm_state with ref.read?
                        
            loop_clock_us = us_ticker_read();


            // 20 ms remote read
            if ((loop_clock_us - prev_remote_time_us) / 1000 >= 15) {
                remote_.read();
                prev_remote_time_us = us_ticker_read();
            }

            if ((loop_clock_us - prev_loop_time_us) / 1000 >= main_loop_dt_ms) {
                // Add subsystems in periodic
                led0_ = !led0_;

                periodic(loop_clock_us - prev_loop_time_us);
                prev_loop_time_us = us_ticker_read();
                
                // Motor updates
                DJIMotor::sendValues();
            }
            // Add sensors updates in your end of loop
            end_of_loop();

            canHandler1_.readAllCan();
            canHandler2_.readAllCan();
        }
    }

    // TODO: make header file
    // void printff(const char *format, ...)
    // {
    //     char temp[50];
    //     va_list args;
    //     va_start(args, format);
    //     int len = vsnprintf(temp, 50, format, args);
    //     if (len > 0)
    //     // usbSerial.write(temp, len);
    //     va_end(args);
    // }
};