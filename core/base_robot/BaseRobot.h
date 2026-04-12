#pragma once

#include "PinNames.h"
#include "mbed.h"
#include "util/communications/DJIRemote.h"
#include "util/communications/referee/ref_serial.h"
#include "util/motor/DJIMotor.h"
#include <us_ticker_api.h>

class BaseRobot {
  public:
    struct Config {
    //PinName remote_pin = PA_10;

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

	// struct ConfigNew {
	// 	PinName remote_rx = PA_3;
	// 	PinName remote_tx = PA_2;
	// };

    //Remote remote_;
    //Referee referee;

    CANHandler canHandler1_;
    CANHandler canHandler2_;
    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;

    // Remote control variables
    float scalar = 1;
    float jx = 0; // -1 to 1
    float jy = 0; // -1 to 1
    // Pitch, Yaw
    float jpitch = 0; // -1 to 1
    float jyaw = 0; // -1 to 1
    float myaw = 0;
    float mpitch = 0;
    int pitchVelo = 0;
    // joystick tolerance
    float tolerance = 0.05;
    // Keyboard Driving
    float mult = 0.7;
    float omega_speed = 0;
    float max_linear_vel = 0;

    // drive and shooting mode
    // TODO rename these chars?????
    // "o" - joystick
    // "u" - drive
    // "d" - beyblade
    // "m" - off

    // "o" - joystick
    // "d" - flywheel
    // "m" - flywheel off
    char drive = 'o'; //default o when using joystick 
    char shot = 'o'; //default o when using joystick
    bool cv_enabled = false; // TODO add to inf main maybe? altho i think cv should always be on

    // clang-format off
    BaseRobot(const Config &config)
        : //referee(config.referee_tx_pin, config.referee_rx_pin), 
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
        // Each can message has an id and data, and djimotor ids start from 0x201(m3508 id 1) and
        // end at 0x20D (gm6020 id 8), there is an overlap of 4 motors (M3508 id 5-8 and gm6020 id
        // 1-4), so that is why we have 12 values only
        canHandler1_.registerCallback(0x201, 0x20D, DJIMotor::getCan1Feedback);
        canHandler2_.registerCallback(0x201, 0x20D, DJIMotor::getCan2Feedback);
        DJIMotor::setCanHandlers(&canHandler1_, &canHandler2_);

        init();

        while (true) {
            // TODO StmIO comms (Ref and Jetson)

            loop_clock_us = us_ticker_read();

            // 20 ms remote read
            if ((loop_clock_us - prev_remote_time_us) / 1000 >= 15) {
                //remote_.read();
                //remoteRead();
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

    // void remoteRead()
    // {
    //     //Keyboard-based drive and shoot mode
    //     if(remote_.keyPressed(Remote::Key::R)){
    //         drive = 'm';
    //     }else if(remote_.keyPressed(Remote::Key::E)){
    //         drive = 'u';
    //     }else if(remote_.keyPressed(Remote::Key::Q)){
    //         drive = 'd';        
    //     }

    //     if(remote_.keyPressed(Remote::Key::V)){
    //         shot = 'm';
    //     }else if(remote_.keyPressed(Remote::Key::C)){
    //         shot = 'd';        
    //     }
        
    //     if(remote_.getMouseR() || remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID){
    //         cv_enabled = true;
    //     }else if(!remote_.getMouseR() ){
    //         cv_enabled = false;
    //     }

    //     //Driving input
    //     scalar = 1;
    //     jx = remote_.leftX() * scalar; // -1 to 1
    //     jy = remote_.leftY() * scalar; // -1 to 1
    //     //Pitch, Yaw
    //     jpitch = remote_.rightY() * scalar; // -1 to 1
    //     jyaw = remote_.rightX() * scalar; // -1 to 1

    //     myaw = remote_.getMouseX();
    //     mpitch = -remote_.getMouseY();

    //     jx = (abs(jx) < tolerance) ? 0 : jx;
    //     jy = (abs(jy) < tolerance) ? 0 : jy;
    //     jpitch = (abs(jpitch) < tolerance) ? 0 : jpitch;
    //     jyaw = (abs(jyaw) < tolerance) ? 0 : jyaw;
        

    //     // Shift to make robot go slower
    //     if (remote_.keyPressed(Remote::Key::SHIFT)) {
    //         mult = 0.5;
    //     }
    //     if(remote_.keyPressed(Remote::Key::CTRL)){
    //         mult = 1;
    //     }

    //     jx += mult * ((remote_.keyPressed(Remote::Key::D) ? 1 : 0) + (remote_.keyPressed(Remote::Key::A) ? -1 : 0));
    //     jy += mult * ((remote_.keyPressed(Remote::Key::W) ? 1 : 0) + (remote_.keyPressed(Remote::Key::S) ? -1 : 0));

    //     float j_hypo = sqrt(jx * jx + jy * jy);
    //     if(j_hypo > 1.0){
    //         jx = jx / j_hypo;
    //         jy = jy / j_hypo;
    //     }
    //     //Bounding the four j variables
    //     jx = max(-1.0F, min(1.0F, jx));
    //     jy = max(-1.0F, min(1.0F, jy));
    //     jpitch = max(-1.0F, min(1.0F, jpitch));
    //     jyaw = max(-1.0F, min(1.0F, jyaw));

    //     // max_linear_vel = -1.24 + 0.0513 * chassis.power_limit + -0.000216 * (chassis.power_limit * chassis.power_limit);
    //     // float max_omega = 0.326 + 0.0857 * chassis_power_limit + -0.000183 * (chassis_power_limit * chassis_power_limit);
    //     float max_omega = 4.8;

    //     if(remote_.keyPressed(Remote::Key::CTRL)){
    //         jx = 0.0;
    //         jy = 0.0;
    //         max_omega = 6.1;
    //     }

    //     float linear_hypo = sqrtf(jx * jx + jy * jy);
    //     if(linear_hypo > 0.8){
    //         linear_hypo = 0.8;
    //     }

    //     float available_beyblade = 1.0 - linear_hypo;
    //     omega_speed = max_omega * available_beyblade;
    // }
};