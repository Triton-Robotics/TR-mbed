#include "main.h"
#include "TestBench.h"
#include "subsystems/ChassisSubsystem.h"
#include <math.h>

#define PI 3.14159265
#define TIME 15
#define RADIUS 0.244475

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

//DEFINE MOTORS, ETC
const int RPM_MAX = 9000;
const int REMOTE_MAX = 660;
const int RPM_REMOTE_RATIO = RPM_MAX / REMOTE_MAX;

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, RADIUS); // radius is 9.625 in

float calculateDeltaYaw(float actual, float desired)
{
    float deltaYaw = desired - actual;

    if (abs(deltaYaw) > PI)
    {
        if (deltaYaw > 0)
            deltaYaw -= 2*PI;
        else
            deltaYaw += 2*PI;
    }
    return deltaYaw;
}

float calculateDistance(float posx, float posy, float final_x, float final_y) {
    return sqrt(pow((final_x - posx), 2) + pow((final_y - posy), 2));
}

int main(){

    //assigning can handler objects to motor class.
    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false); 

    //getting initial feedback.
    DJIMotor::s_getFeedback();

    unsigned long loopTimer_u = us_ticker_read();
    unsigned long timeEnd_u;
    unsigned long timeStart_u;

    int refLoop = 0;

    
    ChassisSpeeds velocity = {0.0, 0.0, 0.0};
    ChassisSpeeds prev_velocity = {0.0, 0.0, 0.0};
    ChassisSpeeds accel = {0.0, 0.0, 0.0};

    std::vector<std::vector<float>> final_pos = {{0.0, 0.0}, {500.0, 0.0}, {500.0, 500.0}, {0.0, 500.0}, {0.0, 0.0}};

    float angle = 0.0;
    float posx = final_pos[0][0]; // need to go to 1676 ish
    float posy = final_pos[0][1]; // we need to go to -6800 (ish)
    int counter = 0;

    // final positions
    int idx = 1;
    bool end = false;
    float final_y = final_pos[idx][1];
    float final_x = final_pos[idx][0];

    // calculating final angle outside loop
    float final_angle = atan2((final_pos[idx][1] - posy), (final_pos[idx][0] - posx));
    float final_angle_previous = 0.0;
    float buffer_y = 20.0;
    float buffer_x = 20.0;
    float buffer_angle = PI/16;

    float vel_init = 1.0; // should be max vel
    float accel_init = 0.4;
    float decel_dist = 1000 * vel_init * vel_init / (2 * accel_init * TIME); // in mm
    float rx_init = 0.4; // you basically divide the angle you want by pi, so this is PI/4 / PI

    while(true){ //main loop
        timeStart_u = us_ticker_read();

        //inner loop runs every 25ms
        if((timeStart_u - loopTimer_u) / 1000 > TIME) { 
            loopTimer_u = timeStart_u;
            led3 = !led3; //led blink tells us how fast the inner loop is running

            if (refLoop >= 5) { //ref code runs 5 of every inner loop, 
                refLoop = 0;
                refereeThread(&referee);
            }
            refLoop ++;

            remoteRead(); //reading data from remote
        
            //MAIN CODE
            Chassis.periodic();
            velocity = Chassis.getChassisSpeeds();
            accel =  {(velocity.vX - prev_velocity.vX) / (0.001 *TIME), (velocity.vY - prev_velocity.vY) / (0.001 *TIME), (velocity.vOmega - prev_velocity.vOmega) / (0.001 *TIME)};

            // update pos and angle in mm
            // velocities in m/s, acceleration in m/s^2, the loop runs every TIME ms
            angle = angle + ((velocity.vOmega * TIME * 0.001 + (1/2 * accel.vOmega * TIME * TIME * 0.001)) * PI);
            while (angle > PI) {
                angle -= 2*PI;
            }
            while (angle < -PI) {
                angle += 2*PI;
            }
            
            posy = posy + ((velocity.vY * sin(angle)) - (velocity.vX * cos(angle))) * TIME + (1/2 * ((accel.vY * sin(angle)) - (accel.vX * cos(angle))) * TIME * TIME * 0.001);
            
            posx = posx + ((velocity.vX * sin(angle)) + (velocity.vY * cos(angle))) * TIME + (1/2 * ((accel.vX * sin(angle)) + (accel.vY * cos(angle))) * TIME * TIME * 0.001);

            float lx = 0;
            float ly = 0;
            float rx = 0;

            if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN) {
                led = 1;
                lx = (remote.leftX() / 660.0) * Chassis.m_OmniKinematicsLimits.max_Vel;
                ly = (remote.leftY() / 660.0) * Chassis.m_OmniKinematicsLimits.max_Vel;
                rx = (remote.rightX() / 660.0);

                Chassis.setChassisSpeeds({lx, ly, rx});
            }
            else if (remote.rightSwitch() == Remote::SwitchState::UP) {
                led = 0;
                final_angle = atan2((final_pos[idx][1] - posy), (final_pos[idx][0] - posx));

                float deltayaw = calculateDeltaYaw(angle, final_angle);
                float distance = calculateDistance(posx, posy, final_x, final_y);

                if (angle > buffer_angle) {
                    rx = -rx_init;
                }
                else if (angle < -buffer_angle) {
                    rx = rx_init;
                }
                else {
                    rx = 0;
                }

                if ((final_y - posy) > buffer_y) {
                    if (abs(velocity.vX) < vel_init) {
                        lx = velocity.vX - accel_init;
                    }
                    else {
                        lx = -vel_init;
                    }

                    if (distance < decel_dist) {
                        if (velocity.vX < 0) {
                            lx = velocity.vX + accel_init;
                        }
                        else {
                            lx = 0;
                        }
                    }
                }
                else if ((posy - final_y) > buffer_y) {
                    if (abs(velocity.vX) < vel_init) {
                        lx = velocity.vX + accel_init;
                    }
                    else {
                        lx = vel_init;
                    }

                    if (distance < decel_dist) {
                        if (velocity.vX > 0) {
                            lx = velocity.vX - accel_init;
                        }
                        else {
                            lx = 0;
                        }
                    }
                }
                else {
                    lx = 0;
                }

                if ((final_x - posx) > buffer_x) {
                    if (velocity.vY < vel_init) {
                        ly = velocity.vY + accel_init;
                    }
                    else {
                        ly = vel_init;
                    }

                    if (distance < decel_dist) {
                        if (velocity.vY > 0) {
                            ly = velocity.vY - accel_init;
                        }
                        else {
                            ly = 0;
                        }
                    }
                }
                else if ((posx - final_x) > buffer_x) {
                    if (velocity.vY < -vel_init) {
                        ly = velocity.vY - accel_init;
                    }
                    else {
                        ly = -vel_init;
                    }

                    if (distance < decel_dist) {
                        if (velocity.vY < 0) {
                            ly = velocity.vY + accel_init;
                        }
                        else {
                            ly = 0;
                        }
                    }
                }
                else {
                    ly = 0;
                }

                if ((ly == 0 && lx == 0) || (distance < M_SQRT2 * buffer_x)) {
                    if (idx < final_pos.size() - 1) {
                        idx += 1;
                        final_y = final_pos[idx][1];
                        final_x = final_pos[idx][0];
                    }
                    else {
                        idx = 0;
                        final_y = final_pos[idx][1];
                        final_x = final_pos[idx][0];
                    }

                    ly = 0;
                    lx = 0;
                }

                Chassis.setChassisSpeeds({lx, ly, rx}); 
            }
            else{
                //OFF
                Chassis.setWheelPower({0,0,0,0});
            }
            
            prev_velocity = {velocity.vX, velocity.vY, velocity.vOmega};
            final_angle_previous = final_angle;

            counter++;
            if (counter > 10) {
                printff("X: %.3f, Y: %.3f, decel: %.3f, %.3f %d\n", posx, posy, angle, decel_dist, idx);
                //WheelSpeeds curr = Chassis.getWheelSpeeds();
                //printff("LF: %.3f RF: %.3f LB: %.3f RB: %.3f\n", curr.LF, curr.RF, curr.LB, curr.RB);
                counter = 0;
            }
            //MOST CODE DOESNT NEED TO RUN FASTER THAN EVERY 15ms

            timeEnd_u = us_ticker_read();
            DJIMotor::s_sendValues();
        }

        //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
        //OTHER QUICK AND URGENT TASKS GO HERE

        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}