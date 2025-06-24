#include "main.h"
#include "TestBench.h"
#include "subsystems/ChassisSubsystem.h"
#include <math.h>

#define PI 3.14159265
#define TIME 15
#define RADIUS 0.2286

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);


constexpr float BUFFER_ANGLE = PI / 16;
// PI/2 radps = 1 in vOmega terms (idk why), so we're converting PI/4 radps to 0.5 vOmega term
constexpr float ROT_INIT = 1; 
constexpr float ACCEL_INIT = 0.5;
constexpr float VEL_INIT = 1.2;
constexpr float DECEL_DIST =  VEL_INIT * VEL_INIT / (2 * ACCEL_INIT* TIME);
constexpr float BUFFER = 0.1;
float rotation = 0;

//CHASSIS DEFINING
I2C i2c(I2C_SDA, I2C_SCL);
BNO055 imu(i2c, IMU_RESET, MODE_IMU);
ChassisSubsystem Chassis(1, 2, 3, 4, imu, RADIUS); // radius is 9.625 in

struct SetValues {
    float lx;
    float ly;
    float rx;
};

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

SetValues calculate_chassis_speeds(float posx, float posy, float angle, float final_x, float final_y, float vX, float vY) {
    float distance = calculateDistance(posx, posy, final_x, final_y);
    SetValues values;
    values.lx = 0;
    values.ly = 0;
    values.rx = 0; 

    if (angle > BUFFER_ANGLE) {
        values.rx = -ROT_INIT;
    }
    else if (angle < -BUFFER_ANGLE) {
        values.rx = ROT_INIT;
    }
    else {
        values.rx = 0;
    }
    // printff("%.3f, %.3f\n", distance, DECEL_DIST);
    // if the final position is too far away:
    if ((final_x - posx) > BUFFER) {
        // if velocity is less than "max vel"
        if (abs(vX) < VEL_INIT) {
            // increase the velocity 
            values.lx = vX + ACCEL_INIT;
        }
        // velocity is too high, cap at 1
        else {
            values.lx = VEL_INIT;
        }

        // if we're within decelerating distance:
        if (abs(final_x - posx) < DECEL_DIST) {
            // if velocity is positive:
            if (vX > 0) {
                // decrease it by our acceleration cap
                values.lx = vX - ACCEL_INIT;
            }
            else {
                values.lx = 0;
            }
        }
    }
    // same logic as above, but this time final_x is negative
    else if ((posx - final_x) > BUFFER) {
        if (abs(vX) < VEL_INIT) {
            values.lx = vX - ACCEL_INIT;
        }
        else {
            values.lx = -VEL_INIT;
        }

        if (abs(final_x - posx) < DECEL_DIST) {
            if (vX < 0) {
                values.lx = vX + ACCEL_INIT;
            }
            else {
                values.lx = 0;
            }
        }
    }
    else {
        values.lx = 0;
    }
    

    // same logic as x
    if ((final_y - posy) > BUFFER) {
        if (abs(vY) < VEL_INIT) {
            values.ly = vY + ACCEL_INIT;
        }
        else {
            values.ly = VEL_INIT;
        }

        if (abs(final_y - posy) < DECEL_DIST) {
            if (vY > 0) {
                values.ly = vY - ACCEL_INIT;
            }
            else {
                values.ly = 0;
            }
        }
    }
    else if ((posy - final_y) > BUFFER) {
        if (abs(vY) < VEL_INIT) {
            values.ly = vY - ACCEL_INIT;
        }
        else {
            values.ly = -VEL_INIT;
        }

        if (abs(final_y - posy) < DECEL_DIST) {
            if (vY < 0) {
                values.ly = vY + ACCEL_INIT;
            }
            else {
                values.ly = 0;
            }
        }
    }
    else {
        values.ly = 0;
    }

    return values;
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

    // set PIDs for sentry motors specifically
    Chassis.setMotorSpeedPID(ChassisSubsystem::LEFT_FRONT, 3.6, 0, 0);
    Chassis.setMotorSpeedPID(ChassisSubsystem::LEFT_BACK, 3.6, 0, 0);
    Chassis.setMotorSpeedPID(ChassisSubsystem::RIGHT_FRONT, 3, 0, 0);
    Chassis.setMotorSpeedPID(ChassisSubsystem::RIGHT_BACK, 3, 0, 0);


    // std::vector<std::vector<float>> final_pos = {{0.0, 0.0}, {0.0, 4020.0}, {-1000.0, 4020}}; 
    // std::vector<std::vector<float>> final_pos = {{0.0, 0.0}, {0.0, 2*0.588}};
    std::vector<std::vector<float>> final_pos = {{0.0, 0.0}, {0.0,1.0}, {0,0}, {1,0}, {0,0}};


    float angle = 0.0;
    float posx = final_pos[0][0]; // need to go to 1676 ish
    float posy = final_pos[0][1]; // we need to go to -6800 (ish)
    int counter = 0;
    int beyblade_counter = 0;
    int settle_counter = 0;
    
    // final positions
    int idx = 1;
    float final_y = final_pos[idx][1];
    float final_x = final_pos[idx][0];

    float rx_init = 0.4; // you basically divide the angle you want by pi, so this is PI/4 / PI

    // ensure robot is not moving at startup
    Chassis.setWheelPower({0,0,0,0});

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
            accel =  {(velocity.vX     - prev_velocity.vX)     / (TIME*0.001), 
                      (velocity.vY     - prev_velocity.vY)     / (TIME*0.001), 
                      (velocity.vOmega - prev_velocity.vOmega) / (TIME*0.001)}; 
            
            // angle is always 0 for robot oriented drive
            angle = angle + ((velocity.vOmega * TIME * 0.001) + (1/2 * accel.vOmega * TIME * 0.001 * TIME * 0.001)) * PI / 2;
            
            if (angle > PI) {
                angle -= 2 * PI;
            }
            else if (angle < -PI) {
                angle += 2 * PI;
            }

            posx = posx + ((velocity.vX * cos(angle) + velocity.vY * sin(angle)) * TIME * 0.001
                + (0.5 * (accel.vX * cos(angle) + accel.vY * sin(angle)) * TIME * 0.001 * TIME * 0.001)) * 1.6;

            posy = posy + ((velocity.vY * cos(angle) - velocity.vX * sin(angle)) * TIME * 0.001
                + (0.5 * (accel.vY * cos(angle) - accel.vX * sin(angle)) * TIME * 0.001 * TIME * 0.001)) * 1.6;

            float lx = 0;
            float ly = 0;
            float rx = 0;

            // game_progress == 4 means "in competition"
            if (true){//if (game_status.game_progress == 4) {
                if (remote.rightSwitch() == Remote::SwitchState::MID || remote.rightSwitch() == Remote::SwitchState::UNKNOWN) {
                    led = 1;
                    lx = (remote.leftX() / 660.0) * Chassis.m_OmniKinematicsLimits.max_Vel;
                    ly = (remote.leftY() / 660.0) * Chassis.m_OmniKinematicsLimits.max_Vel;
                    // rx = (remote.rightX() / 660.0);

                    Chassis.setChassisSpeeds({lx, ly, 0}, ChassisSubsystem::ROBOT_ORIENTED);
                }
                else if (remote.rightSwitch() == Remote::SwitchState::UP) {
                    led = 0;
                    float distance = calculateDistance(posx, posy, final_x, final_y);

                    SetValues values = calculate_chassis_speeds(posx, posy, angle, final_x, final_y, velocity.vX, velocity.vY);

                    // If robot has stopped moving or its close to its setpoint:
                    if ((distance < M_SQRT2 * BUFFER)) {
                        settle_counter+= TIME;
                        // if hp is greater than 20%, go to next setpoint
                        if (robot_status.current_HP >= robot_status.maximum_HP * 0.2) {

                            // if we are not at the final setpoint and 300ms have passed, move to the next setpoint
                            if ((idx < final_pos.size() - 1) && (settle_counter > 300)) {
                                    idx += 1;
                                    final_y = final_pos[idx][1];
                                    final_x = final_pos[idx][0];
                            }
                        }

                        // hp low, so go to previous setpoint
                        else {
                            if (idx > 0) {
                                idx -= 1;
                                final_y = final_pos[idx][1];
                                final_x = final_pos[idx][0];
                            }
                        }
                        values.ly = 0;
                        values.lx = 0;
                    }
                    
                    // we've reached the last setpoint, if we stopped moving, then start beyblading
                    if (idx == final_pos.size() - 1 && velocity.vX == 0 && velocity.vY == 0) {
                        beyblade_counter++;
                    } 

                    if (beyblade_counter > 10) {
                        values.rx = 3;
                    }
                    Chassis.setChassisSpeeds({values.lx, values.ly, values.rx}, ChassisSubsystem::ROBOT_ORIENTED);
                }
                else {
                    //OFF
                    Chassis.setWheelPower({0,0,0,0});
                }
                
                prev_velocity = {velocity.vX, velocity.vY, velocity.vOmega};
            }


            counter++;

            if (counter > 10) {
                // printff("%d\n", idx);
                printff("X:%.3f Y:%.3f a:%.3f fx:%.3f fy:%.3f\n", posx, posy, angle, final_x, final_y);
                // printff("vX:%.2f,vY:%.2f,vO:%.2f",velocity.vX, velocity.vY, velocity.vOmega);
                WheelSpeeds curr = Chassis.getWheelSpeeds();
                // printff("LF: %.3f RF: %.3f LB: %.3f RB: %.3f\n", curr.LF, curr.RF, curr.LB, curr.RB);
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