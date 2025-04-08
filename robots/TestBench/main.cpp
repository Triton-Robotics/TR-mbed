//STARTER 
//THIS EXERCISE IS TO BE DONE AND THEN RUN IN REAL LIFE ON A ROBOT
//YOU WILL NOT BE ABLE TO COMPILE THIS UNLESS YOU SET UP THE BUILD ENVIRONMENT
#include "main.h"
#include "subsystems/ChassisSubsystem.h"
#include <math.h>

DigitalOut led(L26);
DigitalOut led2(L27);
DigitalOut led3(L25);

I2C i2c(I2C_SDA, I2C_SCL);

//DEFINE MOTORS, ETC
const int RPM_MAX = 9000;
const int REMOTE_MAX = 660;
const int RPM_REMOTE_RATIO = RPM_MAX / REMOTE_MAX;

//DJIMotor* frontleft = new DJIMotor(1, CANHandler::CANBus::CANBUS_1, motorType::M3508, "FL");
//DJIMotor* frontright = new DJIMotor(2, CANHandler::CANBus::CANBUS_1, motorType::M3508, "FR");
//DJIMotor* backleft = new DJIMotor(3, CANHandler::CANBus::CANBUS_1, motorType::M3508, "BL");
//DJIMotor* backright = new DJIMotor(4, CANHandler::CANBus::CANBUS_1, motorType::M3508, "BR");


int main(){

    //assigning can handler objects to motor class.
    DJIMotor::s_setCANHandlers(&canHandler1,&canHandler2, false, false); 

    //getting initial feedback.
    DJIMotor::s_getFeedback();

    unsigned long loopTimer_u = us_ticker_read();
    unsigned long timeEnd_u;
    unsigned long timeStart_u;

    int refLoop = 0;

    //DEFINE PIDs AND OTHER CONSTANTS
    BNO055 imu(i2c, IMU_RESET, MODE_IMU);
    ChassisSubsystem Chassis(1, 2, 3, 4, imu, 0.2286); // radius is 9 inch
    ChassisSpeeds velocity = {0.0, 0.0, 0.0};
    ChassisSpeeds prev_velocity = {0.0, 0.0, 0.0};
    ChassisSpeeds accel = {0.0, 0.0, 0.0};

    std::vector<std::vector<float>> final_pos = {{0.0, 0.0}, {0.0, 200.0}, {200.0, 200.0}};

    float angle = 0.0;
    float posx = final_pos[0][0]; // need to go to 1676 ish
    float posy = final_pos[0][1]; // we need to go to -6800 (ish)
    int counter = 0;

    // final positions
    int idx = 1;
    float final_y = final_pos[idx][1];
    float final_x = final_pos[idx][0];

    // calculating final angle outside loop
    float final_angle = - atan(final_y/final_x) / PI;

    float buffer_y = 10.0;
    float buffer_x = 10.0;
    float buffer_angle = PI/16;


    float ly_init = 0.4;
    float lx_init = 0.4;
    float rx_init = 0.25; // you basically divide the angle you want by pi, so this is PI/4 / PI

    while(true){ //main loop
        timeStart_u = us_ticker_read();

        //inner loop runs every 25ms
        if((timeStart_u - loopTimer_u) / 1000 > 25) { 
            loopTimer_u = timeStart_u;
            led = !led; //led blink tells us how fast the inner loop is running

            if (refLoop >= 5) { //ref code runs 5 of every inner loop, 
                refLoop = 0;
                refereeThread(&referee);
            }
            refLoop ++;

            remoteRead(); //reading data from remote
        
            //MAIN CODE
            Chassis.periodic();
            velocity = Chassis.getChassisSpeeds();
            accel =  {(velocity.vX - prev_velocity.vX) / 25, (velocity.vY - prev_velocity.vY) / 25, (velocity.vOmega - prev_velocity.vOmega) / 25};

            // update pos and angle in mm
            // velocities in m/s, acceleration in m/s^2, the loop runs every 25 ms
            angle = angle + (velocity.vOmega * 0.025 + (1/2 * accel.vOmega * 0.025 * 0.025)) * PI;
            posy = posy + ((velocity.vY * cos(angle)) + (velocity.vX * sin(angle))) * 25 + (1/2 * ((accel.vY * cos(angle)) + (accel.vX * sin(angle))) * 25 * 0.025);
            posx = posx + ((velocity.vX * cos(angle)) - (velocity.vY * sin(angle))) * 25 + (1/2 * ((accel.vX * sin(angle)) - (accel.vY * cos(angle))) * 25 * 0.025);

            float lx = 0;
            float ly = 0;
            float rx = 0;

            // Purely changing x and y speeds, not rotation yet!
            //if (final_pos[idx][1] - posy > buffer_y) {
            //    ly = ly_init;
            //}   
            //else if (posy - final_pos[idx][1] > buffer_y) {
            //    ly = -ly_init;
            //}
            //else {
            //    ly = 0;
            //}

            //if (final_pos[idx][0] - posx > buffer_x) {
            //    lx = lx_init;
            //}
            //else if (posx - final_pos[idx][0] > buffer_x) {
            //    lx = -lx_init;
            //}
            //else {
            //    lx = 0;
            //}
            //if (lx == 0 && ly == 0) {
            //    if (idx < final_pos.size() - 1) {
            //        idx++;
            //    }
            //}
            //if (abs(remote.leftX()) < 150 && inc_counter == 0) {
            //    lx = remote.leftX();
            //}
            //if (abs(remote.rightX()) < 45) {
            //   rx = remote.rightX();
            //}
            lx = remote.leftX() / 660.0;
            ly = remote.leftY() / 660.0;
            Chassis.setChassisSpeeds({lx*Chassis.m_OmniKinematicsLimits.max_Vel, ly*Chassis.m_OmniKinematicsLimits.max_Vel, rx}); // changing angle is faster, but we havent made that code yet T_T
            prev_velocity = {velocity.vX, velocity.vY, velocity.vOmega};

            counter++;
            if (counter > 10) {
                printff("X: %.3f, Y: %.3f, A: %.3f, % \n", posx, posy, angle, idx);
                counter = 0;
            }
            //MOST CODE DOESNT NEED TO RUN FASTER THAN EVERY 25ms

            timeEnd_u = us_ticker_read();
            DJIMotor::s_sendValues();
        }
        //posy -= (velocity.vY + accel.vY * 0.0005);

        //FEEDBACK CODE DOES NEED TO RUN FASTER THAN 1MS
        //OTHER QUICK AND URGENT TASKS GO HERE

        DJIMotor::s_getFeedback();
        ThisThread::sleep_for(1ms);
    }
}