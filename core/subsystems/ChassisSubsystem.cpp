#include "ChassisSubsystem.h"
#include <cmath>
#include <stdexcept>

/**
 * @param radius radius in meters
 */
ChassisSubsystem::ChassisSubsystem(short lfId, short rfId, short lbId, short rbId, BNO055 &imu, double radius)
    : LF(lfId, CAN_BUS_TYPE, MOTOR_TYPE),
      RF(rfId, CAN_BUS_TYPE, MOTOR_TYPE),
      LB(lbId, CAN_BUS_TYPE, MOTOR_TYPE),
      RB(rbId, CAN_BUS_TYPE, MOTOR_TYPE),
      imu(imu),
      power_limit(50.0F),
      chassis_radius(radius)
// chassisKalman()
{
    LF.outputCap = 16000; // DJIMotor class has a max outputCap: 16384
    RF.outputCap = 16000;
    LB.outputCap = 16000;
    RB.outputCap = 16000;

    this->lfId = lfId;
    this->rfId = rfId;
    this->lbId = lbId;
    this->rbId = rbId;

    setOmniKinematics(radius);
    m_OmniKinematicsLimits.max_Vel = MAX_VEL; // m/s
    m_OmniKinematicsLimits.max_vOmega = 8; // rad/s

    PEAK_POWER_ALL = 10000;
    PEAK_POWER_SINGLE = 8000;

    FF_Ks = 0;

//    LF.setSpeedPID(2, 0, 0);
//    RF.setSpeedPID(2, 0, 0);
//    LB.setSpeedPID(2, 0, 0);
//    RB.setSpeedPID(2, 0, 0);
    // LF.setSpeedPID(3, 0, 0);
    // RF.setSpeedPID(3, 0, 0);
    // LB.setSpeedPID(3, 0, 0);
    // RB.setSpeedPID(3 , 0, 0);

    pid_LF.setPID(2.45, 0, 0);
    pid_RF.setPID(2.45, 0, 0);
    pid_LB.setPID(2.45, 0, 0);
    pid_RB.setPID(2.45, 0, 0);

    brakeMode = COAST;

    // isInverted[0] = 1; isInverted[1] = 1; isInverted[2] = 1; isInverted[3] = 1;
}

WheelSpeeds ChassisSubsystem::getWheelSpeeds() const
{
    return m_wheelSpeeds;
}

float ChassisSubsystem::limitAcceleration(float desiredRPM, float previousRPM, int power)
{
    float maxAccel = 100; // at 15ms outer_loop_dt_ms, should be 100. everything is scaled off this
    float diff = desiredRPM - previousRPM;

    
    if ((desiredRPM > 0 && previousRPM < 0) || (desiredRPM < 0 && previousRPM > 0)) { // if robot trying to sudden change direction
        return 0;
    }
    
    
    if (diff > maxAccel){   // if the difference is greater than the max acceleration


        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }

        return previousRPM + maxAccel;

    }
    else if (diff < -maxAccel) {
        if(power == 0) {
            return desiredRPM; // let robot do its thing b/c it wont take power
        }

        return previousRPM - maxAccel;
    }

    else {
        return desiredRPM; // under acceleration cap
    }




    // if (diff > maxAccel){   // if the difference is greater than the max acceleration

    //     if(power > 0) { // power: + rpm: + (acceleraiton)
    //         return previousRPM + maxAccel;
    //     }

    //     else if(power < 0 && desiredRPM > 0) { // power: - rpm: + (deceleration)
    //         return previousRPM + maxAccel; // ignore diff, just decelerate 
    //     }

    //     else if(power == 0) {
    //         return desiredRPM; // let robot do its thing b/c it wont take power
    //     }

    // }
    // else if (diff < -maxAccel) {
        
    //     if(power < 0 && desiredRPM < 0) { // power: - rpm: - (acceleration)
    //         return previousRPM - maxAccel; 
    //     }

    //     else if(power > 0 && desiredRPM < 0) { // power: + rpm: - (deceleration)
    //         return previousRPM - maxAccel; // ignore diff, just decelerate
    //     }

    //     else if(power == 0) {
    //         return desiredRPM; // let robot do its thing b/c it wont take power
    //     }

    // }
    // else {
    //     return desiredRPM; // under acceleration
    // }









    // if(previousRPM > 0){
    //     float maxRPMthisFrame = previousRPM + maxAccel;
    //     if(desiredRPM > 0){
    //         if(desiredRPM > maxRPMthisFrame){
    //             return maxRPMthisFrame;
    //         }else {
    //             return desiredRPM;
    //         }
    //     }else {
    //         return 0;
    //     }
    // }else if(previousRPM < 0){
    //     float minRPMthisFrame = previousRPM - maxAccel;
    //     if(desiredRPM < 0){
    //         if(desiredRPM < minRPMthisFrame){
    //             return minRPMthisFrame;
    //         }else {
    //             return desiredRPM;
    //         }
    //     }else{
    //         return 0;
    //     }
    // }else{
    //     return 0;
    // }
}

float ChassisSubsystem::p_theory(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm){
    float krpm2 = 0.000000000616869908524917;
    float kpwr2 = 2.8873053310419543e-26;
    float kboth = 0.00000000679867734389254;
    float a = 0.019247609510979;


    float p1 =  (kboth * LeftFrontPower * LeftFrontRpm) +  (krpm2 * LeftFrontRpm * LeftFrontRpm) +  (kpwr2 * LeftFrontPower * LeftFrontPower) + a;
    float p2 =  (kboth * RightFrontPower * RightFrontRpm) +  (krpm2 * RightFrontRpm * RightFrontRpm) +  (kpwr2 * RightFrontPower * RightFrontPower) + a;
    float p3 =  (kboth * LeftBackPower * LeftBackRpm) +  (krpm2 * LeftBackRpm * LeftBackRpm) +  (kpwr2 * LeftBackPower * LeftBackPower) + a;
    float p4 =  (kboth * RightBackPower * RightBackRpm) +  (krpm2 * RightBackRpm * RightBackRpm) +  (kpwr2 * RightBackPower * RightBackPower) + a;
    
    float p_tot = p1 + p2 + p3 + p4; 

    float A = 224.9;
    float B = 215.8;
    float C = 0.7955;

    float p_tot_c = (A * p_tot * p_tot) + (B * p_tot) + C;

    return p_tot_c;
}

float ChassisSubsystem::Bisection(int LeftFrontPower, int RightFrontPower, int LeftBackPower, int RightBackPower, int LeftFrontRpm, int RightFrontRpm, int LeftBackRpm, int RightBackRpm, float chassisPowerLimit) {
    float scale = 0.5; // initial scale
    float precision = 0.25; // initial precision
    float powerInit = p_theory(LeftFrontPower, RightFrontPower, LeftBackPower, RightBackPower, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);


    if (powerInit > chassisPowerLimit) {

        float powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
        
        for (int i = 0; i < 6; i++) {
            
            // if (abs(powerScaled - chassisPowerLimit) < 0.1) {
            //     printf("ppppp \n");
            //     break;
            // }
            if (powerScaled > chassisPowerLimit) {
                scale = scale - precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Down: %f\n", powerScaled);
            }

            else { // power is low enough
                scale = scale + precision;
                precision = precision / 2;
                powerScaled = p_theory(LeftFrontPower*scale, RightFrontPower*scale, LeftBackPower*scale, RightBackPower*scale, LeftFrontRpm, RightFrontRpm, LeftBackRpm, RightBackRpm);
                // printf("powerScaled Up: %f\n", powerScaled);
            }
        }
        return scale;
    }

    else {
        return 1;
    }

}

float ChassisSubsystem::setWheelSpeeds(WheelSpeeds wheelSpeeds, float pwr_limit)
{
    desiredWheelSpeeds = wheelSpeeds; // WheelSpeeds in RPM
    int powers[4] = {0,0,0,0};
    uint32_t time = us_ticker_read();


    powers[0] = motorPIDtoPower(LEFT_FRONT,wheelSpeeds.LF, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,wheelSpeeds.RF, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,wheelSpeeds.LB, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,wheelSpeeds.RB, (time - lastPIDTime));
    

    float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], powers[0]);
    float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], powers[1]);
    float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], powers[2]);
    float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], powers[3]);

    // float LFrpm = wheelSpeeds.LF;
    // float RFrpm = wheelSpeeds.RF;
    // float LBrpm = wheelSpeeds.LB;
    // float RBrpm = wheelSpeeds.RB;

    
    // float LFrpm = limitAcceleration(wheelSpeeds.LF, previousRPM[0], LF.getData(POWEROUT));
    // float RFrpm = limitAcceleration(wheelSpeeds.RF, previousRPM[1], RF.getData(POWEROUT));
    // float LBrpm = limitAcceleration(wheelSpeeds.LB, previousRPM[2], LB.getData(POWEROUT));
    // float RBrpm = limitAcceleration(wheelSpeeds.RB, previousRPM[3], RB.getData(POWEROUT));

    previousRPM[0] = LFrpm;
    previousRPM[1] = RFrpm;
    previousRPM[2] = LBrpm;
    previousRPM[3] = RBrpm;


    

    powers[0] = motorPIDtoPower(LEFT_FRONT,LFrpm, (time - lastPIDTime));
    powers[1] = motorPIDtoPower(RIGHT_FRONT,RFrpm, (time - lastPIDTime));
    powers[2] = motorPIDtoPower(LEFT_BACK,LBrpm, (time - lastPIDTime));
    powers[3] = motorPIDtoPower(RIGHT_BACK,RBrpm, (time - lastPIDTime));
    lastPIDTime = time;


    // int sum = powers[0] + powers[1] + powers[2] + powers[3];

    // if(sum > PEAK_POWER_ALL){
    //     powers[0] = (powers[0] * PEAK_POWER_ALL)/sum;
    //     powers[1] = (powers[1] * PEAK_POWER_ALL)/sum;
    //     powers[2] = (powers[2] * PEAK_POWER_ALL)/sum;
    //     powers[3] = (powers[3] * PEAK_POWER_ALL)/sum;
    // }

    int p1 = abs(powers[0]);
    int p2 = abs(powers[1]);
    int p3 = abs(powers[2]);
    int p4 = abs(powers[3]);

    
    int r1 = abs(LF.getData(VELOCITY));
    int r2 = abs(RF.getData(VELOCITY));
    int r3 = abs(LB.getData(VELOCITY));
    int r4 = abs(RB.getData(VELOCITY));

    float scale = Bisection(p1, p2, p3, p4, r1, r2, r3, r4, pwr_limit);



    

    // printf("Before Set:%.3f\n", p_theory(p1*scale, p2*scale, p3*scale, p4*scale, r1, r2, r3, r4));

    LF.setPower(powers[0]*scale);
    RF.setPower(powers[1]*scale);
    LB.setPower(powers[2]*scale);
    RB.setPower(powers[3]*scale);

    p1 = abs(LF.getData(POWEROUT));
    p2 = abs(RF.getData(POWEROUT));
    p3 = abs(LB.getData(POWEROUT));
    p4 = abs(RB.getData(POWEROUT));

    // printf("After Set:%.3f\n", p_theory(p1, p2, p3, p4, r1, r2, r3, r4));


    
    return scale;
}

WheelSpeeds ChassisSubsystem::normalizeWheelSpeeds(WheelSpeeds wheelSpeeds) const
{
    double speeds[4] = {wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB};
    double max_speed = m_OmniKinematicsLimits.max_Vel;

    for (double speed : speeds)
        if (speed > max_speed)
            max_speed = speed;

    if (max_speed > m_OmniKinematicsLimits.max_Vel)
        for (double &speed : speeds)
            speed = speed / max_speed * m_OmniKinematicsLimits.max_Vel;

    return {speeds[0], speeds[1], speeds[2], speeds[3]};
}


void ChassisSubsystem::setWheelPower(WheelSpeeds wheelPower)
{
    setMotorPower(LEFT_FRONT, wheelPower.LF);
    setMotorPower(RIGHT_FRONT, wheelPower.RF);
    setMotorPower(LEFT_BACK, wheelPower.LB);
    setMotorPower(RIGHT_BACK, wheelPower.RB);
}

ChassisSpeeds ChassisSubsystem::getChassisSpeeds() const
{
    return m_chassisSpeeds;
}

float ChassisSubsystem::setChassisSpeeds(ChassisSpeeds desiredChassisSpeeds_, float pwr_limit, DRIVE_MODE mode)
{
    if (mode == REVERSE_YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        double yawCurrent = (1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == YAW_ORIENTED)
    {
        // printf("%f\n", double(yaw->getData(ANGLE)));
        double yawCurrent = -(1.0 - (double(yaw->getData(ANGLE)) / TICKS_REVOLUTION)) * 360.0; // change Yaw to CCW +, and ranges from 0 to 360
        desiredChassisSpeeds = rotateChassisSpeed(desiredChassisSpeeds_, yawCurrent);
    }
    else if (mode == ROBOT_ORIENTED)
    {
        desiredChassisSpeeds = desiredChassisSpeeds_; // ChassisSpeeds in m/s
    }
    WheelSpeeds wheelSpeeds = chassisSpeedsToWheelSpeeds(desiredChassisSpeeds); // in m/s (for now)
    wheelSpeeds = normalizeWheelSpeeds(wheelSpeeds);
    wheelSpeeds *= (1 / (WHEEL_DIAMETER_METERS / 2) / (2 * PI / 60) * M3508_GEAR_RATIO);
    float scale = setWheelSpeeds(wheelSpeeds, pwr_limit);
    return scale;
}

/**
 * There's no setChassisPower because it doesn't make sense.
 * Power (is not PWM voltage) saturates your motor speeds, and it's not related to motor speed. 
 */

ChassisSpeeds ChassisSubsystem::rotateChassisSpeed(ChassisSpeeds speeds, double yawCurrent)
{
    // rotate angle counter clockwise
    double theta = (yawCurrent - yawPhase) / 180 * PI;
    return {speeds.vX * cos(theta) - speeds.vY * sin(theta),
            speeds.vX * sin(theta) + speeds.vY * cos(theta),
            speeds.vOmega};
}

DJIMotor &ChassisSubsystem::getMotor(MotorLocation location)
{
    switch (location)
    {
    case LEFT_FRONT:
        return LF;
    case RIGHT_FRONT:
        return RF;
    case LEFT_BACK:
        return LB;
    case RIGHT_BACK:
        return RB;
    }
}

void ChassisSubsystem::setMotorSpeedPID(MotorLocation location, float kP, float kI, float kD)
{
    getMotor(location).setSpeedPID(kP, kI, kD);
}

void ChassisSubsystem::setSpeedIntegralCap(MotorLocation location, double cap)
{
    getMotor(location).setSpeedIntegralCap(cap);
}

void ChassisSubsystem::setSpeedFeedforward(MotorLocation location, double FF)
{
    // getMotor(location).pidSpeed.feedForward = FF * INT15_T_MAX;
    if(location == LEFT_FRONT){
        pid_LF.feedForward = FF * INT15_T_MAX;
    }else if(location == LEFT_BACK){
        pid_LB.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_FRONT){
        pid_RF.feedForward = FF * INT15_T_MAX;
    }else if(location == RIGHT_BACK){
        pid_RB.feedForward = FF * INT15_T_MAX;
    }
}

void ChassisSubsystem::setSpeedFF_Ks(double Ks)
{
    FF_Ks = Ks;
}

ChassisSubsystem::BrakeMode ChassisSubsystem::getBrakeMode()
{
    return brakeMode;
}

void ChassisSubsystem::setBrakeMode(BrakeMode brakeMode)
{
    this->brakeMode = brakeMode;
}

void ChassisSubsystem::initializeImu()
{
    imu.set_mounting_position(MT_P1);
}

double ChassisSubsystem::getMotorSpeed(MotorLocation location, SPEED_UNIT unit = RPM)
{
    double speed = getMotor(location).getData(VELOCITY);
    switch (unit)
    {
    case RPM:
        return speed;
    case METER_PER_SECOND:
        return (speed / M3508_GEAR_RATIO) * (2 * PI / 60) * (WHEEL_DIAMETER_METERS / 2);
    }
}

void ChassisSubsystem::readImu()
{
    imu.get_angular_position_quat(&imuAngles);
}

void ChassisSubsystem::periodic()
{   
    // readImu();
    m_wheelSpeeds = {getMotorSpeed(LEFT_FRONT, METER_PER_SECOND), getMotorSpeed(RIGHT_FRONT, METER_PER_SECOND),
                     getMotorSpeed(LEFT_BACK, METER_PER_SECOND), getMotorSpeed(RIGHT_BACK, METER_PER_SECOND)};

    m_chassisSpeeds = wheelSpeedsToChassisSpeeds(m_wheelSpeeds);
}

double ChassisSubsystem::degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

double ChassisSubsystem::radiansToDegrees(double radians)
{
    return radians / PI * 180.0;
}

int ChassisSubsystem::getHeadingDegrees() const
{
    return (int)imuAngles.yaw;
}

void ChassisSubsystem::setOmniKinematicsLimits(double max_Vel, double max_vOmega)
{
    m_OmniKinematicsLimits.max_Vel = max_Vel;
    m_OmniKinematicsLimits.max_vOmega = max_vOmega;
}

//NEW STUFF FROM THIS PAPER: https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

// void ChassisSubsystem::setOmniKinematics(double radius)
// {
//     float SQRT_2 = sqrt(2);
//     m_OmniKinematics.r1x = -sqrt(radius);
//     m_OmniKinematics.r1y = sqrt(radius);

//     m_OmniKinematics.r2x = sqrt(radius);
//     m_OmniKinematics.r2y = sqrt(radius);

//     m_OmniKinematics.r3x = -sqrt(radius);
//     m_OmniKinematics.r3y = -sqrt(radius);

//     m_OmniKinematics.r4x = sqrt(radius);
//     m_OmniKinematics.r4y = -sqrt(radius);
// }

void ChassisSubsystem::setOmniKinematics(double radius)
{
    float SQRT_2 = sqrt(2);
    m_OmniKinematics.r1x = radius/SQRT_2;
    m_OmniKinematics.r1y = radius/SQRT_2;

    m_OmniKinematics.r2x = radius/SQRT_2;
    m_OmniKinematics.r2y = radius/SQRT_2;

    m_OmniKinematics.r3x = radius/SQRT_2;
    m_OmniKinematics.r3y = radius/SQRT_2;

    m_OmniKinematics.r4x = radius/SQRT_2;
    m_OmniKinematics.r4y = radius/SQRT_2;
}

// WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
// {
//     return {(1 / sqrt(2)) * (chassisSpeeds.vX + chassisSpeeds.vY + chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y))),
//             (1 / sqrt(2)) * (chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
//             (1 / sqrt(2)) * (-chassisSpeeds.vX + chassisSpeeds.vY + chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
//             (1 / sqrt(2)) * (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y)))};
// }

//inputs chassis speeds in m/s, outputs wheel speeds in m/s
WheelSpeeds ChassisSubsystem::chassisSpeedsToWheelSpeeds(ChassisSpeeds chassisSpeeds)
{
    return {(chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r1x) + (m_OmniKinematics.r1y))),
            (chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y))),
            (-chassisSpeeds.vX + chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y))),
            (-chassisSpeeds.vX - chassisSpeeds.vY - chassisSpeeds.vOmega * ((m_OmniKinematics.r4x) + (m_OmniKinematics.r4y)))};
}

// ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
// {
//     Eigen::MatrixXd Inv_K(4, 3);
//     float coef = 1 / sqrt(2);
//     Inv_K << coef, coef, coef * ((m_OmniKinematics.r1x) - (m_OmniKinematics.r1y)),
//         coef, -coef, -coef * ((m_OmniKinematics.r2x) + (m_OmniKinematics.r2y)),
//         -coef, coef, coef * ((m_OmniKinematics.r3x) + (m_OmniKinematics.r3y)),
//         -coef, -coef, -coef * ((m_OmniKinematics.r4x) - (m_OmniKinematics.r4y));
//     Eigen::MatrixXd FWD_K(3, 4);
//     FWD_K = Inv_K.completeOrthogonalDecomposition().pseudoInverse();
//     // printf("[%.1f %.1f %.1f %.1f\n%.1f %.1f %.1f %.1f\n%.1f %.1f %.1f %.1f]\n", 
//     //     FWD_K(0,0), FWD_K(0,1), FWD_K(0,2), FWD_K(0,3), 
//     //     FWD_K(1,0), FWD_K(1,1), FWD_K(1,2), FWD_K(1,3), 
//     //     FWD_K(2,0), FWD_K(2,1), FWD_K(2,2), FWD_K(2,3));
//     Eigen::MatrixXd WS(4, 1);
//     WS << wheelSpeeds.LF, wheelSpeeds.RF, wheelSpeeds.LB, wheelSpeeds.RB;
//     Eigen::MatrixXd CS(3, 1);
//     CS = FWD_K * WS;
//     return {CS(0, 0), CS(1, 0), CS(2, 0)};
// }

ChassisSpeeds ChassisSubsystem::wheelSpeedsToChassisSpeeds(WheelSpeeds wheelSpeeds)
{
    float dist = chassis_radius/sqrt(2);
    float vX = (wheelSpeeds.LF + wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vY = (wheelSpeeds.LF - wheelSpeeds.RF + wheelSpeeds.LB - wheelSpeeds.RB) / 4;
    float vOmega = (-wheelSpeeds.LF - wheelSpeeds.RF - wheelSpeeds.LB - wheelSpeeds.RB) / (4*(2 * dist));
    return {vX, vY, vOmega};
}

char *ChassisSubsystem::MatrixtoString(Eigen::MatrixXd mat)
{
    std::stringstream ss;
    ss << mat;
    ss << '\0';
    ss << '\n';
    const char *a = ss.str().c_str();
    return strdup(a);
}

void ChassisSubsystem::setMotorPower(MotorLocation location, double power)
{
    if (brakeMode == BRAKE && power == 0) // Should be BRAKE
    {
        getMotor(location).setSpeed(0);
        setSpeedFeedforward(location, 0);
        return;
    }
    getMotor(location).setPower(power * INT15_T_MAX);
}

void ChassisSubsystem::setMotorSpeedRPM(MotorLocation location, double speed)
{
    if (brakeMode == COAST && speed == 0)
    {
        setMotorPower(location, 0);
        setSpeedFeedforward(location, 0);
        return;
    }
    getMotor(location).setSpeed(speed);
    double sgn_speed = speed / abs(speed); // if speed is 0, it won't execute this line
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
}

int ChassisSubsystem::motorPIDtoPower(MotorLocation location, double speed, uint32_t dt)
{
    if (brakeMode == COAST && speed == 0)
    {
        setSpeedFeedforward(location, 0);
        return 0;
    }
    
    int power = 0;
    PID pids[4] = {pid_LF,pid_RF,pid_LB,pid_RB};
    
    power = pids[location].calculate(speed, getMotor(location).getData(VELOCITY), dt);
    // printf("[%d]",power);

    if(speed == 0) {
        setSpeedFeedforward(location, 0);
        return power;
    }
    double sgn_speed = speed / abs(speed); // if speed is 0, it won't execute this line
    setSpeedFeedforward(location, FF_Ks * sgn_speed);
    return power;
}

void ChassisSubsystem::setYawReference(DJIMotor *motor, double initial_offset_ticks)
{
    yaw = motor;
    yawPhase = 360.0 * (1.0 - (initial_offset_ticks / TICKS_REVOLUTION)); // change Yaw to CCW +, and ranges from 0 to 360
}

double ChassisSubsystem::radiansToTicks(double radians)
{
    return radians / (2 * PI) * TICKS_PER_ROTATION;
}

double ChassisSubsystem::ticksToRadians(double ticks)
{
    return ticks / TICKS_PER_ROTATION * (2 * PI);
}

double ChassisSubsystem::rpmToRadPerSecond(double RPM)
{
    return RPM * (2 * PI) / SECONDS_PER_MINUTE;
}

double ChassisSubsystem::radPerSecondToRPM(double radPerSecond)
{
    return radPerSecond / (2 * PI) * SECONDS_PER_MINUTE;
}