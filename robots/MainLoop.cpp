#include "MainLoop.h"

BNO055_ANGULAR_POSITION_typedef imuAnglesLocal;

static void imu_thread();
static void ref_thread();

// TODO maybe better to put these in header file and make it extern so robot specific mains have access to these?
unsigned long timeStart;
unsigned long timeStartCV;
unsigned long timeStartRef;
unsigned long timeStartImu;
// TODO this used to be = us_ticker_read() might cause problems
unsigned long loopTimer;
unsigned long loopTimerCV = loopTimer;
unsigned long loopTimerRef = loopTimer;
unsigned long loopTimerImu = loopTimer;
float elapsedms;

// TODO this is bad
int remoteTimer = 0;

void run_main_loop(TR::look_hooks &hooks)
{
    // init();
    hooks.init();
    init_jetson(TR::bcJetson);
    TR::imuThread.start(imu_thread);
    TR::refThread.start(ref_thread);

    while (true)
    {

        timeStartCV = us_ticker_read();

        // TODO this mutex should probably not be TR scoped, should be locally scoped?
        TR::mutex_test.lock();
        imuAnglesLocal.yaw = TR::imuAngles.yaw;
        imuAnglesLocal.pitch = TR::imuAngles.pitch;
        imuAnglesLocal.roll = TR::imuAngles.roll;
        TR::mutex_test.unlock();
        unsigned long mutex_ms = us_ticker_read();

        // jetson comms - 50-200us ish?
        if ((timeStartCV - loopTimerCV) / 1000 > TR::OUTER_LOOP_DT_MS)
        {
            loopTimerCV = timeStartCV;

            // TR::jetson_send_data.chassis_x_velocity = cs.vX;
            // TR::jetson_send_data.chassis_y_velocity = cs.vY;
            // TR::jetson_send_data.chassis_rotation = cs.vOmega;
            TR::jetson_send_data.chassis_x_velocity = TR::chassis_subsystem.getChassisSpeeds().vX;
            TR::jetson_send_data.chassis_y_velocity = TR::chassis_subsystem.getChassisSpeeds().vY;
            TR::jetson_send_data.chassis_rotation = TR::chassis_subsystem.getChassisSpeeds().vOmega;

            // TR::jetson_send_data.pitch_angle_rads = ChassisSubsystem::ticksToRadians((pitch_zero_offset_ticks - pitch.getData(ANGLE)));
            TR::jetson_send_data.pitch_angle_rads = TR::turret_subsystem.get_pitch_angle_rads_zero_offsetted();
            // TR::jetson_send_data.pitch_velocity = pitch.getData(VELOCITY) / 60.0;
            TR::jetson_send_data.pitch_velocity = TR::turret_subsystem.get_pitch_vel_rads_per_sec();

            TR::jetson_send_data.yaw_angle_rads = (imuAnglesLocal.yaw + 180.0) * (M_PI / 180.0);
            // TR::jetson_send_data.yaw_velocity = yaw.getData(VELOCITY) / 60.0;
            TR::jetson_send_data.yaw_velocity = TR::turret_subsystem.get_yaw_vel_rads_per_sec();

            TR::jetson_send_ref.game_state = game_status.game_progress;
            TR::jetson_send_ref.robot_hp = robot_status.current_HP;

            jetson_send_feedback(TR::bcJetson, TR::jetson_send_ref, TR::jetson_send_data);

            // TODO we should make a jetson subsystem so we are not dependent on global variable readresult
            // TR::readResult = jetson_read_values(TR::bcJetson, TR::jetson_received_data, TR::jetson_received_odom);
        }
        unsigned long cv_ms = us_ticker_read();\

        timeStart = us_ticker_read();

        if ((timeStart - loopTimer) / 1000 > TR::OUTER_LOOP_DT_MS)
        {
            elapsedms = (timeStart - loopTimer) / 1000;
            loopTimer = timeStart;
            TR::led = !TR::led;

            // Chassis updates - 50us
            TR::chassis_subsystem.periodic(&imuAnglesLocal);
            // cs = TR::chassis_subsystem.getChassisSpeeds();

            if (remoteTimer > 20)
            {
                remoteTimer = 0;
                // TODO FIX this
                // remoteRead();
                TR::remote.read();

                // TODO update states with remote values
                if (TR::remote.leftSwitch() == Remote::SwitchState::UP)
                    TR::shooter_subsystem.setState(ShootState::SHOOT);
                else if (TR::remote.leftSwitch() == Remote::SwitchState::MID)
                    TR::shooter_subsystem.setState(ShootState::FLYWHEEL);
                else
                    TR::shooter_subsystem.setState(ShootState::OFF);

                if (TR::remote.rightSwitch() == Remote::SwitchState::UP)
                {
                    TR::chassis_subsystem.setState(ChassisSubsystem::YAW_ORIENTED);

                    TR::turret_subsystem.setState(TurretState::AIM);
                }
                else if (TR::remote.rightSwitch() == Remote::SwitchState::DOWN)
                {
                    TR::chassis_subsystem.setState(ChassisSubsystem::BEYBLADE);

                    TR::turret_subsystem.setState(TurretState::AIM);
                }
                else
                {
                    TR::chassis_subsystem.setState(ChassisSubsystem::OFF);
                    TR::turret_subsystem.setState(TurretState::SLEEP);
                }
                // Set desired outside of state since state determines if this will run 
                TR::turret_subsystem.set_desired_turret(TR::remote.getYaw(), 
                                        TR::remote.getPitch(), 
                                        TR::chassis_subsystem.getChassisSpeeds().vOmega);

            }
            remoteTimer += 1;

            // Chassis Code - 100-150 us
            // hooks.chassis_executor();

            // // YAW + PITCH - 150us
            // hooks.turret_executor();
            // // dexer + flywheel - 100us
            // hooks.shooter_executor();
            hooks.periodic();

            hooks.print_rate_limited();

            // send CAN - 100us
            DJIMotor::s_sendValues();
        }
        // canread - 100-200us
        TR::canHandler1.readAllCan();
        TR::canHandler2.readAllCan();

        hooks.end_of_loop();
        // printff("tt%d\n", (us_ticker_read() - timeStartCV));

        // Sleep if our loop is shorter than 1ms to finish other executions
        if ((us_ticker_read() - timeStartCV) < 1000)
        {
            ThisThread::sleep_until(us_ticker_read() - timeStartCV);
        }
    }
}

static void imu_thread()
{
    while (1)
    {

        timeStartImu = us_ticker_read();

        if ((timeStartImu - loopTimerImu) / 1000 > 10)
        {
            loopTimerImu = timeStartImu;

#ifdef USE_IMU
            TR::mutex_test.lock();
            TR::imu.get_angular_position_quat(&TR::imuAngles);
            TR::mutex_test.unlock();
#else
            yaw_current_angle = (yaw >> ANGLE) * 360.0 / TICKS_REVOLUTION;
#endif
        }

        ThisThread::sleep_for(1ms);
    }
}

static void ref_thread()
{
    while (1)
    {
        timeStartRef = us_ticker_read();

        // referee loop every 15ms - seems like 6ms when dc and 600us when connected
        // TODO I dont think this works properly
        if ((timeStart - loopTimerRef) / 1000 > 5 * TR::OUTER_LOOP_DT_MS)
        {
            loopTimerRef = timeStart;
            TR::led2 = TR::referee.readable();
            refereeThread(&TR::referee);

            // POWER LIMIT OVERRIDE INCASE
            if (robot_status.chassis_power_limit < 10)
            {
                // TODO remove the dependency on this global variable
                // TR::chassis_power_limit = 49;
            }
            else
            {
                // TR::chassis_power_limit = robot_status.chassis_power_limit;
            }

            // TR::chassis_subsystem.power_limit = (float)TR::chassis_power_limit;
        }
        ThisThread::sleep_for(1ms);
    }
}