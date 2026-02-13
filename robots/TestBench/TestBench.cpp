// This is our controls playground for system identification
#include "base_robot/BaseRobot.h"
#include "util/algorithms/general_functions.h"

#include "util/communications/CANHandler.h"
#include "util/motor/DJIMotor.h"

#define IMPULSE_STRENGTH 8191

// Testing Parameters (change these to set the relevant motor)
// (CHECK THE CONFIG FOR NEW ROBOTS!!!!)
bool infantry = true, hero = false;
bool pitch = false, yaw = true;
bool ind = false;
bool flywheel = false;
bool chassis = false;
bool position = false, velocity = true;

// Fake PID stuff, unused
PID::config test_motor_vel_PID = {1, 0, 0};
PID::config test_motor_pos_PID = {1, 0, 0};

class TestBench : public BaseRobot {
  public:
    // Pre-init robot motors
    DJIMotor infPitch;
    DJIMotor infYaw;
    DJIMotor indexer;

    DJIMotor heroPitch;
    DJIMotor heroYaw;

    DJIMotor chassis1;
    DJIMotor chassis2;
    DJIMotor chassis3;
    DJIMotor chassis4;

    // Motors to set power to
    DJIMotor *testMot = nullptr;
    DJIMotor *testMot2 = nullptr;
    DJIMotor *testMot3 = nullptr;
    DJIMotor *testMot4 = nullptr;

    // Remote variables
    bool prevL = false;
    bool switL = false;
    bool switLDown = false;
    bool switR = false;
    bool switRDown = false;
    bool debug = false;

    // Debugging variables (plot these in Ozone / add more so u can plot in ozone)
    int motorSpeed = 0;
    int powerValue = 0;
    int ry = 0, desiredVelocity = 0;
    int stepAmplitude = IMPULSE_STRENGTH;
    int16_t powerBuffer = 0;
    int16_t velocityBuffer = 0, angleBuffer = 0, torqueBuffer = 0;
    bool impulse = true, measureStart = false;

    TestBench(Config &config)
        : BaseRobot(config),
          // clang-format off
        infPitch(DJIMotor::config{
            7,
            CANHandler::CANBUS_2,
            M3508,
            "inf pitch",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        infYaw(DJIMotor::config{
            4,
            CANHandler::CANBUS_1,
            M3508,
            "inf yaw",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        indexer(DJIMotor::config{
            2,
            CANHandler::CANBUS_2,
            M3508,
            "indexer",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        heroPitch(DJIMotor::config{
            5,
            CANHandler::CANBUS_2,
            M3508,
            "hero pitch",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        heroYaw(DJIMotor::config{
            1,
            CANHandler::CANBUS_1,
            M3508,
            "hero yaw",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        chassis1(DJIMotor::config{
            1,
            CANHandler::CANBUS_1,
            M3508,
            "inf pitch",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        chassis2(DJIMotor::config{
            2,
            CANHandler::CANBUS_1,
            M3508,
            "inf pitch",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        chassis3(DJIMotor::config{
            3,
            CANHandler::CANBUS_1,
            M3508,
            "inf pitch",
            test_motor_vel_PID,
            test_motor_pos_PID
        }),
        chassis4(DJIMotor::config{
            4,
            CANHandler::CANBUS_1,
            M3508,
            "inf pitch",
            test_motor_vel_PID,
            test_motor_pos_PID
        })
        // clang-format on        
    {}

    ~TestBench() {}

    void init() override 
    {
        // initialize ur test motor pointers
        if (infantry && pitch) {
            testMot = &infPitch;
        }
        else if (chassis) {
            testMot = &chassis1;
            testMot2 = &chassis2;
            testMot3 = &chassis3;
            testMot4 = &chassis4;
        }
        else if (infantry && yaw)
            testMot = &infYaw;
        else if (hero && pitch)
            testMot = &heroPitch;
        else if (hero && yaw)
            testMot = &heroYaw;
        else if (ind)
            testMot = &indexer;
    }
    
    void periodic(unsigned long dt_us) override {
        prevL = switL;
        switL = (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP);
        switLDown = (remote_.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN);
        switR = (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP);
        switRDown = (remote_.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN);

        if (switL) { // step response
            powerBuffer = stepAmplitude;
        }
        else if (switLDown) { // inverted step
            powerBuffer = -stepAmplitude;
        }
        else if (switR) {  // arbitrary response (CHANGE THIS FOR OTHER RESPONSE TYPES)
            ry = remote_.getChannel(Remote::Channel::RIGHT_VERTICAL);
            // powerBuffer = -IMPULSE_STRENGTH + ((float) ry+REMOTE_MAX)/(REMOTE_MAX*2) * (2*IMPULSE_STRENGTH);
            powerBuffer = IMPULSE_STRENGTH*(2/(1 + exp(.002*ry)) - 1);
        }
        else if (switRDown) { // constant power increase (Ramp Response)
            if (!measureStart) {
                measureStart = true;
                powerBuffer = 0;
            }
            else if (powerBuffer > IMPULSE_STRENGTH) {
                powerBuffer = 0;
            }
            else
                powerBuffer++;
        }
        else {
            measureStart = false;
            powerBuffer = 0;
        }

        testMot->setPower(powerBuffer);
        testMot2 ? testMot2->setPower(powerBuffer) : (void)0; // for chassis motor
        testMot3 ? testMot3->setPower(powerBuffer) : (void)0; // for chassis motor
        testMot4 ? testMot4->setPower(powerBuffer) : (void)0; // for chassis motor
        angleBuffer = (*testMot)>>ANGLE;
        velocityBuffer = (*testMot)>>VELOCITY;
        torqueBuffer = (*testMot)>>TORQUE;

        if (switL || switR || switLDown || switRDown) { // print only when test is active
            if (position) {
                printf("%d\t%d\n", powerBuffer, angleBuffer);
            } else if (velocity) {
                printf("%d\t%d\n", powerBuffer, velocityBuffer);
            }
        }
    }

    void end_of_loop() override {}

    unsigned int main_loop_dt_ms() override { return 2; } // 500 Hz loop
};


int main() {
    printf("HELLO\n");
    BaseRobot::Config config = BaseRobot::Config{}; 
    TestBench TestBench(config);

    TestBench.main_loop();
    // blocking
}