# Class Constructor:

```cpp
CanMotor(int canID, CANHandler::CANBus bus, motorType mType = STANDARD)
```

**canID** : The ID of the motor. Most, if not all motors will blink quickly, and counting the blinks will tell you what the ID of the motor is

**CANBus** : An enum of two possible can busses the [Waveshare Can transciever](https://www.amazon.com/SN65HVD230-CAN-Board-Communication-Development/dp/B00KM6XMXO/ref=sr_1_1?crid=PL0JKI6FA69A&keywords=waveshare+can+transceiver&qid=1649575254&sprefix=waveshare+can+transceiv%2Caps%2C323&sr=8-1) could be on: CANBUS_1 or CANBUS_2.

**motorType** : enum that determines what kind of motor is being represented by a specific CanMotor Object.

## Examples

Some sample motor creations below:  
`CanMotor gimbalXZ(5, CANHandler::CANBUS_1, GM6020);`

`CanMotor turretYaw(3, CANHandler::CANBUS_2, M3508);`

# Motor movement:

Using the PID for the motor class requires you to set up the PID values for the motor, separately for both Position and Speed, using these two functions:  

`void setPositionPID(double Kp, double Ki, double Kd)`

`void setSpeedPID(double Kp, double Ki, double Kd)`

You can cap the integral or cap the output of the PID with these functions:  

`void setPositionIntegralCap(double cap)`

`void setPositionOutputCap(double cap)`

and

`void setSpeedIntegralCap(double cap)`

`void setSpeedOutputCap(double cap)`

Once the PID is set up, you can just give it a speed or a position:

`void setPosition(int value)`

`void setSpeed(int value)`

or you can just give it a power value:

`void setPower(int value)`

Finally, the most important part of using the CanMotor class is running DJIMotor::tick(); to send all the motor values, get feedback values, and calculate multiTurn angle. 

However, if the threading is active, it will create a new thread for it, do not call it, or you will get a hard fault

## Examples

Example code with PID is:

```cpp
#include "main.hpp"
///////////////////////////////////////////////////
//IN main.hpp
static DJIRemote myremote(PA_0, PA_1);

CANHandler canHandler1(PA_11,PA_12);
CANHandler canHandler2(PB_12,PB_13);

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

Thread threadingRemote(osPriorityHigh);
///////////////////////////////////////////////////

CANHandler canPorts(PA_11,PA_12,PB_12,PB_13);

DJIMotor standard(1,CANHandler::CANBUS_1,STANDARD);
DJIMotor gimbly(7,CANHandler::CANBUS_1,GIMBLY);

int main(){
    threadingRemote.start(&remoteThread);
    DJIMotor::setCANHandlers(&canHandler1,&canHandler2);

    gimbly.setPositionPID(5, 0, 10);
    standard.setSpeedPID(0.5, 0, 2);

    gimbly.setPositionIntegralCap(4000);
    standard.setSpeedOutputCap(1000);

    int val = 8738;
    while(1){
        standard.setSpeed(2000);

        gimbly.setPosition(val);

        DJIMotor::tick();

        printf("Speed:%d\n",standard.getData(VELOCITY));
    }
}
```

---

# Requirements to making the CanMotor class work:

There are some things you need to make sure you have to make the motor class work:

## 1. Attach NewCANHandlers

To make the CanMotor class work, you need to attach two CANHandler objects to it, which stores two can busses that we use to make the motors run.

You do this with this function

`DJIMotor::setCANHandlers(&handler1, &handler2);`

Where each handler is a CANHandler object.

## 2. DJIMotor::tick()

You need to have a `DJIMotor::tick();` at the end of your loops, or on a scheduler to send all the motor values, as well as some other things.

IT IS VITAL THAT YOU CALL THIS AT THE END OF EACH LOOP

```cpp
DJIMotor::tick();
```

However, normally this is automatically done by the setCANHandlers() function, unless you give it the option not to begin the thread.

## **VERY IMPORTANT**

Keep in mind that while the GM6020 motors are technically capable to operate on canIDs 1-7, CAN restrictions do not allow them to operate on IDs other than 5-7. This means with two CAN busses, you have a limit of 6 GM6020s. Motors using the C620s do not have any limitations like this.

Keep in mind that the GM6020s operate on one higher of a can chunk than the M3508s and the M2006s. This means that its id is effectively increased by four. An M3508 on can ID 6 has the same configuration as a GM6020 on can ID 2. If the motor class detects conflicting motors of this nature, it will tell you, and both motors will be disabled.

Also keep in mind that the GM6020s can only operate on can IDs 1-7, so essentially each CAN Handler can handle 4 STANDARDs, 3 GM6020s, and then 4 more which can be assigned to either.

# Motor Feedback

When you run `DJIMotor::tick();`, the DJIMotor class reads a message from both CAN busses and updates a collection of feedback data collected from all collected motors. You can get that data with the following getter:

`int getData(motorDataType data)`

`data` can be:

`ANGLE` (0-8191)

`VELOCITY` (RPM)

`TORQUE` (?)

`TEMPERATURE` (Celsius)

`MULTITURNANGLE` (0-8191) , with no limit on how far back or forward.

It's important to keep in mind that `ANGLE` is only a relative angle, which resets when it goes over 8191, the range is always from 0 to 8191. It is more useful to use the `MULTITURN` functionality, as it handles angles larger than the bounds of a full rotation, so you could enter in 16282 and it would do two full rotations, or -24573 to go three rotations the other direction.
You can also reset the multiTurnAngle with zeroPos

`void zeroPos()`

# Debugging

There are some options to be had for debugging data, such as:

`static void printChunk(CANHandler::CANBus bus, short sendID)`

Which will print the set of four motors in that chunk.

`void printAllMotorData()`

Which will print all the motor data of a single motor
