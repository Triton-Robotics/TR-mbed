# EMBEDDED TO DO

- Write a Turret Subsystem Class - WRITTEN BUT UNTESTED

- Fix and set up pids again. . .

- In new CANHandler spaghetti, set up a system to tell if new messages are not coming instead

- In new CANMotor Spaghetti, set up a system to take the fact that its frozen and not do anything new

- In new CANHandler spaghetti, figure out how to use the built in attach functionality with feedback

- Remove ratio option for motor constructor, if its an external gearbox have the end-user deal with it on their own.

- Add switch to the motor auto-ratio definitions instead of if-else

- Make a class for Referee System 

- Add class M3508, M2006, and GM6020 as extensions of CANMotor which reduces arguments by one

- move CANBus enum to outside so we dont keep having to do CanHandler::CANBus_1
