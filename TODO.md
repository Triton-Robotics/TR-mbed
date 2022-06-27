# EMBEDDED TO DO - 1. WRITE GOOD CODE

*Italics - New Recruit Work*

- *Write a Turret Subsystem Class*

- *Rewrite a Chassis Subsystem Class*

- In new CANHandler spaghetti, ~~set up~~ **FIX** system to tell if new messages are not coming instead

- ~~In new CANMotor Spaghetti, set up a system to take the fact that its frozen and not do anything new~~ Maybe not

- In new CANHandler spaghetti, figure out how to use the built in attach functionality with feedback

- *Add switch to the motor auto-ratio definitions instead of if-else*

- Make a class for Referee System (Help Dexin turn it into a class)

- *Add class M3508, M2006, and GM6020 as extensions of CANMotor which reduces arguments by one (Get John or a new recruit some more experience with the CANMotor class)*

- *move CANBus enum to outside and change all robot code so we dont keep having to do NewCanHandler::CANBus_1*

- Make CANMotor class faster.

- Add referee system into code
  
  - *Recalibrate flywheels when ammo booster goes from off to on*
  
  - Add heat/chassis power and somehow limit shots so that we dont go over the limits
  
  - *add constants for heat/chassis power depending on level*
    
    - get level
  
  - make referee system way way faster so it doesnt impede can

- **Fix CAN lag** - (Realtime? change ref to idle and lower priorities for the rest)
  
  * test canhandler on its own, see if that still lags

- *Write good code PID code for all robots*

- *Tune PIDs*

- *Write alternate debug scripts*

- MUTEX - Protect our threads.
