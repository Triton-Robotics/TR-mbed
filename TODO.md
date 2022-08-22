# EMBEDDED TO DO - 1. WRITE GOOD CODE

*Italics = New Recruit Work*

## HIGH PRIORITY

- ***MUTEX*** - Protect our threads.

- **Fix CAN lag** - (Realtime? change ref to idle and lower priorities for the rest)
  
  - test canhandler on its own, see if that still lags
  
  - `wait_us` INSTEAD OF `sleep_for`
  
  - Wire CAN in series?
    
    - Try only one motor, then one motor through center board, then two motors through center board, then two motors wired in series

- ***Fix Remote Control*** *for Nucleo without old-dev board*

- Improve the functionality to **Detect CAN Lag** *(MAYBE RECRUIT??)*

- *Accurate MULTITURN on M3508 (Maybe run it through a speed pid so its slow)*

- Make CANMotor class faster.

- ***Write Good Robot Code*** *Code for all robots that uses Speed PID and REF Code*

- **Add referee system into code**
  
  - *Recalibrate flywheels when ammo booster goes from off to on*
  
  - *Add heat/chassis power and somehow limit shots so that we dont go over the limits*
    
    - Multiple options:
    
    - Fast so heat goes up, then slow so it goes down again
    
    - operator control until heat gets to 80%, then do active heatlimiting to keep same heat
  
  - *add constants for heat/chassis power depending on level*
    
    get level
  
  - make referee system way way faster so it doesnt impede can

- *Tune PIDs*

## LOW PRIORITY

- *Write alternate debug scripts*

- *Write a Turret Subsystem Class*
  
  - *Hero serializer have accurate half rotation so you can control shooting (Theo's thing)*

- *Rewrite a Chassis Subsystem Class*

- Make a class for Referee System (Help Dexin turn it into a class)

- *Add class M3508, M2006, and GM6020 as extensions of CANMotor which reduces arguments by one (Get John or a new recruit some more experience with the CANMotor class)*

- *move CANBus enum to outside and change all robot code so we dont keep having to do NewCanHandler::CANBus_1*

- *Prevent shooting hero when the turret is in the bounds of the wire pole*

- In new CANHandler spaghetti, figure out how to use the built in attach functionality with feedback

- *Add switch to the motor auto-ratio definitions instead of if-else*

## NO PRIORITY

- ***Store Match Footage for CV***
