# EMBEDDED TO DO - 1. WRITE GOOD CODE

*Italics = New Recruit Work*

## RECRUIT SHIT

* Look at photo on phone, fill out

* fix week 4 (URGENT)

## HIGH PRIORITY

- <mark>EMMA</mark> => ***MUTEX*** - Protect our threads.
  
  - Create a printf thread that doesn't interfere with anything else, lowest priority
  
  - Work with dexin to make ref thread

- <mark>ANSHAL</mark> => **LAG** - Figure out what is causing the massive 30-45 second pre-code delay
  
  - Iteratively test each include
  
  - try just making blinky with all the tr includes

- **Fix CAN lag** - (Realtime? change ref to idle and lower priorities for the rest)
  
  - test canhandler on its own, see if that still lags
  
  - `wait_us` INSTEAD OF `sleep_for`
  
  - Wire CAN in series?
    
    - Try only one motor, then one motor through center board, then two motors through center board, then two motors wired in series

- <mark>DEHAO</mark> => ***Fix Remote Control*** *for Nucleo without old-dev board*
  
  - Get any sort of reasonable data
  
  - Signal processing to eliminate bad data
    
    - look at the bits, see if theres a pattern that can be exploited with the bad data?
  
  - According to Theo according to CU, the data only fucks up when the vertical sticks are used, so test that

- <mark>GIO</mark> => ***IMU*** - Make it functional, make it efficient, minimal loss of position

- <mark>TANIA</mark> => **IMU on Old Devboard** - Make it functional, and incorporate its sending of data into the existing remote code

- **Add referee system into code**
  
  - <mark>STELLA</mark> => *Recalibrate flywheels when ammo booster goes from off to on*
  
  - <mark>ROSHAN</mark> => *Add heat and somehow limit shots so that we dont go over the limits*
    
    - Multiple options:
    
    - Fast so heat goes up, then slow so it goes down again
    
    - operator control until heat gets to 80%, then do active heatlimiting to keep same heat
  
  - <mark>JUSTIN</mark> => Chassis power limiting, don't lose health, requires ref system
  
  - *add constants for heat/chassis power depending on level*
    
    get level
  
  - make referee system way way faster so it doesnt impede can

- Improve the functionality to **Detect CAN Lag** *(MAYBE RECRUIT??)*

- <mark>GUAN</mark> => ***SD CARD OUTPUT*** - Log debug messages into an SD Card for viewing after match and for minimal delay, minimizing time lost is most important here.

- *Accurate MULTITURN on M3508 (Maybe run it through a speed pid so its slow)*

- Swap the current timing system (Kinda bootleg) with a callback set up with a timer. Ace says he knows a specific class that allows for this.

- Make CANMotor class faster.

- ***Write Good Robot Code*** *Code for all robots that uses Speed PID and REF Code*

- <mark>BEN</mark> => *Tune PIDs*

- Make sendValues more efficient by limiting what addresses it sends to based on what motors exist.

## LOW PRIORITY

- *Write alternate debug scripts*

- <mark>STELLA</mark> => *Write a Turret Subsystem Class*
  
  - *Hero serializer have accurate half rotation so you can control shooting (Theo's thing)*

- <mark>DEXIN</mark> => ~~Make a class for Referee System (Help Dexin turn it into a class)~~ Make Referee system usable and quick

- *Add class M3508, M2006, and GM6020 as extensions of CANMotor which reduces arguments by one (Get John or a new recruit some more experience with the CANMotor class)*

- *move CANBus enum to outside and change all robot code so we dont keep having to do NewCanHandler::CANBus_1*

- In new CANHandler spaghetti, figure out how to use the built in attach functionality with feedback

- *Add switch to the motor auto-ratio definitions instead of if-else*

- <mark>FUTURE BEN</mark> => Simple strafe-auto sentry to move around and beyblade

## NO PRIORITY

- ***Store Match Footage for CV***
