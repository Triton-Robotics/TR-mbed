# EMBEDDED TO DO

- Write mecanum code - WRITTEN BUT UNTESTED

- Can2 and CANHandler class use in motor.hpp masterclass - WRITTEN BUT UNTESTED

- Grab more can wires so we can test multiple motors at once - SUNDAY 

- Add support for can2 by making a can handler

- Make Chassis Subsystem

- Finish Cloning TR Repo section on documentation

- Make a class for Referee System 

- Look into MBED-os scheduler

# MBED STUDIO Setup

## Installation

- Click this link and install [Mbed STudio](https://os.mbed.com/studio/)

- Install Github Desktop [Github Desktop](https://desktop.github.com/)

- Open Github Desktop and clone this repo to any location using this link: https://github.com/ARMmbed/mbed-os.git

## Blinky Demo

Test upload code to verifiy correct installation to the STM32 Nucleo F446RE

1) Open MBED Studio

2) Go to file -> New Program

3) A window like something below will pop up
   
   1) Change example program to be an Empty MBED OS 6 Program   
   
   2) Program name: Blinky
   
   3) Click on "Link to an existing shared MBED OS instance". Select the MBED OS folder that you cloned 
   
   4) Click Add program
   
   5) These are the steps you should take everytime you create a new, local program.
   
   ![](assets/2022-04-09-16-37-36-image.png)

4) Open main.cpp under the Blinky program folder you just created![](assets/2022-04-09-16-42-55-image.png)

5) Type this code into main.cpp![](assets/2022-04-09-16-43-56-image.png)

6) Plug in to the Nucleo. Make sure Blinky is set to Active Program and that when you plugged into the Nucleo there is a little green USB Icon next to the name.![](assets/2022-04-09-16-46-26-image.png)

7) Hit the play button to compile and uplode to the board. It will take some time as this is the first time you are compiling the entire project

8) If you did everything right, the built-in LED should start blinking on the Nucleo!

## Cloning Triton Robotics Github Repo

- Clone the [TR Github](https://github.com/Triton-Robotics/TR-mbed6) with Github Desktop

- Put it in the "User/Mbed Programs" location



## Installion a Junction

You need to create a **JUNCTION** in the TR-mbed6 folder to point to the mbed-os library.

1) Install the [Link Shell Extension](https://download.cnet.com/Link-Shell-Extension-64-bit/3000-2248_4-75213087.html)

2) Right click on the mbed-os folder that you cloned at the very beginning

3) Pick Link Source on the mbed-os folder 

4) Go inside TR-mbed6 and right click, select **Drop As Junction** 