# Setup - Week 0

### Logistics

As an embedded member, we will have weekly meetings in person starting Fall. 

We will be scheduling multiple meetings, in the past We've had 3, and you are expected to make at minimum one, but optimally two each week. I do not cap the meetings on time.

For online, we'll have one midweek a week, online, for me to go over the material 

### Prerequisites

We develop our code on linux, and the setup is consistent if you are using linux. 

- The recommended option is dual booting, a tutorial to do so can be found here:
  [How to Dual Boot Any Linux Distribution With Windows – and Get Rid of It When You Need To](https://www.freecodecamp.org/news/how-to-dual-boot-any-linux-distribution-with-windows/)

- It is also possible to complete the following steps on Windows or Mac, however they are less documented and are more likely to sporadically stop working. 

If you'd like to test on an actual board, you can buy one for like 15 bucks [here]([NUCLEO-F446RE STMicroelectronics | Mouser](https://www.mouser.com/ProductDetail/STMicroelectronics/NUCLEO-F446RE?qs=PRtH0mD6DWYnuBoPSlbRCA%3D%3D)), but if not we have a website where you can run example code in the [ARM mbed Simulator](http://195.130.59.221/). One thing to keep in mind with the simulator is that this is an older version of mbed, so while most things are the same, sleep commands and other timing, threading, interrupt related things may be different in syntax when you transition to a real board in fall.

### Setup steps

Steps for setup can be found at [README.md](../README.md)

### Testing your setup

The most you can do without a physical board to test your setup is to build a blinky and see if it compiles. All the rest of our testing, if you don't have a board, can be done on the online simulator, but if you have a board, it is highly recommended you use that.

# Assignment

All that is to turn in for Week 0 is to send a bin or elf file from your first build of the code.  Build a basic blinky or anything you want and submit the bin or elf file. If you have trouble with setup, compilation, or anything else, contact me on discord, I'm @anshalaj.
