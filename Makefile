Infantry:
	cp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Infantry/TR-Infantry.bin /media/ben332004/NOD_F446RE/
	#scp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Infantry/TR-Infantry.bin pi@raspberrypi.local:/media/pi/NODE_F446RE/

Sentry:
	cp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR-Sentry.bin /media/ben332004/NODE_F446RE/
Hero:
	cp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Hero/TR-Hero.bin /media/ben332004/NODE_F446RE/
Engineer:
	cp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Engineer/TR-Engineer.bin /media/ben332004/NODE_F446RE/
Test-Bench:
	#cp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Test-Bench/TR-TestBench.bin /media/ben332004/NODE_F446RE/
	scp ./cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Test-Bench/TR-TestBench.bin pi@raspberrypi.local:/media/pi/NOD_F446RE/
test-bench:
	sudo chmod a+rw /dev/ttyACM0
	cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Test-Bench/TR-TestBench.bin /media/${USER}/NOD_F446RE
	mbed-tools sterm -b 115200
serial:
	mbed-tools sterm --baudrate 112500
