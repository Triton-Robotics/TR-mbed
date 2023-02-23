test_bench:
	${MAKE} test-bench
test-bench:
	cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Test-Bench/TR-TestBench.bin /media/${USER}/NODE_F446RE

hero:
	cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Hero/TR-Hero.bin /media/${USER}/NOD_F446RE

infantry:
	cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Infantry/TR-Infantry.bin /media/${USER}/NOD_F446RE

sentry:
	cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Sentry/TR-Sentry.bin /media/${USER}/NOD_F446RE

engineer:
	cp cmake_build/NUCLEO_F446RE/develop/GCC_ARM/robots/Engineer/TR-Engineer.bin /media/${USER}/NOD_F446RE

serial:
	mbed-tools sterm -b 115200

serial9600:
	mbed-tools sterm