.PHONY: infantry-build infantry-flash

BOARD = nucleo_f446re

hero-build:
	west build -b $(BOARD) -p auto -d build/Hero robots/Hero

hero-build-clean:
	west build -b $(BOARD) -p always -d build/Hero robots/Hero

hero-flash: hero-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript Jlink/flashHero.jlink"

infantry-build:
	west build -b $(BOARD) -p auto -d build/Infantry robots/Infantry

infantry-build-clean:
	west build -b $(BOARD) -p always -d build/Infantry robots/Infantry
	infantry-build

infantry-flash: infantry-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript JLink/flashInfantry.jlink"

sentry-build:
	west build -b $(BOARD) -p auto -d build/Sentry robots/Sentry

sentry-build-clean:
	west build -b $(BOARD) -p always -d build/Sentry robots/Sentry

sentry-flash: Sentry-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript JLink/flashSentry.jlink"

testbench-build:
	west build -b $(BOARD) -p auto -d build/Testbench robots/Testbench

testbench-build-clean:
	west build -b $(BOARD) -p always -d build/Testbench robots/Testbench
	testbench-build

testbench-flash: testbench-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript JLink/flashTestbench.jlink"