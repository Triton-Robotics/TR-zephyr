.PHONY: infantry-build infantry-flash

BOARD = nucleo_f446re

infantry-build:
	west build -b $(BOARD) -p auto -d build/Infantry robots/Infantry

infantry-build-clean:
	west build -b $(BOARD) -p always -d build/Infantry robots/Infantry

infantry-flash: infantry-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript JLink/flashInfantry.jlink"

testbench-build:
	west build -b $(BOARD) -p auto -d build/Testbench robots/Testbench

testbench-build-clean:
	west build -b $(BOARD) -p always -d build/Testbench robots/Testbench

testbench-flash: testbench-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript JLink/flashTestbench.jlink"