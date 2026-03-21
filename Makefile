.PHONY: infantry-build infantry-flash

BOARD = nucleo_f446re

infantry-build:
	west build -b $(BOARD) -p auto -d build/Infantry robots/Infantry

infantry-flash: infantry-build
	powershell.exe -Command "jlink.exe -device STM32F446RE -if SWD -speed 4000 -CommanderScript JLink/flashInfantry.jlink"