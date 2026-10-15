#!/usr/bin/env python3
"""Compile and run production Jetson/Sentry code with host hardware substitutes."""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

here = Path(__file__).resolve().parent
root = here.parents[1]
headers = [
    "zephyr/kernel.h", "zephyr/device.h", "zephyr/devicetree.h",
    "zephyr/drivers/i2c.h", "zephyr/drivers/pwm.h", "zephyr/drivers/gpio.h",
    "zephyr/dt-bindings/pwm/pwm.h", "util/algorithms/mbedMutex.cpp",
    "util/communications/mbedSerial.h", "util/algorithms/PID.h",
    "util/communications/CANHandler.h", "util/communications/DJIRemote2.h",
    "util/peripherals/imu/ISM330.h", "base_robot/BaseRobot.h",
]
with tempfile.TemporaryDirectory(prefix="sentry-autonomy-") as directory:
    temp = Path(directory)
    for name in headers:
        header = temp / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text('#include "platform.h"\n')
    binary = temp / "sentry_autonomy"
    subprocess.run([
        *shlex.split(os.environ.get("CXX", "g++")),
        "-std=c++17", "-Wall", "-Wextra", "-fsanitize=address,undefined", "-g",
        "-I", str(temp), "-I", str(here), "-I", str(root / "core"),
        str(here / "test.cpp"),
        str(root / "core/util/communications/jetson/Jetson.cpp"),
        str(root / "core/util/algorithms/general_functions.cpp"),
        "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)
