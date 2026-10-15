## Pre-req for windows: WSL

If you're on linux, feel free to skip this section; however, if you're on mac, here's your setup [guide](.md/macSetUp.md) if you're on windows, you need to set up Windows Subsystem for linux, (WSL). This can be done through the following command in powershell (more specifically, this installs ubuntu)

```
wsl --install
```

Once you have that done, go to "Turn Windows Features on or off" in your settings. Scroll down to Windows Subsystem for linux, and make sure it's enabled. Once that's done, you'll be prompted to restart your computer. 

Once that's done, open vscode, and in the bottom left there should be a >< symbol with the alt text "Open a remote window" click on that, and you are ready to proceed. 

## Getting Started:
1. Clone this repository 
For convenience make sure that the clone is "TR-zephyr", otherwise you'll have to alter the directory in a few of the set up commands.

2. Download the Zephyr SDK
```
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/sha256.sum | shasum --check --ignore-missing
tar xvf zephyr-sdk-0.16.8_linux-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh
```

3. Create and activate venv
```
cd ~/TR-zephyr
python3 -m venv .venv
source .venv/bin/activate
```

4. Install west in the activated venv
```
pip install west
```

5. Initialize the Zephyr RTOS
```
west init -l manifest-repo
west update
```

6. Install requirements
```
pip install -r zephyr/scripts/requirements.txt
```

## Building and Flashing:

For Jetson-only autonomy on Infantry hardware, see the [Sentry firmware guide](robots/Sentry/README.md).

Go into the makefiles folder, and copy your OS-specific makefile, paste it outside of the folder into the root, and rename it to `Makefile` (it's case sensitive). If there's already a makefile there, just replace it to be safe, but it should be a WSL makefile by default. 


Download the [JLink](https://www.segger.com/downloads/jlink/) software on your device (J-Link Software and Documentation pack and Ozone - The J-Link Debugger).

<!-- *This also only works for WSL* -->

To build for the first time:
```
make infantry-build-clean
```

**Note:** All build commands have the format: [robot]-build. The -clean is optional and basically makes a "pristine" build that clears your local cache'd build and rebuilds from scratch, which takes longer but can be very helpful if you're working with lower-level code; however, it's not strictly necessarily and most of the time just using your cache'd files is fine, but it can be a potential source of error. 

To flash:
```
make infantry-flash
```

## Intellisense and debugging

This section is technically optional but **highly recommended**.

### 1. **Get clangd** (VS code extension and run sudo apt install clangd)
Intellisense for everything but devicetree stuff

Once you have clangd, you should run the following commands to link each robot's compile commands json to each robot folder

```
cd ~/TR-zephyr

make infantry-build-clean
make hero-build-clean
make sentry-build-clean
make testbench-build-clean
```

```
cd robots/Infantry
ln -s ../../build/Infantry/compile_commands.json compile_commands.json

cd ../Sentry
ln -s ../../build/Sentry/compile_commands.json compile_commands.json

cd ../Hero
ln -s ../../build/Hero/compile_commands.json compile_commands.json

cd ../Testbench
ln -s ../../build/Testbench/compile_commands.json compile_commands.json

cd ../../core
ln -s ../build/Infantry/compile_commands.json compile_commands.json
```

From this point, you might see a few erroneous clangd errors, so run the following to get rid of them

```
cd ~/TR-zephyr

cat > .clangd << 'EOF'
CompileFlags:
  Remove:
    - -mfp16-format=ieee
    - -mtp=soft
    - -specs=picolibc.specs
    - -fno-reorder-functions
EOF
```

Also, in your .vscode file, in settings.json, append the following to the end, (don't forget to comma before this). Make sure you put in your path to your zephyr-sdk.

```
"clangd.arguments": [
        "--query-driver=[YOUR-PATH HERE]/zephyr-sdk-0.16.8/arm-zephyr-eabi/bin/arm-zephyr-eabi-*"
    ]
```

### 2. Set up Ozone 
Read the guide [here](.md/OzoneSetUp.md)

### 3. Get nRF DeviceTree extension on vscode (from nordic semiconductor)
Intellisense for device tree. Very helpful; however, can be an annoying set up. Talk to your embed lead if you run into any issues. 

## Jetson UART telemetry timing

A telemetry group is 40 bytes: referee (6), robot state (30), and user input
(4). At 115200 baud, 8N1, the link carries at most 11,520 bytes/s; one group
takes about 3.47 ms. The previous 2 ms enqueue interval exceeded that capacity.
Telemetry now takes one coherent latest-state snapshot per group and waits
10 ms between groups, allowing the transmitter to empty between updates.

This addresses measured Jetson receive bursts of 4096 bytes roughly every
395 ms. A stationary camera test produced 879 detections but only 61 aiming
solutions in 20 seconds because embedded feedback was frequently too old.
Verify receive intervals again after flashing; host tests cannot establish
actual UART timing.

The UART idle interrupt now disables TX without acquiring a mutex, keeps RX
active, and permits subsequent writes to restart TX. The existing Sentry
500 ms command watchdog, independent packet timestamps, motor configuration,
and control conventions are preserved. On this branch, the watchdog explicitly
puts the turret into SLEEP when aiming commands expire; telemetry improvements
do not change that behavior.

Run both host regressions before a firmware build:

```bash
python3 tests/test_jetson_transport_host.py
python3 tests/sentry_autonomy/run.py
```

The transport regression compiles the actual Jetson/SerialBase sources with a
simulated Zephyr UART. It checks telemetry bytes and pacing, idle interrupt lock
avoidance, TX restart after interrupt preemption, continued RX, and rejection of
invalid command timestamps. The existing Sentry suite covers control behavior.
These tests do not replace a Zephyr build and hardware validation.
