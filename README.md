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
Download the [JLink](https://www.segger.com/downloads/jlink/) software on your device (J-Link Software and Documentation pack and Ozone - The J-Link Debugger).

<!-- *This also only works for WSL* -->

To build:
```
make infantry-build
```

(Be aware that sometimes you may have to run a build twice for it to compile, see the note at the end for details)

To flash:
```
make infantry-flash
```

Lastly, if you need to wipe your current build and make a clean one, you can append -clean to build. For example

```
make infantry-build-clean
```

However, you should know that this won't ever compile due to a race condition in Zephyr itself (related to offsets.h). As a result, after infantry-build-clean, just run infantry-build, and you should be good. If that doesn't compile, then the error's on your end. 

## Intellisense

This section is technically optional but **highly recommended**.

1. **Get clangd** (VS code extension and run sudo apt install clangd)
Intellisense for everything but devicetree stuff

2. Get nRF DeviceTree extension on vscode (from nordic semiconductor)
Intellisense for device tree. Very helpful; however, can be an annoying set up. Talk to your embed lead if you run into any issues. 

