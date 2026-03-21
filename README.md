## Getting Started:

1. Clone this repository 

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
*This requires Jlink to already be set up on your computer*

*This also only works for WSL*

To build:
```
make infantry-build
```

To flash:
```
make infantry-flash
```