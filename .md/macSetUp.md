Since mac doesn't really have a WSL equivalent, you'll just have to do the mac equivalent of a few commands

## 1. Clone this repo
For convenience make sure that the clone is "TR-zephyr", otherwise you'll have to alter the directory in a few of the set up commands.

## 2. Download the Zephyr SDK

If you don't have homebrew, install it 

```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
``` 
Then install wget with 

```
brew install wget
```

Now to actually install the SDK.

If you're on an M-series chip (M1, M2, M3 etc.) use the following command

```
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_macos-aarch64.tar.xz
wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/sha256.sum | shasum --check --ignore-missing
tar xvf zephyr-sdk-0.16.8_macos-aarch64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh
```

If you're on an intel chip, use the following command 

```
cd ~
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_macos-x86_64.tar.xz
wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/sha256.sum | shasum --check --ignore-missing
tar xvf zephyr-sdk-0.16.8_macos-x86_64.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh
```

Now you can return to step 3 of the main readme 