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

From here on, you can return to step 3 of the main readme, **HOWEVER**, if you run into any issues with those steps, we have some potential fixes below. Note that this list is **not exhaustive**, so talk to your embed lead if you have any questions. 

## 3. Common Issues

At some point, likely when you try to build, you might see something about your toolchain not being set. If that's the case, try the following commands.

```
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
```

and then run the following to find the path to your zephyr sdk

```
find ~ -maxdepth 4 -type d -name "zephyr-sdk*" 2>/dev/null
```

you might see some pop ups about accessing your files. It's just a simple search so go ahead and allow. Once you've done that, add your specific path to this command and run it 

```
export ZEPHYR_SDK_INSTALL_DIR=/path/to/your/zephyr-sdk
```

From here, try building, and if it works, you'll have to add the two export commands to your .zshrc

This command opens it in vs code


```
code ~/.zshrc
```
This opens it in a generic text editor.

```
nano ~/.zshrc
```

Adding these exports here basically just makes it so that they always run whenever you open your terminal, so that you don't always have to manually set them. 