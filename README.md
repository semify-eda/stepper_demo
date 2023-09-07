The provided `main.cpp` source file is compatible with both the Arduino IDE and VSCode using PlatformIO. This script is tailored to operate on an STM32 Nucleo F4464RE Microcontroller. To replicate this setup, ensure that either the Arduino IDE or the PlatformIO IDE is properly configured.

For configuring the Arduino IDE, refer to: https://docs.simuli.co/getting-started/stm32/arduino-ide-and-virtual-lab/setting-up-arduino-ide-for-stm32

For the PlatformIO IDE, follow the steps outlined here: https://docs.platformio.org/en/latest/integration/ide/vscode.html#ide-vscode

If you are using PlatformIO, remember to include the `platformio.ini` configuration file to ensure correct settings and dependencies.

By following these guidelines, you'll be able to seamlessly run and experiment with the provided code on your STM32 Nucleo F4464RE Microcontroller.


I added a new source file, `smartwave.cpp`, to send control signals to the motor shield via I2C using SmartWave. This has been tested with the STM32 Nucleo F4464RE microcontroller. It's important to note that this code won't work with the Arduino UNO board since it requires multiple I2C channels, enabling the microcontroller to function both as a controller and as a peripheral.
