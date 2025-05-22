# CC1200 Radio Driver

![CC1200 Dev Kit](https://www.ti.com/content/dam/ticom/images/products/ic/processors/evm-boards/cc1200emk-868-930-angled.png:large)

This driver provides implementations for both Mbed OS and STM32 HAL to control the CC1200 (and CC1201) radio ICs. Most features of the IC are implemented, and you aren't limited to premade configurations! This code implements the calculations needed to configure each part of the chip from raw frequency and data rate inputs.

Originally written by Jamie Smith @ USC Rocket Propulsion Lab.

Migrated from the original Mbed repository [here](https://os.mbed.com/users/MultipleMonomials/code/CC1200/shortlog/).

## Directory Structure

This repository is organized as follows:

- `mbed/` - Original Mbed OS implementation
- `stm32/` - STM32 HAL implementation for STM32F411CEU
- `common/` - Shared files between implementations
- `examples/` - Example applications for both implementations
  - `examples/morse/` - Morse code transmission example using the CC1200
- `mock/` - Mock implementations for testing without hardware
  - `mock/stm32/` - Mock STM32 HAL implementation
  - `mock/mbed/` - Mock Mbed OS implementation

## Implementation Details

### Mbed OS Implementation
The original implementation uses Mbed OS for hardware abstraction and is suitable for any Mbed-compatible platform.

### STM32 HAL Implementation
A new implementation using STM32 HAL has been added, specifically targeting the STM32F411CEU microcontroller. This implementation maintains all the functionality of the original driver while using STM32 HAL for hardware interfacing.

### Mock Implementations
Mock implementations for both STM32 HAL and Mbed OS have been added to allow testing without actual hardware. These mock implementations simulate the behavior of the hardware interfaces, making it possible to develop and test the driver on a computer.

## Installation

### STM32 HAL Implementation

1. Copy the following files to your project:
   - `stm32/CC1200_HAL.h`
   - `stm32/CC1200_HAL.cpp`
   - `common/CC1200Bits.h`

2. Make sure your project includes the STM32 HAL libraries for your target microcontroller

3. Configure your build system to include the driver files

### Mbed OS Implementation

1. Copy the following files to your project:
   - `mbed/CC1200.h`
   - `mbed/CC1200.cpp`
   - `common/CC1200Bits.h`

2. Make sure your project includes the Mbed OS libraries

3. Configure your build system to include the driver files

### Mock Testing

1. Copy the entire `mock` directory to your project

2. Use the provided CMakeLists.txt in the `mock` directory to build the mock test application

3. Run the mock test application to verify the driver functionality without hardware

```bash
# Build the mock test application
mkdir build
cd build
cmake ../mock
make

# Run the STM32 HAL mock test
./cc1200_stm32_test
```

## About the CC1200
The [CC1200](https://www.ti.com/product/CC1200) is a digital radio transceiver supporting a large variety of different bands, modulations, and packet formats. It lets you use a single part (with different circuitry) to transmit on a couple different bands, including the 430MHz European ISM band and the 900MHz American ISM band. It can be configured for a variety of different packet formats, including OOK, ASK, and a number of variants of FSK. Data rates can be as high as 1Mbps (using 4-FSK at 500kbps), or as low as a few hundred bytes per second depending on your bandwidth needs and distance requirements. All in all, this is an extremely capable radio chip can be adapted for almost any 100-900MHz digital radio application.

## Features

- Automatic calculation of correct register values for:
  - RF frequency
  - FSK deviation
  - Symbol rate
  - Output power
  - RX filter bandwidth (this one's harder than it looks!)
- Easy handling of data packets
- GPIO configuration
- Preamble and sync word configuration
- RTOS compatible (always locks SPI bus during transactions)
- Two debug levels available
- RSSI and LQI support

### Not Supported:

- Transparent mode
- FM mode
- ASK parameter configuration

## Usage Examples

### STM32 HAL Implementation

```cpp
// Initialize SPI and GPIO
SPI_HandleTypeDef hspi1;
// Configure SPI in your initialization code

// Create CC1200 instance
CC1200 radio(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0);

// Initialize the radio
if (radio.begin()) {
    // Configure radio parameters
    radio.setRadioFrequency(CC1200::Band::BAND_820_960MHz, 915000000);
    radio.setSymbolRate(50000);
    radio.setRXFilterBandwidth(104000, false);
    radio.setFSKDeviation(50000);
    radio.setOutputPower(14.0f);
    radio.setPacketMode(CC1200::PacketMode::FIXED_LENGTH, false);
    radio.setPacketLength(16, 8);
    
    // Transmit data
    const char* data = "Hello, CC1200!";
    radio.enqueuePacket(data, strlen(data));
    radio.sendCommand(CC1200::Command::TX);
    
    // Switch to receive mode
    radio.sendCommand(CC1200::Command::RX);
    
    // Receive data
    char buffer[64];
    if (radio.hasReceivedPacket()) {
        size_t len = radio.receivePacket(buffer, sizeof(buffer));
        // Process received data
    }
}
```

### Mbed OS Implementation

```cpp
// Create SPI and GPIO instances
SPI spi(PA_7, PA_6, PA_5, PA_4, use_gpio_ssel);
DigitalOut rst(PB_0, 1);

// Configure SPI
spi.format(8, 0);  // 8 bits, mode 0
spi.frequency(5000000);  // 5 MHz

// Create CC1200 instance
CC1200 radio(spi, rst);

// Initialize and use the radio as shown in the STM32 example
```

### Mock Testing

The mock test application in `examples/mock_test.cpp` demonstrates how to use the CC1200 driver with mock implementations for testing without hardware. It includes examples for both STM32 HAL and Mbed OS implementations.

## Recent Changes

### Version 2.0 (May 2025)

- Added STM32 HAL implementation for STM32F411CEU microcontroller
- Created mock implementations for both STM32 HAL and Mbed OS for testing without hardware
- Reorganized directory structure to separate Mbed OS and STM32 HAL implementations
- Added common directory for shared files
- Created examples directory with sample applications
- Fixed various bugs and improved error handling
- Added comprehensive documentation
- Frequency offsets


## Examples
See the [cc1200-demo](https://github.com/mbed-ce/cc1200-demo) project for examples of how to use the driver.

## Note: Radio Settings
For configuring radio settings, TI provides a number of configurations for you in their SmartRF application. The MBed OS driver lets you use these, but you can also enter your own settings if you need something different than what SmartRF provides. I will say, in my experience, the CC1200 does tend to be a bit of a house of cards - changing even one value to be incorrect (out of the 10-15 values that you need to configure) can easily cause the chip to stop functioning entirely. So, I recommend you stick to the provided configurations if possible, and only change things if you know what you're doing and are sure that you need a different value.

## Changelog

### Version 2.0 Jan 14 2023
- Migrate to CMake build system
- Switch the debug stream to be a FILE * instead of the deprecated Stream class
- Bring in CC1200Morse project
- Switch time unit in CC1200Morse to std::chrono

### Version 1.2 May 3 2021

- Added unfinished infinite length packet support via the readStream() and writeStream() functions. The API is complete and basic usage works but there's still a bug I haven't been able to track down yet where incorrect data is transmitted at the end of a stream. Use with caution!
- Added preferHigherCICDec parameter to setRXFilterBandwidth
- Removed setIFMixCFG() (which takes a byte parameter) and replaced it with setIFCfg(), which takes documented enum class values.
- Added setAGCSettleWait(), which per my testing is needed for correct 430MHz operation.
- Added support for reading RSSI and LQI values, both from packet appended status bytes and from the registers.
- Update 430MHz black box registers based on SmartRF values
- Removed setIQMismatchCompensationEnabled(). This call has been replaced by the new 2nd parameter to setIFCfg().

### Version 1.1 Aug 28 2020

- Add fixed length packet support and other features needed for Morse support.
- Fix bug causing weird behavior with low sample rates (<1ksps).

NOTE: you must now call setPacketMode() when configuring the radio.

### Version 1.0 Aug 10 2020

Initial Release