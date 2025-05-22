/*
 * Mock implementation of Mbed OS for testing purposes
 * This file provides mock definitions for Mbed OS functions and types
 * to allow compiling and testing the CC1200 driver on a computer.
 */

#ifndef MBED_MOCK_H
#define MBED_MOCK_H

#include <stdint.h>
#include <stdio.h>
#include <chrono>

namespace std {
    using namespace chrono;
}

namespace mbed {

// Forward declarations
class DigitalOut;
class SPI;

// PinName type for pin definitions
typedef int PinName;

// Define some mock pin names
constexpr PinName NC = -1;
constexpr PinName PA_0 = 0;
constexpr PinName PA_1 = 1;
constexpr PinName PA_2 = 2;
constexpr PinName PA_3 = 3;
constexpr PinName PA_4 = 4;
constexpr PinName PA_5 = 5;
constexpr PinName PA_6 = 6;
constexpr PinName PA_7 = 7;
constexpr PinName PB_0 = 16;
constexpr PinName PB_1 = 17;
constexpr PinName PB_2 = 18;
constexpr PinName PB_3 = 19;
constexpr PinName PB_4 = 20;
constexpr PinName PB_5 = 21;
constexpr PinName PB_6 = 22;
constexpr PinName PB_7 = 23;

// SPI mode for GPIO SSEL
constexpr int use_gpio_ssel = 1;

// Digital output class
class DigitalOut {
public:
    DigitalOut(PinName pin, int value = 0);
    ~DigitalOut();
    
    void write(int value);
    int read();
    
    DigitalOut& operator= (int value);
    DigitalOut& operator= (DigitalOut& rhs);
    operator int();
    
private:
    PinName pin_;
    int value_;
};

// SPI class
class SPI {
public:
    SPI(PinName mosi, PinName miso, PinName sclk, PinName ssel = NC, int mode = 0);
    ~SPI();
    
    void format(int bits, int mode = 0);
    void frequency(int hz);
    
    virtual int write(int value);
    
    void select();
    void deselect();
    
private:
    PinName mosi_;
    PinName miso_;
    PinName sclk_;
    PinName ssel_;
    int mode_;
    int bits_;
    int frequency_;
    uint8_t last_tx_byte_;
    uint8_t next_rx_byte_;
};

// Timer class
class Timer {
public:
    Timer();
    ~Timer();
    
    void start();
    void stop();
    void reset();
    
    std::chrono::microseconds elapsed_time() const;
    
private:
    std::chrono::steady_clock::time_point start_time_;
    bool running_;
};

// Wait functions
void wait_us(int us);

} // namespace mbed

// Enable debug output for mock functions
void mock_mbed_set_debug(int enable);

// Set the next byte to be received in SPI transactions
void mock_mbed_spi_set_rx_byte(mbed::SPI* spi, uint8_t byte);

// Get the last byte transmitted in SPI transactions
uint8_t mock_mbed_spi_get_last_tx_byte(mbed::SPI* spi);

using namespace mbed;

#endif /* MBED_MOCK_H */
