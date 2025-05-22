/*
 * Mock implementation of Mbed OS for testing purposes
 * This file provides mock implementations of Mbed OS functions
 * to allow compiling and testing the CC1200 driver on a computer.
 */

#include "mbed.h"
#include <map>
#include <thread>
#include <chrono>

using namespace mbed;

// Debug flag
static int debug_enabled = 0;

// Map to store SPI instances
static std::map<SPI*, uint8_t> spi_last_tx_bytes;
static std::map<SPI*, uint8_t> spi_next_rx_bytes;

// Enable debug output for mock functions
void mock_mbed_set_debug(int enable) {
    debug_enabled = enable;
}

// Set the next byte to be received in SPI transactions
void mock_mbed_spi_set_rx_byte(SPI* spi, uint8_t byte) {
    spi_next_rx_bytes[spi] = byte;
}

// Get the last byte transmitted in SPI transactions
uint8_t mock_mbed_spi_get_last_tx_byte(SPI* spi) {
    return spi_last_tx_bytes[spi];
}

// Implementation of DigitalOut
DigitalOut::DigitalOut(PinName pin, int value) : pin_(pin), value_(value) {
    if (debug_enabled) {
        printf("DigitalOut constructor: pin=%d, value=%d\n", pin, value);
    }
}

DigitalOut::~DigitalOut() {
    if (debug_enabled) {
        printf("DigitalOut destructor: pin=%d\n", pin_);
    }
}

void DigitalOut::write(int value) {
    value_ = value ? 1 : 0;
    if (debug_enabled) {
        printf("DigitalOut::write: pin=%d, value=%d\n", pin_, value_);
    }
}

int DigitalOut::read() {
    if (debug_enabled) {
        printf("DigitalOut::read: pin=%d, value=%d\n", pin_, value_);
    }
    return value_;
}

DigitalOut& DigitalOut::operator= (int value) {
    write(value);
    return *this;
}

DigitalOut& DigitalOut::operator= (DigitalOut& rhs) {
    write(rhs.read());
    return *this;
}

DigitalOut::operator int() {
    return read();
}

// Implementation of SPI
SPI::SPI(PinName mosi, PinName miso, PinName sclk, PinName ssel, int mode) :
    mosi_(mosi), miso_(miso), sclk_(sclk), ssel_(ssel), mode_(mode),
    bits_(8), frequency_(1000000), last_tx_byte_(0), next_rx_byte_(0xFF) {
    
    if (debug_enabled) {
        printf("SPI constructor: mosi=%d, miso=%d, sclk=%d, ssel=%d, mode=%d\n", 
               mosi, miso, sclk, ssel, mode);
    }
    
    // Initialize the SPI maps
    spi_last_tx_bytes[this] = 0;
    spi_next_rx_bytes[this] = 0xFF;
}

SPI::~SPI() {
    if (debug_enabled) {
        printf("SPI destructor\n");
    }
    
    // Remove from the SPI maps
    spi_last_tx_bytes.erase(this);
    spi_next_rx_bytes.erase(this);
}

void SPI::format(int bits, int mode) {
    bits_ = bits;
    mode_ = mode;
    if (debug_enabled) {
        printf("SPI::format: bits=%d, mode=%d\n", bits, mode);
    }
}

void SPI::frequency(int hz) {
    frequency_ = hz;
    if (debug_enabled) {
        printf("SPI::frequency: %d Hz\n", hz);
    }
}

int SPI::write(int value) {
    uint8_t tx_byte = static_cast<uint8_t>(value);
    uint8_t rx_byte = spi_next_rx_bytes[this];
    
    // Store the last transmitted byte
    last_tx_byte_ = tx_byte;
    spi_last_tx_bytes[this] = tx_byte;
    
    if (debug_enabled) {
        printf("SPI::write: tx=0x%02X, rx=0x%02X\n", tx_byte, rx_byte);
    }
    
    return rx_byte;
}

void SPI::select() {
    if (debug_enabled) {
        printf("SPI::select\n");
    }
}

void SPI::deselect() {
    if (debug_enabled) {
        printf("SPI::deselect\n");
    }
}

// Implementation of Timer
Timer::Timer() : running_(false) {
    if (debug_enabled) {
        printf("Timer constructor\n");
    }
}

Timer::~Timer() {
    if (debug_enabled) {
        printf("Timer destructor\n");
    }
}

void Timer::start() {
    start_time_ = std::chrono::steady_clock::now();
    running_ = true;
    if (debug_enabled) {
        printf("Timer::start\n");
    }
}

void Timer::stop() {
    running_ = false;
    if (debug_enabled) {
        printf("Timer::stop\n");
    }
}

void Timer::reset() {
    start_time_ = std::chrono::steady_clock::now();
    if (debug_enabled) {
        printf("Timer::reset\n");
    }
}

std::chrono::microseconds Timer::elapsed_time() const {
    if (!running_) {
        return std::chrono::microseconds(0);
    }
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    
    if (debug_enabled) {
        printf("Timer::elapsed_time: %lld us\n", static_cast<long long>(duration.count()));
    }
    
    return duration;
}

// Implementation of wait functions
void wait_us(int us) {
    if (debug_enabled) {
        printf("wait_us: %d us\n", us);
    }
    
    // For testing, we can actually sleep for the requested time
    // std::this_thread::sleep_for(std::chrono::microseconds(us));
    
    // Or we can just return immediately for faster testing
}
