/*
 * Test application for CC1200 driver using mock implementations
 * This example demonstrates how to use the CC1200 driver with mock hardware
 * for testing on a computer without actual hardware.
 */

#include <stdio.h>
#include <string.h>

// Include the appropriate driver based on the target platform
#ifdef USE_STM32_HAL
#include "../stm32/CC1200_HAL.h"
#else
#include "../mbed/CC1200.h"
#endif

// Mock response data for CC1200 chip
#define MOCK_PART_NUMBER 0x20  // CC1200 part number
#define MOCK_PART_VERSION 0x01 // Version 1

int main() {
    printf("CC1200 Mock Test Application\n");
    printf("============================\n\n");

#ifdef USE_STM32_HAL
    printf("Using STM32 HAL implementation\n");
    
    // Enable debug output for mock functions
    mock_set_debug(1);
    
    // Initialize HAL
    HAL_Init();
    
    // Create SPI handle
    SPI_HandleTypeDef hspi1;
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    
    HAL_SPI_Init(&hspi1);
    
    // Create CC1200 instance
    CC1200 radio(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0);
    
    // Set up mock responses for chip identification
    // When reading the PARTNUMBER register, return the CC1200 part number
    // When reading the PARTVERSION register, return version 1
    uint8_t rx_buffer[2] = {MOCK_PART_NUMBER, MOCK_PART_VERSION};
    mock_spi_set_rx_buffer(SPI1, rx_buffer, 2);
    
#else
    printf("Using Mbed OS implementation\n");
    
    // Enable debug output for mock functions
    mock_mbed_set_debug(1);
    
    // Create SPI and GPIO instances
    SPI spi(PA_7, PA_6, PA_5, PA_4, use_gpio_ssel);
    DigitalOut rst(PB_0, 1);
    
    // Configure SPI
    spi.format(8, 0);  // 8 bits, mode 0
    spi.frequency(5000000);  // 5 MHz
    
    // Create CC1200 instance
    CC1200 radio(PA_7, PA_6, PA_5, PA_4, PB_0);
    
    // Set up mock responses for chip identification
    // When reading the PARTNUMBER register, return the CC1200 part number
    // When reading the PARTVERSION register, return version 1
    mock_mbed_spi_set_rx_byte(&spi, MOCK_PART_NUMBER);
    
    // Note: In a real implementation, we would need to set up more complex
    // mock responses for the SPI transactions, but this is simplified for
    // demonstration purposes.
#endif

    // Initialize the radio
    printf("\nInitializing CC1200...\n");
    bool success = radio.begin();
    
    if (success) {
        printf("CC1200 initialized successfully!\n");
        
        // Configure radio parameters
        printf("\nConfiguring radio parameters...\n");
        
        // Set radio frequency (915 MHz)
        radio.setRadioFrequency(CC1200::Band::BAND_820_960MHz, 915000000);
        
        // Set modulation format (GFSK)
        radio.setModulationFormat(CC1200::ModFormat::GFSK2);
        
        // Set symbol rate (50 kbps)
        radio.setSymbolRate(50000);
        
        // Set FSK deviation (25 kHz)
        radio.setFSKDeviation(25000);
        
        // Set RX filter bandwidth (100 kHz)
        radio.setRXFilterBandwidth(100000);
        
        // Set output power (10 dBm)
        radio.setOutputPower(10.0f);
        
        // Configure packet format
        radio.setPacketMode(CC1200::PacketMode::VARIABLE_LENGTH, true);
        
        // Enable CRC
        radio.setCRCEnabled(true);
        
        // Configure sync word (0xD391)
        radio.configureSyncWord(0xD391, CC1200::SyncMode::REQUIRE_16_OF_16_SYNC_BITS);
        
        // Configure what happens after RX/TX
        radio.setOnReceiveState(CC1200::State::RX, CC1200::State::IDLE);
        radio.setOnTransmitState(CC1200::State::RX);
        
        printf("Radio configuration complete!\n");
        
        // Transmit a packet
        const char data[] = "Hello, CC1200!";
        printf("\nTransmitting packet: %s\n", data);
        
        if (radio.enqueuePacket(data, strlen(data))) {
            radio.sendCommand(CC1200::Command::TX);
            printf("Packet transmitted successfully!\n");
        } else {
            printf("Failed to transmit packet!\n");
        }
        
        // Switch to receive mode
        printf("\nSwitching to receive mode...\n");
        radio.sendCommand(CC1200::Command::RX);
        
        // In a real application, we would wait for a packet to be received
        // and then process it. For this mock test, we'll just simulate
        // receiving a packet.
        
        printf("\nMock test completed successfully!\n");
    } else {
        printf("Failed to initialize CC1200!\n");
    }
    
    return 0;
}
