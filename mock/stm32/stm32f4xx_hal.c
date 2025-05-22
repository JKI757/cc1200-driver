/*
 * Mock implementation of STM32 HAL for testing purposes
 * This file provides mock implementations of STM32 HAL functions
 * to allow compiling and testing the CC1200 driver on a computer.
 */

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Mock instances for GPIO ports */
GPIO_TypeDef _GPIOA = {0};
GPIO_TypeDef _GPIOB = {0};
GPIO_TypeDef _GPIOC = {0};
GPIO_TypeDef _GPIOD = {0};
GPIO_TypeDef _GPIOE = {0};
GPIO_TypeDef _GPIOF = {0};
GPIO_TypeDef _GPIOG = {0};
GPIO_TypeDef _GPIOH = {0};

GPIO_TypeDef* GPIOA = &_GPIOA;
GPIO_TypeDef* GPIOB = &_GPIOB;
GPIO_TypeDef* GPIOC = &_GPIOC;
GPIO_TypeDef* GPIOD = &_GPIOD;
GPIO_TypeDef* GPIOE = &_GPIOE;
GPIO_TypeDef* GPIOF = &_GPIOF;
GPIO_TypeDef* GPIOG = &_GPIOG;
GPIO_TypeDef* GPIOH = &_GPIOH;

/* Mock instances for SPI */
void* SPI1 = (void*)1;
void* SPI2 = (void*)2;
void* SPI3 = (void*)3;

/* Mock GPIO state */
typedef struct {
    uint16_t pins[8];  // State of pins for 8 GPIO ports (A-H)
} MockGPIOState;

static MockGPIOState gpio_state = {0};

/* Mock SPI state */
typedef struct {
    uint8_t last_tx_byte;
    uint8_t next_rx_byte;
    uint8_t* rx_buffer;
    size_t rx_buffer_size;
    size_t rx_buffer_pos;
} MockSPIState;

static MockSPIState spi_state[3] = {0};  // For SPI1, SPI2, SPI3

/* Debug flag */
static int debug_enabled = 0;

/* Start time for HAL_GetTick */
static clock_t start_time = 0;

/* Enable debug output for mock functions */
void mock_set_debug(int enable) {
    debug_enabled = enable;
}

/* Initialize HAL */
void HAL_Init(void) {
    if (debug_enabled) {
        printf("HAL_Init called\n");
    }
    
    /* Initialize start time for HAL_GetTick */
    start_time = clock();
    
    /* Initialize GPIO state */
    memset(&gpio_state, 0, sizeof(gpio_state));
    
    /* Initialize SPI state */
    memset(spi_state, 0, sizeof(spi_state));
    
    /* Set default RX values for SPI */
    for (int i = 0; i < 3; i++) {
        spi_state[i].next_rx_byte = 0xFF;  // Default to 0xFF for reads
    }
}

/* Get tick count in milliseconds */
uint32_t HAL_GetTick(void) {
    clock_t current = clock();
    return (uint32_t)((current - start_time) * 1000 / CLOCKS_PER_SEC);
}

/* Delay for specified milliseconds */
void HAL_Delay(uint32_t Delay) {
    if (debug_enabled) {
        printf("HAL_Delay(%lu ms)\n", Delay);
    }
    
    /* In a real implementation, we would sleep here */
    /* For testing, we'll just return immediately */
}

/* Initialize GPIO */
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_Init) {
    if (debug_enabled) {
        printf("HAL_GPIO_Init(GPIO%c, Pin: 0x%04X)\n", 
               GPIOx == GPIOA ? 'A' : 
               GPIOx == GPIOB ? 'B' : 
               GPIOx == GPIOC ? 'C' : 
               GPIOx == GPIOD ? 'D' : 
               GPIOx == GPIOE ? 'E' : 
               GPIOx == GPIOF ? 'F' : 
               GPIOx == GPIOG ? 'G' : 
               GPIOx == GPIOH ? 'H' : '?', 
               GPIO_Init->Pin);
    }
    
    /* Nothing to do in mock implementation */
}

/* Deinitialize GPIO */
void HAL_GPIO_DeInit(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) {
    if (debug_enabled) {
        printf("HAL_GPIO_DeInit(GPIO%c, Pin: 0x%04X)\n", 
               GPIOx == GPIOA ? 'A' : 
               GPIOx == GPIOB ? 'B' : 
               GPIOx == GPIOC ? 'C' : 
               GPIOx == GPIOD ? 'D' : 
               GPIOx == GPIOE ? 'E' : 
               GPIOx == GPIOF ? 'F' : 
               GPIOx == GPIOG ? 'G' : 
               GPIOx == GPIOH ? 'H' : '?', 
               GPIO_Pin);
    }
    
    /* Nothing to do in mock implementation */
}

/* Read GPIO pin state */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    int port_idx = -1;
    
    if (GPIOx == GPIOA) port_idx = 0;
    else if (GPIOx == GPIOB) port_idx = 1;
    else if (GPIOx == GPIOC) port_idx = 2;
    else if (GPIOx == GPIOD) port_idx = 3;
    else if (GPIOx == GPIOE) port_idx = 4;
    else if (GPIOx == GPIOF) port_idx = 5;
    else if (GPIOx == GPIOG) port_idx = 6;
    else if (GPIOx == GPIOH) port_idx = 7;
    
    if (port_idx >= 0) {
        GPIO_PinState state = (gpio_state.pins[port_idx] & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        
        if (debug_enabled) {
            printf("HAL_GPIO_ReadPin(GPIO%c, Pin: 0x%04X) = %d\n", 
                   'A' + port_idx, GPIO_Pin, state);
        }
        
        return state;
    }
    
    return GPIO_PIN_RESET;
}

/* Write GPIO pin state */
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    int port_idx = -1;
    
    if (GPIOx == GPIOA) port_idx = 0;
    else if (GPIOx == GPIOB) port_idx = 1;
    else if (GPIOx == GPIOC) port_idx = 2;
    else if (GPIOx == GPIOD) port_idx = 3;
    else if (GPIOx == GPIOE) port_idx = 4;
    else if (GPIOx == GPIOF) port_idx = 5;
    else if (GPIOx == GPIOG) port_idx = 6;
    else if (GPIOx == GPIOH) port_idx = 7;
    
    if (port_idx >= 0) {
        if (PinState == GPIO_PIN_SET) {
            gpio_state.pins[port_idx] |= GPIO_Pin;
        } else {
            gpio_state.pins[port_idx] &= ~GPIO_Pin;
        }
        
        if (debug_enabled) {
            printf("HAL_GPIO_WritePin(GPIO%c, Pin: 0x%04X, State: %d)\n", 
                   'A' + port_idx, GPIO_Pin, PinState);
        }
    }
}

/* Toggle GPIO pin state */
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    int port_idx = -1;
    
    if (GPIOx == GPIOA) port_idx = 0;
    else if (GPIOx == GPIOB) port_idx = 1;
    else if (GPIOx == GPIOC) port_idx = 2;
    else if (GPIOx == GPIOD) port_idx = 3;
    else if (GPIOx == GPIOE) port_idx = 4;
    else if (GPIOx == GPIOF) port_idx = 5;
    else if (GPIOx == GPIOG) port_idx = 6;
    else if (GPIOx == GPIOH) port_idx = 7;
    
    if (port_idx >= 0) {
        gpio_state.pins[port_idx] ^= GPIO_Pin;
        
        if (debug_enabled) {
            printf("HAL_GPIO_TogglePin(GPIO%c, Pin: 0x%04X)\n", 
                   'A' + port_idx, GPIO_Pin);
        }
    }
}

/* Lock GPIO pin configuration */
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (debug_enabled) {
        printf("HAL_GPIO_LockPin(GPIO%c, Pin: 0x%04X)\n", 
               GPIOx == GPIOA ? 'A' : 
               GPIOx == GPIOB ? 'B' : 
               GPIOx == GPIOC ? 'C' : 
               GPIOx == GPIOD ? 'D' : 
               GPIOx == GPIOE ? 'E' : 
               GPIOx == GPIOF ? 'F' : 
               GPIOx == GPIOG ? 'G' : 
               GPIOx == GPIOH ? 'H' : '?', 
               GPIO_Pin);
    }
    
    return HAL_OK;
}

/* Initialize SPI */
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef* hspi) {
    if (debug_enabled) {
        printf("HAL_SPI_Init(SPI%d)\n", 
               hspi->Instance == SPI1 ? 1 : 
               hspi->Instance == SPI2 ? 2 : 
               hspi->Instance == SPI3 ? 3 : 0);
    }
    
    return HAL_OK;
}

/* Deinitialize SPI */
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef* hspi) {
    if (debug_enabled) {
        printf("HAL_SPI_DeInit(SPI%d)\n", 
               hspi->Instance == SPI1 ? 1 : 
               hspi->Instance == SPI2 ? 2 : 
               hspi->Instance == SPI3 ? 3 : 0);
    }
    
    return HAL_OK;
}

/* Helper function to get SPI index */
static int get_spi_index(void* instance) {
    if (instance == SPI1) return 0;
    if (instance == SPI2) return 1;
    if (instance == SPI3) return 2;
    return -1;
}

/* Set the next byte to be received in SPI transactions */
void mock_spi_set_rx_byte(void* spi_instance, uint8_t byte) {
    int idx = get_spi_index(spi_instance);
    if (idx >= 0) {
        spi_state[idx].next_rx_byte = byte;
    }
}

/* Set a buffer of bytes to be received in SPI transactions */
void mock_spi_set_rx_buffer(void* spi_instance, uint8_t* buffer, size_t size) {
    int idx = get_spi_index(spi_instance);
    if (idx < 0) return;
    
    spi_state[idx].rx_buffer = buffer;
    spi_state[idx].rx_buffer_size = size;
    spi_state[idx].rx_buffer_pos = 0;
    
    if (debug_enabled) {
        printf("Setting mock SPI RX buffer for SPI%d with %zu bytes: ", idx+1, size);
        for (size_t i = 0; i < size; i++) {
            printf("0x%02X ", buffer[i]);
        }
        printf("\n");
    }
}

/* Get the last byte transmitted in SPI transactions */
uint8_t mock_spi_get_last_tx_byte(void* spi_instance) {
    int idx = get_spi_index(spi_instance);
    if (idx >= 0) {
        return spi_state[idx].last_tx_byte;
    }
    return 0;
}

/* SPI transmit */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    int idx = get_spi_index(hspi->Instance);
    
    if (debug_enabled) {
        printf("HAL_SPI_Transmit(SPI%d, Data[0]: 0x%02X, Size: %d)\n", 
               idx + 1, pData[0], Size);
    }
    
    if (idx >= 0 && Size > 0) {
        spi_state[idx].last_tx_byte = pData[Size - 1];
    }
    
    return HAL_OK;
}

/* SPI receive */
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    int idx = get_spi_index(hspi->Instance);
    
    if (debug_enabled) {
        printf("HAL_SPI_Receive(SPI%d, Size: %d)\n", idx + 1, Size);
    }
    
    if (idx >= 0) {
        for (uint16_t i = 0; i < Size; i++) {
            if (spi_state[idx].rx_buffer && spi_state[idx].rx_buffer_pos < spi_state[idx].rx_buffer_size) {
                pData[i] = spi_state[idx].rx_buffer[spi_state[idx].rx_buffer_pos++];
            } else {
                pData[i] = spi_state[idx].next_rx_byte;
            }
        }
    }
    
    return HAL_OK;
}

/* SPI transmit and receive */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) {
    int idx = get_spi_index(hspi->Instance);
    if (idx < 0) return HAL_ERROR;
    
    if (debug_enabled) {
        printf("HAL_SPI_TransmitReceive(SPI%d, TxData[0]: 0x%02X, Size: %d)\n", idx+1, pTxData[0], Size);
    }
    
    // Special handling for CC1200 extended register reads
    // When reading PARTNUMBER (0x8F) or PARTVERSION (0x90), return the mock values
    static int ext_reg_read_state = 0;
    static uint8_t ext_reg_addr = 0;
    
    for (uint16_t i = 0; i < Size; i++) {
        /* Store the transmitted byte */
        spi_state[idx].last_tx_byte = pTxData[i];
        
        // Handle extended register read sequence
        if (pTxData[i] == 0xAF) { // 0x2F | 0x80 (CC1200_EXT_ADDR | CC1200_READ)
            ext_reg_read_state = 1;
            pRxData[i] = 0; // Status byte, not important for our test
            continue;
        }
        
        if (ext_reg_read_state == 1) {
            ext_reg_addr = pTxData[i];
            ext_reg_read_state = 2;
            pRxData[i] = 0; // Don't care about this byte
            continue;
        }
        
        if (ext_reg_read_state == 2) {
            ext_reg_read_state = 0;
            // Return appropriate mock value based on register address
            if (ext_reg_addr == 0x8F) { // PARTNUMBER
                pRxData[i] = 0x20; // CC1200 part number
            } else if (ext_reg_addr == 0x90) { // PARTVERSION
                pRxData[i] = 0x01; // Version 1
            } else {
                // For other registers, use the buffer if available
                if (spi_state[idx].rx_buffer != NULL && spi_state[idx].rx_buffer_pos < spi_state[idx].rx_buffer_size) {
                    pRxData[i] = spi_state[idx].rx_buffer[spi_state[idx].rx_buffer_pos++];
                } else {
                    pRxData[i] = spi_state[idx].next_rx_byte;
                }
            }
            continue;
        }
        
        /* Get the next byte to receive */
        if (spi_state[idx].rx_buffer != NULL && spi_state[idx].rx_buffer_pos < spi_state[idx].rx_buffer_size) {
            pRxData[i] = spi_state[idx].rx_buffer[spi_state[idx].rx_buffer_pos++];
        } else {
            pRxData[i] = spi_state[idx].next_rx_byte;
        }
    }
    
    return HAL_OK;
}
