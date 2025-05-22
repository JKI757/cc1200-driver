/*
 * Basic example of using the CC1200 driver with STM32 HAL
 * This example demonstrates basic transmission and reception
 * for the STM32F411CEU microcontroller
 */

#include "stm32f4xx_hal.h"
#include "../stm32/CC1200_HAL.h"
#include <stdio.h>

// SPI handle
SPI_HandleTypeDef hspi1;

// Function prototypes
void SystemClock_Config(void);
void Error_Handler(void);
void SPI1_Init(void);
void GPIO_Init(void);

// GPIO definitions for CC1200
#define CC1200_CS_PIN       GPIO_PIN_4
#define CC1200_CS_PORT      GPIOA
#define CC1200_RST_PIN      GPIO_PIN_0
#define CC1200_RST_PORT     GPIOB

int main(void)
{
  // Initialize HAL
  HAL_Init();
  
  // Configure system clock
  SystemClock_Config();
  
  // Initialize peripherals
  GPIO_Init();
  SPI1_Init();
  
  // Create CC1200 instance
  CC1200 radio(&hspi1, CC1200_CS_PORT, CC1200_CS_PIN, CC1200_RST_PORT, CC1200_RST_PIN);
  
  // Initialize the radio
  if (!radio.begin()) {
    printf("Failed to initialize CC1200\n");
    Error_Handler();
  }
  
  printf("CC1200 initialized successfully\n");
  
  // Configure radio parameters
  radio.setRadioFrequency(CC1200::Band::BAND_820_960MHz, 915000000);  // 915 MHz
  radio.setModulationFormat(CC1200::ModFormat::GFSK2);                // GFSK modulation
  radio.setSymbolRate(50000);                                         // 50 kbps
  radio.setFSKDeviation(25000);                                       // 25 kHz deviation
  radio.setRXFilterBandwidth(100000);                                 // 100 kHz bandwidth
  radio.setOutputPower(10.0f);                                        // 10 dBm output power
  
  // Configure packet format
  radio.setPacketMode(CC1200::PacketMode::VARIABLE_LENGTH, true);     // Variable length packets with status bytes
  radio.setCRCEnabled(true);                                          // Enable CRC
  
  // Configure sync word (0xD391)
  radio.configureSyncWord(0xD391, CC1200::SyncMode::REQUIRE_16_OF_16_SYNC_BITS);
  
  // Configure what happens after RX/TX
  radio.setOnReceiveState(CC1200::State::RX, CC1200::State::IDLE);    // Stay in RX after receiving a packet
  radio.setOnTransmitState(CC1200::State::RX);                        // Go to RX after transmitting
  
  // Main loop
  bool transmitMode = true;
  uint32_t lastActionTime = HAL_GetTick();
  const uint32_t actionInterval = 5000;  // 5 seconds between actions
  
  while (1) {
    uint32_t currentTime = HAL_GetTick();
    
    // Alternate between transmit and receive every 5 seconds
    if (currentTime - lastActionTime >= actionInterval) {
      lastActionTime = currentTime;
      
      if (transmitMode) {
        // Transmit a packet
        const char data[] = "Hello from STM32F411!";
        printf("Transmitting packet: %s\n", data);
        
        if (radio.enqueuePacket(data, sizeof(data) - 1)) {
          radio.sendCommand(CC1200::Command::TX);
          printf("Packet enqueued for transmission\n");
        } else {
          printf("Failed to enqueue packet\n");
        }
      } else {
        // Start receiving
        printf("Switching to receive mode\n");
        radio.sendCommand(CC1200::Command::RX);
      }
      
      transmitMode = !transmitMode;  // Toggle mode for next time
    }
    
    // Check for received packets when in receive mode
    if (!transmitMode) {
      if (radio.hasReceivedPacket()) {
        char buffer[128];
        size_t len = radio.receivePacket(buffer, sizeof(buffer));
        
        if (len > 0) {
          // Null-terminate the string
          if (len < sizeof(buffer)) {
            buffer[len] = '\0';
          } else {
            buffer[sizeof(buffer) - 1] = '\0';
          }
          
          printf("Received packet (%u bytes): %s\n", (unsigned int)len, buffer);
          
          // Get signal quality information
          float rssi = radio.getRSSIRegister();
          uint8_t lqi = radio.getLQIRegister();
          printf("RSSI: %.1f dBm, LQI: %u\n", rssi, lqi);
        }
      }
    }
    
    // Small delay to prevent busy-waiting
    HAL_Delay(10);
  }
}

void SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // Adjust for 5MHz
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  // Enable GPIO clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  // Configure CS pin
  GPIO_InitStruct.Pin = CC1200_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CC1200_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(CC1200_CS_PORT, CC1200_CS_PIN, GPIO_PIN_SET);  // Deselect by default
  
  // Configure RST pin
  GPIO_InitStruct.Pin = CC1200_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CC1200_RST_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(CC1200_RST_PORT, CC1200_RST_PIN, GPIO_PIN_SET);  // Not in reset by default
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Configure the main internal regulator output voltage
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Initialize the RCC Oscillators
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // Initialize the CPU, AHB and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  // Turn on an error LED or provide some indication
  while(1) {
    // Infinite loop
  }
}

// This function is called by printf for output
int _write(int file, char *ptr, int len)
{
  // Implement your own UART or other output mechanism here
  // For example, using HAL_UART_Transmit if you have a UART configured
  
  // For now, just return the length to avoid compiler warnings
  return len;
}
