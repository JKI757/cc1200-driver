/*
 * Mock implementation of STM32 HAL for testing purposes
 * This file provides mock definitions for STM32 HAL functions and types
 * to allow compiling and testing the CC1200 driver on a computer.
 */

#ifndef STM32F4XX_HAL_MOCK_H
#define STM32F4XX_HAL_MOCK_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Basic types */
typedef uint32_t HAL_StatusTypeDef;
typedef uint32_t HAL_LockTypeDef;

#define HAL_OK                      ((HAL_StatusTypeDef)0x00U)
#define HAL_ERROR                   ((HAL_StatusTypeDef)0x01U)
#define HAL_BUSY                    ((HAL_StatusTypeDef)0x02U)
#define HAL_TIMEOUT                 ((HAL_StatusTypeDef)0x03U)

#define HAL_MAX_DELAY               0xFFFFFFFFU

/* GPIO definitions */
typedef enum {
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
} GPIO_PinState;

typedef struct {
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
  uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct {
  void* dummy;  // Just a placeholder to make the struct non-empty
} GPIO_TypeDef;

/* GPIO pin definitions */
#define GPIO_PIN_0                  ((uint16_t)0x0001)
#define GPIO_PIN_1                  ((uint16_t)0x0002)
#define GPIO_PIN_2                  ((uint16_t)0x0004)
#define GPIO_PIN_3                  ((uint16_t)0x0008)
#define GPIO_PIN_4                  ((uint16_t)0x0010)
#define GPIO_PIN_5                  ((uint16_t)0x0020)
#define GPIO_PIN_6                  ((uint16_t)0x0040)
#define GPIO_PIN_7                  ((uint16_t)0x0080)
#define GPIO_PIN_8                  ((uint16_t)0x0100)
#define GPIO_PIN_9                  ((uint16_t)0x0200)
#define GPIO_PIN_10                 ((uint16_t)0x0400)
#define GPIO_PIN_11                 ((uint16_t)0x0800)
#define GPIO_PIN_12                 ((uint16_t)0x1000)
#define GPIO_PIN_13                 ((uint16_t)0x2000)
#define GPIO_PIN_14                 ((uint16_t)0x4000)
#define GPIO_PIN_15                 ((uint16_t)0x8000)
#define GPIO_PIN_All                ((uint16_t)0xFFFF)

/* GPIO mode definitions */
#define GPIO_MODE_INPUT             0x00000000U
#define GPIO_MODE_OUTPUT_PP         0x00000001U
#define GPIO_MODE_OUTPUT_OD         0x00000011U
#define GPIO_MODE_AF_PP             0x00000002U
#define GPIO_MODE_AF_OD             0x00000012U
#define GPIO_MODE_ANALOG            0x00000003U
#define GPIO_MODE_IT_RISING         0x10110000U
#define GPIO_MODE_IT_FALLING        0x10210000U
#define GPIO_MODE_IT_RISING_FALLING 0x10310000U
#define GPIO_MODE_EVT_RISING        0x10120000U
#define GPIO_MODE_EVT_FALLING       0x10220000U
#define GPIO_MODE_EVT_RISING_FALLING 0x10320000U

/* GPIO pull definitions */
#define GPIO_NOPULL                 0x00000000U
#define GPIO_PULLUP                 0x00000001U
#define GPIO_PULLDOWN               0x00000002U

/* GPIO speed definitions */
#define GPIO_SPEED_FREQ_LOW         0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM      0x00000001U
#define GPIO_SPEED_FREQ_HIGH        0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U

/* SPI definitions */
typedef struct {
  uint32_t Mode;
  uint32_t Direction;
  uint32_t DataSize;
  uint32_t CLKPolarity;
  uint32_t CLKPhase;
  uint32_t NSS;
  uint32_t BaudRatePrescaler;
  uint32_t FirstBit;
  uint32_t TIMode;
  uint32_t CRCCalculation;
  uint32_t CRCPolynomial;
} SPI_InitTypeDef;

typedef struct __SPI_HandleTypeDef {
  void* Instance;
  SPI_InitTypeDef Init;
  HAL_LockTypeDef Lock;
  uint32_t State;
  void* pTxBuffPtr;
  uint16_t TxXferSize;
  uint16_t TxXferCount;
  void* pRxBuffPtr;
  uint16_t RxXferSize;
  uint16_t RxXferCount;
  struct {
    void* hdmatx;
    void* hdmarx;
  } hdma;
  void (*TxCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*RxCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*TxRxCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*TxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*RxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*TxRxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*ErrorCallback)(struct __SPI_HandleTypeDef *hspi);
  void (*AbortCpltCallback)(struct __SPI_HandleTypeDef *hspi);
  uint32_t MspInitCallback;
  uint32_t MspDeInitCallback;
} SPI_HandleTypeDef;



/* SPI mode definitions */
#define SPI_MODE_SLAVE              0x00000000U
#define SPI_MODE_MASTER             0x00000104U

/* SPI direction definitions */
#define SPI_DIRECTION_2LINES        0x00000000U
#define SPI_DIRECTION_2LINES_RXONLY 0x00000400U
#define SPI_DIRECTION_1LINE         0x00008000U

/* SPI data size definitions */
#define SPI_DATASIZE_8BIT           0x00000000U
#define SPI_DATASIZE_16BIT          0x00000800U

/* SPI clock polarity definitions */
#define SPI_POLARITY_LOW            0x00000000U
#define SPI_POLARITY_HIGH           0x00000002U

/* SPI clock phase definitions */
#define SPI_PHASE_1EDGE             0x00000000U
#define SPI_PHASE_2EDGE             0x00000001U

/* SPI NSS definitions */
#define SPI_NSS_SOFT                0x00000200U
#define SPI_NSS_HARD_INPUT          0x00000000U
#define SPI_NSS_HARD_OUTPUT         0x00000004U

/* SPI baud rate prescaler definitions */
#define SPI_BAUDRATEPRESCALER_2     0x00000000U
#define SPI_BAUDRATEPRESCALER_4     0x00000008U
#define SPI_BAUDRATEPRESCALER_8     0x00000010U
#define SPI_BAUDRATEPRESCALER_16    0x00000018U
#define SPI_BAUDRATEPRESCALER_32    0x00000020U
#define SPI_BAUDRATEPRESCALER_64    0x00000028U
#define SPI_BAUDRATEPRESCALER_128   0x00000030U
#define SPI_BAUDRATEPRESCALER_256   0x00000038U

/* SPI first bit definitions */
#define SPI_FIRSTBIT_MSB            0x00000000U
#define SPI_FIRSTBIT_LSB            0x00000080U

/* SPI TI mode definitions */
#define SPI_TIMODE_DISABLE          0x00000000U
#define SPI_TIMODE_ENABLE           0x00000010U

/* SPI CRC calculation definitions */
#define SPI_CRCCALCULATION_DISABLE  0x00000000U
#define SPI_CRCCALCULATION_ENABLE   0x00002000U

/* Mock instances for GPIO ports */
extern GPIO_TypeDef* GPIOA;
extern GPIO_TypeDef* GPIOB;
extern GPIO_TypeDef* GPIOC;
extern GPIO_TypeDef* GPIOD;
extern GPIO_TypeDef* GPIOE;
extern GPIO_TypeDef* GPIOF;
extern GPIO_TypeDef* GPIOG;
extern GPIO_TypeDef* GPIOH;

/* Mock instances for SPI */
extern void* SPI1;
extern void* SPI2;
extern void* SPI3;

/* Function prototypes */
void HAL_Init(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef* hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef* hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout);

/* Mock functions for testing */
void mock_set_debug(int enable);
void mock_spi_set_rx_byte(void* spi_instance, uint8_t byte);
void mock_spi_set_rx_buffer(void* spi_instance, uint8_t* buffer, size_t size);
uint8_t mock_spi_get_last_tx_byte(void* spi_instance);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_MOCK_H */
