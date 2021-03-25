/*
 * STM32_EEPROM_SPI.h
 *
 *  Created on: 28 mar. 2019
 *      Author: TOSHIBA
 */

#ifndef STM32_EEPROM_SPI_H_
#define STM32_EEPROM_SPI_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
//#include "cmsis_os.h"

//si la libreria hal delay, comentar si es freertos etc
//Comentar si es Low Layer
#define HAL_DRIVER 1

/* M95040 SPI EEPROM defines */
#define EEPROM_WREN  0x06  /*!< Write Enable */
#define EEPROM_WRDI  0x04  /*!< Write Disable */
#define EEPROM_RDSR  0x05  /*!< Read Status Register */
#define EEPROM_WRSR  0x01  /*!< Write Status Register */
#define EEPROM_READ  0x03  /*!< Read from Memory Array */
#define EEPROM_WRITE 0x02  /*!< Write to Memory Array */

#define EEPROM_WIP_FLAG        0x01  /*!< Write In Progress (WIP) flag */

#define EEPROM_PAGESIZE        32    /*!< Pagesize according to documentation */
#define EEPROM_BUFFER_SIZE     32    /*!< EEPROM Buffer size. Setup to your needs */
///////////////////////PORTING :'V
//#define osDelay(x) HAL_Delay(x)

#ifdef HAL_DRIVER
	#define EEPROM_PORT_CS		GPIOB
	#define EEPROM_PORT_WP		GPIOA
	#define EEPROM_PORT_HOLD	GPIOC
	#define EEPROM_CS_PIN		GPIO_PIN_6
	#define EEPROM_WP_PIN		GPIO_PIN_9
	#define EEPROM_HOLD_PIN		GPIO_PIN_7
/*
	#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)//b6 EN LA TARJETA DE RESIDEO
	#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

	#define EEPROM_WP_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)//A9
	#define EEPROM_WP_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)

	#define EEPROM_HOLD_HIGH()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)//C7
	#define EEPROM_HOLD_LOW()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
*/
#else
	//#define GPIO_SET_EEPROM 	1
	//#define GPIO_RESET_EEPROM 	0
	#define EEPROM_PORT_CS		GPIOB
	#define EEPROM_PORT_WP		GPIOA
	#define EEPROM_PORT_HOLD	GPIOC
	#define EEPROM_CS_PIN		LL_GPIO_PIN_6
	#define EEPROM_WP_PIN		LL_GPIO_PIN_9
	#define EEPROM_HOLD_PIN		LL_GPIO_PIN_7
#endif

#define GPIO_SET_EEPROM 	1
#define GPIO_RESET_EEPROM 	0

/**
 * @brief EEPROM Operations statuses
 */
typedef enum {
    EEPROM_STATUS_PENDING,
    EEPROM_STATUS_COMPLETE,
    EEPROM_STATUS_ERROR
} EepromOperations;

#define SET_EEP_H 1

typedef enum
{
  HAL_OK_H       = 0x00U,
  HAL_ERROR_H    = 0x01U,
  HAL_BUSY_H     = 0x02U,
  HAL_TIMEOUT_H  = 0x03
} HAL_StatusTypeDef_H;

void EEPROM_SPI_INIT(void);
EepromOperations EEPROM_SPI_WriteBuffer(void *EEPROM_SPI,uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_SPI_WritePage(void *EEPROM_SPI,uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_SPI_ReadBuffer(void *EEPROM_SPI,uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint8_t EEPROM_SPI_WaitStandbyState(void *EEPROM_SPI);

/* Low layer functions */
uint8_t EEPROM_SendByte(void *EEPROM_SPI,uint8_t byte);
void sEE_WriteEnable(void *EEPROM_SPI);
void sEE_WriteDisable(void *EEPROM_SPI);
void sEE_WriteStatusRegister(void *EEPROM_SPI,uint8_t regval);
uint8_t sEE_ReadStatusRegister(void);

void  EEPROM_SPI_SendInstruction(void *EEPROM_SPI,uint8_t *instruction, uint8_t size);
void  EEPROM_SPI_ReadStatusByte(SPI_HandleTypeDef SPIe, uint8_t *statusByte );
//porting
void HAL_GPIO_WritePinH(void *GPIO,uint16_t PIN,int state);
void HAL_DelayH(uint32_t time);
HAL_StatusTypeDef_H HAL_SPI_TransmitH(void *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef_H HAL_SPI_ReceiveH(void *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* STM32_EEPROM_SPI_H_ */

