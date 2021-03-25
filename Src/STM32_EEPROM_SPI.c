/*
 * STM32_EEPROM_SPI.c
 *
 *  Created on: 28 mar. 2019
 *      Author: TOSHIBA
 */


/**
 * Copyright Nikita Bulaev 2017-2019
 *
 * Some parts of this lib is taken from STM32 StdPerif libriary
 * stm32l152d_eval_spi_ee.c and adopted for the HAL.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "STM32_EEPROM_SPI.h"

//SPI_HandleTypeDef * EEPROM_SPI;
//uint8_t EEPROM_StatusByte;
//uint8_t RxBuffer[EEPROM_BUFFER_SIZE] = {0x00};

/**
 * @brief Init EEPROM SPI
 *
 * @param hspi Pointer to SPI struct handler
 */
void EEPROM_SPI_INIT(void) {
    //EEPROM_SPI = hspi;
    //EEPROM_WP_HIGH();
    //EEPROM_HOLD_HIGH();
	HAL_GPIO_WritePinH(EEPROM_PORT_WP,EEPROM_WP_PIN,GPIO_SET_EEPROM);
	HAL_GPIO_WritePinH(EEPROM_PORT_HOLD,EEPROM_HOLD_PIN,GPIO_SET_EEPROM);

}

/**
  * @brief  Writes more than one byte to the EEPROM with a single WRITE cycle
  *         (Page WRITE sequence).
  *
  * @note   The number of byte can't exceed the EEPROM page size.
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr: EEPROM's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the EEPROM, must be equal
  *         or less than "EEPROM_PAGESIZE" value.
  * @retval EepromOperations value: EEPROM_STATUS_COMPLETE or EEPROM_STATUS_ERROR
  */
EepromOperations EEPROM_SPI_WritePage(void *EEPROM_SPI,uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
	#ifdef HAL_DRIVER
	while (((SPI_HandleTypeDef *)EEPROM_SPI)->State != HAL_SPI_STATE_READY) {

        HAL_DelayH(1);
    }
	#else

	#endif
    HAL_StatusTypeDef_H spiTransmitStatus;

    sEE_WriteEnable(EEPROM_SPI);

    /*
        We gonna send commands in one packet of 3 bytes
     */
    uint8_t header[3];

    header[0] = EEPROM_WRITE;   // Send "Write to Memory" instruction
    header[1] = WriteAddr >> 8; // Send 16-bit address
    header[2] = WriteAddr;

    // Select the EEPROM: Chip Select low

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_RESET_EEPROM);

    EEPROM_SPI_SendInstruction(EEPROM_SPI,(uint8_t*)header, 3);

    // Make 5 attemtps to write the data
    for (uint8_t i = 0; i < 5; i++) {
        spiTransmitStatus = HAL_SPI_TransmitH(EEPROM_SPI, pBuffer, NumByteToWrite, 100);

        if (spiTransmitStatus == HAL_BUSY_H) {

            HAL_DelayH(5);
        } else {
            break;
        }
    }

    // Deselect the EEPROM: Chip Select high

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_SET_EEPROM);
    // Wait the end of EEPROM writing
    EEPROM_SPI_WaitStandbyState(EEPROM_SPI);

    // Disable the write access to the EEPROM
    sEE_WriteDisable(EEPROM_SPI);

    if (spiTransmitStatus == HAL_ERROR_H) {
        return EEPROM_STATUS_ERROR;
    } else {
        return EEPROM_STATUS_COMPLETE;
    }
}

/**
  * @brief  Writes block of data to the EEPROM. In this function, the number of
  *         WRITE cycles are reduced, using Page WRITE sequence.
  *
  * @param  pBuffer: pointer to the buffer  containing the data to be written
  *         to the EEPROM.
  * @param  WriteAddr: EEPROM's internal address to write to.
  * @param  NumByteToWrite: number of bytes to write to the EEPROM.
  * @retval EepromOperations value: EEPROM_STATUS_COMPLETE or EEPROM_STATUS_ERROR
  */
EepromOperations EEPROM_SPI_WriteBuffer(void *EEPROM_SPI,uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
    uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
    uint16_t sEE_DataNum = 0;

    EepromOperations pageWriteStatus = EEPROM_STATUS_PENDING;

    Addr = WriteAddr % EEPROM_PAGESIZE;
    count = EEPROM_PAGESIZE - Addr;
    NumOfPage =  NumByteToWrite / EEPROM_PAGESIZE;
    NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

    if (Addr == 0) { /* WriteAddr is EEPROM_PAGESIZE aligned  */
        if (NumOfPage == 0) { /* NumByteToWrite < EEPROM_PAGESIZE */
            sEE_DataNum = NumByteToWrite;
            pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }

        } else { /* NumByteToWrite > EEPROM_PAGESIZE */
            while (NumOfPage--) {
                sEE_DataNum = EEPROM_PAGESIZE;
                pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }

                WriteAddr +=  EEPROM_PAGESIZE;
                pBuffer += EEPROM_PAGESIZE;
            }

            sEE_DataNum = NumOfSingle;
            pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }
        }
    } else { /* WriteAddr is not EEPROM_PAGESIZE aligned  */
        if (NumOfPage == 0) { /* NumByteToWrite < EEPROM_PAGESIZE */
            if (NumOfSingle > count) { /* (NumByteToWrite + WriteAddr) > EEPROM_PAGESIZE */
                temp = NumOfSingle - count;
                sEE_DataNum = count;
                pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }

                WriteAddr +=  count;
                pBuffer += count;

                sEE_DataNum = temp;
                pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);
            } else {
                sEE_DataNum = NumByteToWrite;
                pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);
            }

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }
        } else { /* NumByteToWrite > EEPROM_PAGESIZE */
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / EEPROM_PAGESIZE;
            NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;

            sEE_DataNum = count;

            pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

            if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                return pageWriteStatus;
            }

            WriteAddr +=  count;
            pBuffer += count;

            while (NumOfPage--) {
                sEE_DataNum = EEPROM_PAGESIZE;

                pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }

                WriteAddr +=  EEPROM_PAGESIZE;
                pBuffer += EEPROM_PAGESIZE;
            }

            if (NumOfSingle != 0) {
                sEE_DataNum = NumOfSingle;

                pageWriteStatus = EEPROM_SPI_WritePage(EEPROM_SPI,pBuffer, WriteAddr, sEE_DataNum);

                if (pageWriteStatus != EEPROM_STATUS_COMPLETE) {
                    return pageWriteStatus;
                }
            }
        }
    }

    return EEPROM_STATUS_COMPLETE;
}

/**
  * @brief  Reads a block of data from the EEPROM.
  *
  * @param  pBuffer: pointer to the buffer that receives the data read from the EEPROM.
  * @param  ReadAddr: EEPROM's internal address to read from.
  * @param  NumByteToRead: number of bytes to read from the EEPROM.
  * @retval None
  */
EepromOperations EEPROM_SPI_ReadBuffer(void *EEPROM_SPI,uint8_t* pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead) {
	#ifdef HAL_DRIVER
	while (((SPI_HandleTypeDef *)EEPROM_SPI)->State != HAL_SPI_STATE_READY) {

        HAL_DelayH(1);
    }
	#else

	#endif
    /*
        We gonna send all commands in one packet of 3 bytes
     */

    uint8_t header[3];

    header[0] = EEPROM_READ;    // Send "Read from Memory" instruction
    header[1] = ReadAddr >> 8;  // Send 16-bit address
    header[2] = ReadAddr;

    // Select the EEPROM: Chip Select low

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_RESET_EEPROM);
    /* Send WriteAddr address byte to read from */
    EEPROM_SPI_SendInstruction(EEPROM_SPI,header, 3);

    while (HAL_SPI_ReceiveH(EEPROM_SPI, (uint8_t*)pBuffer, NumByteToRead, 200) == HAL_BUSY_H) {

        HAL_DelayH(1);
    };

    // Deselect the EEPROM: Chip Select high

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_SET_EEPROM);
    return EEPROM_STATUS_COMPLETE;
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  *
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint8_t EEPROM_SendByte(void *EEPROM_SPI,uint8_t byte) {
    uint8_t answerByte;

    /* Loop while DR register in not empty */
	#ifdef HAL_DRIVER
    while (((SPI_HandleTypeDef *)EEPROM_SPI)->State == HAL_SPI_STATE_RESET) {

        HAL_DelayH(1);
    }
	#else
	#endif

    /* Send byte through the SPI peripheral */
    if (HAL_SPI_TransmitH(EEPROM_SPI, &byte, 1, 200) != HAL_OK_H) {
        Error_Handler();
    }

    /* Wait to receive a byte */
	#ifdef HAL_DRIVER
    while (((SPI_HandleTypeDef *)EEPROM_SPI)->State == HAL_SPI_STATE_RESET) {

        HAL_DelayH(1);
    }
	#else
	#endif
    /* Return the byte read from the SPI bus */
    if (HAL_SPI_ReceiveH(EEPROM_SPI, &answerByte, 1, 200) != HAL_OK_H) {
        Error_Handler();
    }

    return (uint8_t)answerByte;
}
/**
  * @brief  Enables the write access to the EEPROM.
  *
  * @param  None
  * @retval None
  */
void sEE_WriteEnable(void *EEPROM_SPI) {
    // Select the EEPROM: Chip Select low

	HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_RESET_EEPROM);
    uint8_t command[1] = { EEPROM_WREN };
    /* Send "Write Enable" instruction */
    EEPROM_SPI_SendInstruction(EEPROM_SPI,(uint8_t*)command, 1);

    // Deselect the EEPROM: Chip Select high

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_SET_EEPROM);;
}

/**
  * @brief  Disables the write access to the EEPROM.
  *
  * @param  None
  * @retval None
  */
void sEE_WriteDisable(void *EEPROM_SPI) {
    // Select the EEPROM: Chip Select low

	HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_RESET_EEPROM);
    uint8_t command[1] = { EEPROM_WRDI };

    /* Send "Write Disable" instruction */
    EEPROM_SPI_SendInstruction(EEPROM_SPI,(uint8_t*)command, 1);

    // Deselect the EEPROM: Chip Select high

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_SET_EEPROM);
}

/**
  * @brief  Write new value in EEPROM Status Register.
  *
  * @param  regval : new value of register
  * @retval None
  */
void sEE_WriteStatusRegister(void *EEPROM_SPI,uint8_t regval) {
    uint8_t command[2];

    command[0] = EEPROM_WRSR;
    command[1] = regval;

    // Enable the write access to the EEPROM
    sEE_WriteEnable(EEPROM_SPI);

    // Select the EEPROM: Chip Select low

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_RESET_EEPROM);
    // Send "Write Status Register" instruction
    // and Regval in one packet
    EEPROM_SPI_SendInstruction(EEPROM_SPI,(uint8_t*)command, 2);

    // Deselect the EEPROM: Chip Select high

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_SET_EEPROM);
    sEE_WriteDisable(EEPROM_SPI);
}


/**
  * @brief  Polls the status of the Write In Progress (WIP) flag in the EEPROM's
  *         status register and loop until write operation has completed.
  *
  * @param  None
  * @retval None
  */
uint8_t EEPROM_SPI_WaitStandbyState(void *EEPROM_SPI) {
    uint8_t sEEstatus[1] = { 0x00 };
    uint8_t command[1] = { EEPROM_RDSR };

    // Select the EEPROM: Chip Select low

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_RESET_EEPROM);
    // Send "Read Status Register" instruction
    EEPROM_SPI_SendInstruction(EEPROM_SPI,(uint8_t*)command, 1);

    // Loop as long as the memory is busy with a write cycle
    do {

        while (HAL_SPI_ReceiveH(EEPROM_SPI, (uint8_t*)sEEstatus, 1, 200) == HAL_BUSY_H) {

            HAL_DelayH(1);
        };


        HAL_DelayH(1);
    } while ((sEEstatus[0] & EEPROM_WIP_FLAG) == SET_EEP_H); // Write in progress

    // Deselect the EEPROM: Chip Select high

    HAL_GPIO_WritePinH(EEPROM_PORT_CS, EEPROM_CS_PIN,GPIO_SET_EEPROM);

    return 0;
}

/**
 * @brief Low level function to send header data to EEPROM
 *
 * @param instruction array of bytes to send
 * @param size        data size in bytes
 */
void EEPROM_SPI_SendInstruction(void *EEPROM_SPI,uint8_t *instruction, uint8_t size) {
	#ifdef HAL_DRIVER
    while (((SPI_HandleTypeDef *)EEPROM_SPI)->State == HAL_SPI_STATE_RESET) {

        HAL_DelayH(1);
    }
	#else


	#endif
    if (HAL_SPI_TransmitH(EEPROM_SPI, (uint8_t*)instruction, (uint16_t)size, 200) != HAL_OK_H) {
        Error_Handler();
    }
}

//bloque de portabilidad
//HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
HAL_StatusTypeDef_H HAL_SPI_TransmitH(void *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{

	#ifdef HAL_DRIVER
		HAL_StatusTypeDef_H status_op;
		status_op = (HAL_StatusTypeDef_H)HAL_SPI_Transmit(hspi,pData,Size,Timeout);
		/*
		switch(HAL_SPI_Transmit(hspi,pData,Size,Timeout))
		{
		case 0x00U:
			status_op = HAL_OK_H;
			break;
		case 0x01U:
			status_op = HAL_ERROR_H;
			break;
		case 0x02U:
			status_op = HAL_BUSY_H;
			break;
		case 0x03:
			status_op = HAL_TIMEOUT_H;
			break;
		}*/
		return status_op;
	#else

	#endif
}


HAL_StatusTypeDef_H HAL_SPI_ReceiveH(void *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{

	#ifdef HAL_DRIVER
		HAL_StatusTypeDef_H status_op;
		status_op = (HAL_StatusTypeDef_H)HAL_SPI_Receive(hspi,pData,Size,Timeout);
		return status_op;
	#else

	#endif
}
//HAL_GPIO_WritePin

void HAL_DelayH(uint32_t time)
{
	#ifdef HAL_DRIVER
		HAL_Delay(time);
	#else
		LL_mDelay(time);
	#endif
}
void HAL_GPIO_WritePinH(void *GPIO,uint16_t PIN,int state)
{
	#ifdef HAL_DRIVER
		HAL_GPIO_WritePin(GPIO, PIN,state);//addr
	#else
		if(state == 0) 	LL_GPIO_ResetOutputPin(GPIO,PIN);
		else			LL_GPIO_SetOutputPin(GPIO,PIN);
	#endif
}
