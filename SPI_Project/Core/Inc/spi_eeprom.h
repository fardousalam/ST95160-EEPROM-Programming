/*
 * spi_eeprom.h
 *
 *  Created on: Jul 21, 2021
 *      Author: f.alam
 */

#ifndef INC_SPI_EEPROM_H_
#define INC_SPI_EEPROM_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
    EEPROM_STATUS_PENDING,
    EEPROM_STATUS_COMPLETE,
    EEPROM_STATUS_ERROR
} EepromOperations;

/* M95160 SPI EEPROM defines */
#define EEPROM_WREN  0x06  /*!< Write Enable */
#define EEPROM_WRDI  0x04  /*!< Write Disable */
#define EEPROM_RDSR  0x05  /*!< Read Status Register */
#define EEPROM_WRSR  0x01  /*!< Write Status Register */
#define EEPROM_READ  0x03  /*!< Read from Memory Array */
#define EEPROM_WRITE 0x02  /*!< Write to Memory Array */

#define EEPROM_WIP_FLAG        0x01  /*!< Write In Progress (WIP) flag */

#define EEPROM_PAGESIZE        32    /*!< Pagesize according to documentation */
#define EEPROM_BUFFER_SIZE     32    /*!< As the page size is 32 so we take the buffer size 32*/
#define EEPROM_PAGE_NUM        64    /* Total number of pages */
#define Max_Size              2048   /*Total allocated memory (32x64)*/

uint8_t  Tx[Max_Size];                    //Transmit
uint8_t  Rx[Max_Size];


#define EEPROM_CS_HIGH()    HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_SET)                  //chip select
#define EEPROM_CS_LOW()     HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_RESET)               // chip deselect


void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi);                                                                 //Spi initialization
void EEwrite_byte(uint16_t StartAddress,uint8_t* data);                                                        // Write one byte
void EEread_byte(uint16_t StartAddress,uint8_t* data);                                                        // read one byte
uint8_t EEwrite_arry(uint16_t StartAddress, uint8_t* data, uint16_t size);                                  // write  to a block of memory
uint8_t EEread_arry(uint16_t StartAddress, uint8_t* data, uint16_t size);                                 // read from a block of memory
uint8_t Erase_memory();                                                                                      // Erase full memory
void EEPROM_ERROR();                                                                                          // Error Handler
uint8_t EEPROM_SPI_WriteBuffer( uint16_t WriteAddr,uint8_t* data, uint16_t NumByteToWrite);

void Write_Enable();                                                                  // Enable write Enable latch
void Write_Disable();                                                                 // Disable the write Enable latch
uint8_t Read_Status_Register();                                                       // Read the status register
uint8_t EEPROM_SPI_WaitStandbyState(void);                                            // Wait untill the Write is in Progress
void Write_Status_Register(uint8_t regval);                                           // write the status register



#ifdef __cplusplus
}
#endif


#endif /* INC_SPI_EEPROM_H_ */
