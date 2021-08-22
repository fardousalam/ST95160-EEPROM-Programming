/*
 * spi_eeprom.c
 *
 *  Created on: Jul 21, 2021
 *      Author: f.alam
 */

#include "spi_eeprom.h"
#include<string.h>

SPI_HandleTypeDef * EEPROM_SPI;
uint8_t EEPROM_StatusByte;



void EEPROM_SPI_INIT(SPI_HandleTypeDef * hspi)
{
    EEPROM_SPI = hspi;
}
void EEPROM_ERROR()
{



}



void EEwrite_byte(uint16_t StartAddress, uint8_t* data)
{
	  while (EEPROM_SPI->State != HAL_SPI_STATE_READY)
		{
		        HAL_Delay(5);
		}

		Write_Enable();
		uint8_t command[3];
		command[0] = EEPROM_WRITE;// Send "Write to Memory" instruction
		command[1] = StartAddress>>8;
		command[2] = StartAddress;
		//command[3] = data[size];
		EEPROM_CS_LOW();
		HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,3,100);
		HAL_SPI_Transmit(EEPROM_SPI,(uint8_t*) data, 1, 100);
	    EEPROM_CS_HIGH();
		EEPROM_SPI_WaitStandbyState();
		Write_Disable();


}

void EEread_byte(uint16_t StartAddress,uint8_t* data)
{
	while (EEPROM_SPI->State != HAL_SPI_STATE_READY)
		    {
			HAL_Delay(5);
		    }
		     uint8_t command[3];
		     command[0] = EEPROM_READ;    // Send "Read from Memory" instruction
		     command[1] = StartAddress >> 8;  // Send 16-bit address
		     command[2] = StartAddress;
		    EEPROM_CS_LOW();// Select the EEPROM: Chip Select low
		    HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,3, 100);
   while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)data, 1 , 100) == HAL_BUSY)
		      {
		    	      HAL_Delay(5);
		       };
		    // Deselect the EEPROM: Chip Select high
		    EEPROM_CS_HIGH();

}

uint8_t EEread_arry(uint16_t StartAddress,uint8_t* data, uint16_t size)
{
	while (EEPROM_SPI->State != HAL_SPI_STATE_READY)
		    {
			HAL_Delay(5);
		    }
		     uint8_t command[3];

		     command[0] = EEPROM_READ;    // Send "Read from Memory" instruction
		     command[1] = StartAddress >> 8;  // Send 16-bit address
		     command[2] = StartAddress;
		    EEPROM_CS_LOW();// Select the EEPROM: Chip Select low
		    HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,3, 100);
   while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)data, size , 100) == HAL_BUSY)
		      {
		    	      HAL_Delay(5);
		       };
		    // Deselect the EEPROM: Chip Select high
		    EEPROM_CS_HIGH();
		    Read_Status_Register();
		    uint8_t Status_read_byte=Read_Status_Register();
		    return Status_read_byte;
}

uint8_t Erase_memory()
{
          uint8_t Er[2048];
          for(int i=0;i<2048;i++)
          {
        	  Er[i]=0xff;
          }

          while (EEPROM_SPI->State != HAL_SPI_STATE_READY)
         		{
         		        HAL_Delay(5);
         		}

         		Write_Enable();
         		uint8_t command[3];
         		command[0] = EEPROM_WRITE;// Send "Write to Memory" instruction
         		command[1] = 0x00>>8;
         		command[2] = 0x00;
         		//command[3] = data[size];
         		EEPROM_CS_LOW();
         		HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,3,100);
         		HAL_SPI_Transmit(EEPROM_SPI,(uint8_t*) Er,2048, 100);
         	    EEPROM_CS_HIGH();
         		EEPROM_SPI_WaitStandbyState();
         		Write_Disable();
          Read_Status_Register();
          uint8_t Erase_status_byte=Read_Status_Register();
          return Erase_status_byte;
}

void Write_Enable()
{
	EEPROM_CS_LOW();                                                          //   Select the EEPROM: Chip Select low
	uint8_t command[1] = { EEPROM_WREN };                                     //  "Write Enable" instruction
	HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,1, 100);                   //  Transmit the instruction
	EEPROM_CS_HIGH();                                                         //  Deselect the EEPROM: Chip Select high
}
void Write_Disable()
{
	EEPROM_CS_LOW();                                                            // Select the EEPROM: Chip Select low
	uint8_t command[1] = { EEPROM_WRDI };                                       //  "Write Disable" instruction
	HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,1, 100);                     // Transmit the instruction
	EEPROM_CS_HIGH();                                                           // Deselect the EEPROM: Chip Select high
}

uint8_t Read_Status_Register()
{
	    uint8_t status;
	    EEPROM_CS_LOW();                                                        // Select the EEPROM: Chip Select low
		uint8_t command[1] = { EEPROM_RDSR };                                   //  "Read Status Register" instruction
		uint8_t spi_buf[1];
		HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,1, 100);                 // Transmit the instruction
		HAL_SPI_Receive(EEPROM_SPI,(uint8_t*)spi_buf,1, 100);                   // Recieve  the value of status register
		status= spi_buf[0];
		EEPROM_CS_HIGH();                                                       // Deselect the EEPROM: Chip Select high
		return status;                                                          // return the value
}

uint8_t EEPROM_SPI_WaitStandbyState(void)
{
    uint8_t sEEstatus[1] = { 0x00 };
    uint8_t command[1] = { EEPROM_RDSR };

    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    // Send "Read Status Register" instruction
    HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,1, 100);

    // Loop as long as the memory is busy with a write cycle
    do {

        while (HAL_SPI_Receive(EEPROM_SPI, (uint8_t*)sEEstatus, 1, 200) == HAL_BUSY) {
        	HAL_Delay(5);
        };

        HAL_Delay(5);

    } while ((sEEstatus[0] & EEPROM_WIP_FLAG) == SET); // Write in progress

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();

    return 0;
}

void Write_Status_Register(uint8_t regval)
{
    uint8_t command[2];

    command[0] = EEPROM_WRSR;
    command[1] = regval;

    // Enable the write access to the EEPROM
    Write_Enable();

    // Select the EEPROM: Chip Select low
    EEPROM_CS_LOW();

    // Send "Write Status Register" instruction
    // and Regval in one packet

    HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,2, 100);

    // Deselect the EEPROM: Chip Select high
    EEPROM_CS_HIGH();

    Write_Disable();

}

uint8_t EEwrite_arry(uint16_t StartAddress, uint8_t* data, uint16_t size)
{
	  while (EEPROM_SPI->State != HAL_SPI_STATE_READY)
		{
		        HAL_Delay(5);
		}

		Write_Enable();
		uint8_t command[3];
		command[0] = EEPROM_WRITE;// Send "Write to Memory" instruction
		command[1] = StartAddress>>8;
		command[2] = StartAddress;
		//command[3] = data[size];
		EEPROM_CS_LOW();
		HAL_SPI_Transmit(EEPROM_SPI, (uint8_t*)command,3,100);
		HAL_SPI_Transmit(EEPROM_SPI,(uint8_t*) data, size, 100);
	    EEPROM_CS_HIGH();
		EEPROM_SPI_WaitStandbyState();
		Write_Disable();
		Read_Status_Register();
		uint8_t Status_write_byte=Read_Status_Register();
		return Status_write_byte;

}

uint8_t EEPROM_SPI_WriteBuffer( uint16_t WriteAddr,uint8_t* data, uint16_t NumByteToWrite)
{
	uint16_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	uint16_t sEE_DataNum = 0;
	Addr = WriteAddr % EEPROM_PAGESIZE;
	count = EEPROM_PAGESIZE - Addr;
	NumOfPage =  NumByteToWrite / EEPROM_PAGESIZE;
    NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;
    if (Addr == 0)

    { /* WriteAddr is EEPROM_PAGESIZE aligned  */
        if (NumOfPage == 0)
               { /* NumByteToWrite < EEPROM_PAGESIZE */
                   sEE_DataNum = NumByteToWrite;
                   EEwrite_arry(WriteAddr, data , sEE_DataNum);
               }
     else
        {
    	 while (NumOfPage--)
    	   {
    		 sEE_DataNum = EEPROM_PAGESIZE;
    		 EEwrite_arry(WriteAddr, data , sEE_DataNum);
    		 WriteAddr +=  EEPROM_PAGESIZE;
    		 data += EEPROM_PAGESIZE;

    	   }

    	 sEE_DataNum = NumOfSingle;
    	 EEwrite_arry(WriteAddr, data , sEE_DataNum);
        }
     }
    else

    {  /* WriteAddr is not EEPROM_PAGESIZE aligned  */
        if (NumOfPage == 0)
           { /* NumByteToWrite < EEPROM_PAGESIZE */


               if (NumOfSingle > count)

               { /* (NumByteToWrite + WriteAddr) > EEPROM_PAGESIZE */
                temp = NumOfSingle - count;
                sEE_DataNum = count;
                EEwrite_arry(WriteAddr, data , sEE_DataNum);
                WriteAddr +=  count;
                data += count;
                sEE_DataNum = temp;
                EEwrite_arry(WriteAddr, data , sEE_DataNum);
               }
                else
                {
                     sEE_DataNum = NumByteToWrite;
                     EEwrite_arry(WriteAddr, data , sEE_DataNum);
                 }
             }

        else
        {
             NumByteToWrite -= count;
        	 NumOfPage =  NumByteToWrite / EEPROM_PAGESIZE;
        	 NumOfSingle = NumByteToWrite % EEPROM_PAGESIZE;
        	 sEE_DataNum = count;
        	 EEwrite_arry(WriteAddr, data , sEE_DataNum);
        	 WriteAddr +=  count;
        	 data+= count;
        	 while (NumOfPage--)
        	   {
        	        sEE_DataNum = EEPROM_PAGESIZE;
        	        EEwrite_arry(WriteAddr, data , sEE_DataNum);
                    WriteAddr +=  EEPROM_PAGESIZE;
                    data += EEPROM_PAGESIZE;
        	    }

        	 if (NumOfSingle != 0)
        	     {
        	         sEE_DataNum = NumOfSingle;
        	         EEwrite_arry(WriteAddr, data , sEE_DataNum);

        	     }

         }




    }

	Read_Status_Register();
	uint8_t Status_write_byte=Read_Status_Register();
	return Status_write_byte;


}





