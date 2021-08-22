# ST95160 EEPROM Device Programming

# Motivation of This project:
We often use external memory devices in Embedded project to store some data to it and fetch the data when it is required. In this project ST95160 EEPROM is used as a target memory device. The communication protocol is SPI. As the Stm32 Cube IDE doesnot have built in specific library files to use for this eeprom device programming, spi_eeprom.h & spi_eeprom.c files are written for this project. This library files can be used in future to program  
