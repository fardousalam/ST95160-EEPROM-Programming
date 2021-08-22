# ST95160 EEPROM Device Programming

# Motivation of This project:
We often use external memory devices in Embedded project to store some data to it and fetch the data when it is required. In this project ST95160 EEPROM is used as a target memory device. The communication protocol is SPI. As the Stm32 Cube IDE doesnot have built in specific library files to use for this eeprom device programming, spi_eeprom.h & spi_eeprom.c files are written for this project. This library files can be used in future to program any ST M95xxx series EEPROM devices.

# Tasks of this project:
1. Divide the total memory of the EEPROM into different pages. one page contains 32 bytes so total 64 pages. 32x64=2048 Kbytes i.e 16 Kb
2. write one byte at a particular address and read from that particular address.
3. write a block of memory or whole memory and read from the block of memory or whole memory
4. Erase a single memory address or block of memory or whole memory

# Future works:
1. Page and offset addressing
2. Error messages when the EEPROM is not able to communicate or Connect with microcontroller

# Requirements
1. Stm32 nucleo or discovery board
2. stm32 cube IDE
3. ST95160 EEPROM

# Installation
Install the Stm32 cube IDE and Configure the project according to microcontroller.


