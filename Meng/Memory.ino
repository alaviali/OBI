/*
Used the following components and wire routing:
(1) Arduino Uno
(2) Microchip 23LCV1024
(3) 10K Resistor
*/

// Arduino Pinouts
// 1-4 GND
// 2 MISO ATMEGA PIN 18
//   Arduino Digital PIN 12
// 3 NC
// 5 MOSI ATMEGA PIN 17
//   Arduino Digital PIN 11
// 6 SCK ATMEGA PIN 19
//   Arduino Digital PIN 13
// 7 V_BAT CR2032 
// 8 5V Power

// Wire Routing
// Arduino-- 23LC1024
// D13 <------> SCK
// D12 <------> MISO
// D11 <------> MOSI
// D10 <------> CS, but CS tied to GND here
// 5V  <------> VCC
// 5V  <------> HOLD
// 5V  <-10KR-> CS, but CS tied to  GND here so no resistor needed
// GND <------> VSS


#include <SPI.h>

//SRAM opcodes
#define RDSR        5
#define WRSR        1
#define READ        3
#define WRITE       2

//Byte transfer functions
uint8_t Spi23LC1024Read8(uint32_t address) {
	uint8_t read_byte;

	SPI.transfer(READ);
	SPI.transfer((uint8_t)(address >> 16) & 0xff);
	SPI.transfer((uint8_t)(address >> 8) & 0xff);
	SPI.transfer((uint8_t)address);
	read_byte = SPI.transfer(0x00);
	return read_byte;
}

void Spi23LC1024Write8(uint32_t address, uint8_t data_byte) {
	SPI.transfer(WRITE);
	SPI.transfer((uint8_t)(address >> 16) & 0xff);
	SPI.transfer((uint8_t)(address >> 8) & 0xff);
	SPI.transfer((uint8_t)address);
	SPI.transfer(data_byte);
}

void setup(void) {
	uint32_t i;
	uint8_t value;

	Serial.begin(9600);
	SPI.begin();

	for (i = 0; i<32; i++) {
		Spi23LC1024Write8(i, (uint8_t)i);
		value =  (i);
		Serial.println((uint16_t)value);
	}
}

void loop() {
}
