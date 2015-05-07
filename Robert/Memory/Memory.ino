#include <SPI.h>

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
// D10 <------> CS
// 5V  <------> VCC
// 5V  <------> HOLD
// 5V  <-10KR-> CS  //  pullup resistor not needed
// GND <------> VSS


#include <SPI.h>

//SRAM opcodes
#define RDSR        5 // not used
#define WRSR        1 // not used
#define READ        3
#define WRITE       2

//PORTB |= (1 << PORTB2); // pull CS high

//Byte transfer functions
uint8_t Spi23LC1024Read8(uint32_t address) {
	uint8_t read_byte;

	PORTB &= ~(1 << PORTB2);        //set SPI_SS low
	SPI.transfer(READ);
	SPI.transfer((uint8_t)(address >> 16) & 0xff);
	SPI.transfer((uint8_t)(address >> 8) & 0xff);
	SPI.transfer((uint8_t)address);
	read_byte = SPI.transfer(0x00);
	PORTB |= (1 << PORTB2);         //set SPI_SS high
	return read_byte;
}

void Spi23LC1024Write8(uint32_t address, uint8_t data_byte) {
	PORTB &= ~(1 << PORTB2);        //set SPI_SS low
	SPI.transfer(WRITE);
	SPI.transfer((uint8_t)(address >> 16) & 0xff);
	SPI.transfer((uint8_t)(address >> 8) & 0xff);
	SPI.transfer((uint8_t)address);
	SPI.transfer(data_byte);
	PORTB |= (1 << PORTB2);         //set SPI_SS high
}

void setup(void) {
       PORTB |= (1 << PORTB2); // pull CS high

	Serial.begin(9600);
        Serial.println("Memory test");
	SPI.begin();


}

void loop() {
	uint32_t i;
	uint8_t value;
 
  	for (i = 0; i<32; i++) {
		Spi23LC1024Write8(i, (uint8_t)i);
		value = Spi23LC1024Read8(i);
		Serial.println(value);
	}

        delay(1000);
}
