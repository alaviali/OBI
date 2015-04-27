// timer interrupt based grove dust sensor program
// timer1 will interrupt at 1Hz

// JST Pin 1 (Black Wire) = > Arduino GND
// JST Pin 3 (Red wire) = > Arduino 5VDC
// JST Pin 4 (Yellow wire) = > Arduino Digital Pin 8
#include <SPI.h>

//toggle variables to monitor timer interrupt status using oscilloscope
boolean toggle1 = 0;

int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 1000;//sample 1s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

//SRAM opcodes
#define RDSR        5
#define WRSR        1
#define READ        3
#define WRITE       2

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

void setup(){

	//set pins as outputs
	Serial.begin(9600);
	SPI.begin();
	pinMode(8, INPUT);
	pinMode(5, OUTPUT);

	cli();//stop interrupts
	
	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1 = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = 62499;// must be <65536, see worklog for calculation
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS12 and CS10 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	
	sei();//enable global interrupts
	
}//end setup

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
	//generates pulse wave of the same fequency as the interrupt, monitor using oscilloscope
	if (toggle1){
		digitalWrite(5, HIGH);
		toggle1 = 0;
	}
	else{
		digitalWrite(5, LOW);
		toggle1 = 1;
	} // THIS PART NEEDS TO BE COMMENTED OUT IN FINAL CODE!

	ratio = lowpulseoccupancy / (sampletime_ms*10.0);  // Integer percentage 0=>100
	concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve

	uint16_t concentration16 = (uint16_t)concentration;
	uint8_t upperh = (uint8_t)(concentration16 >> 8);
	uint8_t lowerh = (uint8_t)(concentration16);
	uint16_t concentration_new = (upperh << 8) | lowerh;

	Serial.print("Concentration: ");
	Serial.println(concentration16);
	Serial.print("\tRepeated: ");
	Serial.println(concentration_new);

	Spi23LC1024Write8(0, upperh);
	Spi23LC1024Write8(1, lowerh);

	uint8_t upperh_new = Spi23LC1024Read8(0);
	uint8_t lowerh_new = Spi23LC1024Read8(1);

	uint16_t concentration_read = (upperh_new << 8) | lowerh_new;
	Serial.print("\t\tRead out: ");
	Serial.println(concentration_read);

	lowpulseoccupancy = 0;
} 

void loop(){
	duration = pulseIn(pin, LOW);
	lowpulseoccupancy = lowpulseoccupancy + duration; 
}