#define VOLTAGE_REGULATOR_DIGITAL_OUT_PIN 8
#define MQ7_ANALOG_IN_PIN 0

#define MQ7_HEATER_5_V_TIME_MILLIS 60000
#define MQ7_HEATER_1_4_V_TIME_MILLIS 90000

#define GAS_LEVEL_READING_PERIOD_MILLIS 1000

unsigned long startMillis;
unsigned long switchTimeMillis;
boolean heaterInHighPhase;

//toggle variables to monitor timer interrupt status using oscilloscope
boolean toggle1 = 0;

void setup(){
	Serial.begin(9600);

	pinMode(VOLTAGE_REGULATOR_DIGITAL_OUT_PIN, OUTPUT);

	startMillis = millis();

	turnHeaterHigh();

	Serial.println("Elapsed Time (s), Gas Level");

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
}

ISR(TIMER1_COMPA_vect)
{
	//generates pulse wave of the same fequency as the interrupt, monitor using oscilloscope
	if (toggle1){
		digitalWrite(13, HIGH);
		toggle1 = 0;
	}
	else{
		digitalWrite(13, LOW);
		toggle1 = 1;
	} // THIS PART NEEDS TO BE DELETED IN FINAL CODE!

	if (heaterInHighPhase){
		// 5v phase of cycle. see if need to switch low yet
		if (millis() > switchTimeMillis) {
			turnHeaterLow();
		}
	}
	else {
		// 1.4v phase of cycle. see if need to switch high yet
		if (millis() > switchTimeMillis) {
			turnHeaterHigh();
		}
	}

	readGasLevel();
	delay(GAS_LEVEL_READING_PERIOD_MILLIS);
}

void loop(){
	__asm__("nop");
}

void turnHeaterHigh(){
	// 5v phase
	digitalWrite(VOLTAGE_REGULATOR_DIGITAL_OUT_PIN, LOW);
	heaterInHighPhase = true;
	switchTimeMillis = millis() + MQ7_HEATER_5_V_TIME_MILLIS;
}

void turnHeaterLow(){
	// 1.4v phase
	digitalWrite(VOLTAGE_REGULATOR_DIGITAL_OUT_PIN, HIGH);
	heaterInHighPhase = false;
	switchTimeMillis = millis() + MQ7_HEATER_1_4_V_TIME_MILLIS;
}

void readGasLevel(){
	unsigned int gasLevel = analogRead(MQ7_ANALOG_IN_PIN);
	unsigned int time = (millis() - startMillis) / 1000;

	if (heaterInHighPhase)
	{
		Serial.print(time);
		Serial.print(",");
		Serial.println("-");
	}
	else
	{
		Serial.print(time);
		Serial.print(",");
		Serial.println(gasLevel);
	}
	
}


