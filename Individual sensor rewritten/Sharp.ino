//	Sensor Pin		Arduino Pin
// 1	Vled		->	5V (150ohm resistor)
// 2	LED - GND	->	GND
// 3	LED			->	Digital pin 2
// 4	S - GND		->	GND
// 5	Vo			->	Analog pin 0
// 6	Vcc			->	5V

#define DUST_PIN 0
int dustVal = 0;

#define LED_POWER 2
int delayTime = 280;
int delayTime2 = 40;
float offTime = 480;

//toggle variables to monitor timer interrupt status using oscilloscope
boolean toggle1 = 0;

void setup()
{
	Serial.begin(9600);
	pinMode(LED_POWER, OUTPUT);

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

	// ledPower is any digital pin on the arduino connected to Pin 3 on the sensor
	digitalWrite(LED_POWER, LOW); // power on the LED
	delay(delayTime);
	dustVal = analogRead(DUST_PIN); // read the dust value via pin 5 on the sensor
	delay(delayTime2);
	digitalWrite(LED_POWER, HIGH); // turn the LED off
	delay(offTime);

	delay(200);
	Serial.print("Dust value: ");
	Serial.println(dustVal);
}

void loop()
{
	__asm__("nop");
}
