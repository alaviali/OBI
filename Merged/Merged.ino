
//toggle variables to monitor timer interrupt status using oscilloscope
boolean toggle1 = 0;

// **************** FOR GROVE SENSOR ****************
int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 1000;//sample 1s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

// **************** FOR SHARP SENSOR ****************
#define DUST_PIN 0
int dustVal = 0;

#define LED_POWER 2
int delayTime = 280;
int delayTime2 = 40;
float offTime = 480;

// **************** FOR MQ7 SENSOR ****************
#define VOLTAGE_REGULATOR_DIGITAL_OUT_PIN 8
#define MQ7_ANALOG_IN_PIN 0

#define MQ7_HEATER_5_V_TIME_MILLIS 60000
#define MQ7_HEATER_1_4_V_TIME_MILLIS 90000

#define GAS_LEVEL_READING_PERIOD_MILLIS 1000

unsigned long startMillis;
unsigned long switchTimeMillis;
boolean heaterInHighPhase;

// **************** FOR TEMP & HUMIDITY SENSOR ****************

#include "DHT.h"
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
	Serial.begin(9600);

	// **************** FOR SHARP SENSOR ****************
	pinMode(LED_POWER, OUTPUT);

	// **************** FOR GROVE SENSOR ****************
	pinMode(8, INPUT);
	pinMode(13, OUTPUT);

	// **************** FOR MQ7 SENSOR ****************
	pinMode(VOLTAGE_REGULATOR_DIGITAL_OUT_PIN, OUTPUT);
	startMillis = millis();
	turnHeaterHigh();

	// **************** FOR TEMP & HUMIDITY SENSOR ****************
	dht.begin();


	cli();//stop interrupts

	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1 = 0;//initialize counter value to 0
	// set compare match register for 4 seconds increments
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

	// **************** FOR SHARP SENSOR ****************

	// ledPower is any digital pin on the arduino connected to Pin 3 on the sensor
	digitalWrite(LED_POWER, LOW); // power on the LED
	delay(delayTime);
	dustVal = analogRead(DUST_PIN); // read the dust value via pin 5 on the sensor
	// delay(delayTime2); // commented out for now
	digitalWrite(LED_POWER, HIGH); // turn the LED off
	//delay(offTime); //  commented out for now

	// delay(200);  // interval between readings, not needed in merged code
	Serial.print("Dust value: ");
	Serial.println(dustVal);

	// **************** FOR GROVE SENSOR ****************

	ratio = lowpulseoccupancy / (sampletime_ms*10.0);  // Integer percentage 0=>100
	concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
	Serial.print("Concentration: ");
	Serial.println(concentration);
	lowpulseoccupancy = 0;

	// **************** FOR MQ7 SENSOR ****************

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
	// delay(GAS_LEVEL_READING_PERIOD_MILLIS); // interval between readings no longer needed

	// **************** FOR TEMP & HUMIDITY SENSOR ****************

	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	float h = dht.readHumidity();
	// Read temperature as Celsius
	float t = dht.readTemperature();
	// Read temperature as Fahrenheit
	float f = dht.readTemperature(true);

	// Check if any reads failed and exit early (to try again).
	if (isnan(h) || isnan(t) || isnan(f)) {
		Serial.println("Failed to read from DHT sensor!");
		return;
	}

	// Compute heat index
	// Must send in temp in Fahrenheit!
	float hi = dht.computeHeatIndex(f, h);

	Serial.print("Humidity: ");
	Serial.print(h);
	Serial.print(" %\t");
	Serial.print("Temperature: ");
	Serial.print(t);
	Serial.print(" *C ");
	Serial.print(f);
	Serial.println(" *F\t");
}

void loop()
{
	// **************** FOR SHARP SENSOR ****************
	// nothing

	// **************** FOR GROVE SENSOR ****************
	duration = pulseIn(pin, LOW);
	lowpulseoccupancy = lowpulseoccupancy + duration;

	// **************** FOR MQ7 SENSOR ****************
	// nothing

	// **************** FOR TEMP & HUMIDITY SENSOR ****************
	// Wait a few seconds between measurements.
	// delay(500); // interval between readings no longer needed  
}


// **************** FOR MQ7 SENSOR ****************
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

