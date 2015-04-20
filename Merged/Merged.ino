//	PINS		ARDUINO		ATMEGA
//	GROVE		D3			5
//	SHARP		AI3			26
//	SHARP_LED	D4			6
//	MQ7			AI0			23
//	MQ7_VOLTAGE	D2			4
//	TEMP		AI1			24
//	PULSE		AI2			25


//toggle variables to monitor timer interrupt status using oscilloscope
boolean toggle1 = 0;

// **************** FOR GROVE SENSOR ****************
int pin = 3; // Digital PIN 3 ATMEGA 5
unsigned long duration;
unsigned long starttime;
unsigned long grovesampletime_ms = 1000;//sample 1s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

// **************** FOR SHARP SENSOR ****************
#define DUST_PIN 3 // Analog PIN 3 ATMEGA 26
int dustVal = 0;

#define LED_POWER 2 // Digital PIN 4 ATMEGA 6
int delayTime = 280;
int delayTime2 = 40;
float offTime = 480;

// **************** FOR MQ7 SENSOR ****************
#define VOLTAGE_REGULATOR_DIGITAL_OUT_PIN 2 // Digital PIN 2 ATMEGA PIN 4
#define MQ7_ANALOG_IN_PIN 0 // Analong PIN 0 ATMEGA PIN 23

#define MQ7_HEATER_5_V_TIME_MILLIS 60000
#define MQ7_HEATER_1_4_V_TIME_MILLIS 90000

#define GAS_LEVEL_READING_PERIOD_MILLIS 1000

unsigned long startMillis;
unsigned long switchTimeMillis;
boolean heaterInHighPhase;

// **************** FOR TEMP & HUMIDITY SENSOR ****************

#include "DHT.h"
#define DHTPIN A1     // Analong PIN 1 ATMEGA 24
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// **************** FOR PULSE SENSOR ****************

//  VARIABLES
int pulsePin = 2;                 // Analog PIN 2 ATMEGA 25
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

// Variables used in data processing
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, must be seeded! 
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
unsigned long starttime_report;
unsigned long starttime_sample;
unsigned long reporttime_ms = 1000; // report every 1s in case we miss it because of interrupt;
unsigned long sampletime_ms = 20; // sample every 20ms

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

	// **************** FOR PULSE SENSOR ****************
	starttime_report = millis(); // reset report time stamp to current time
	starttime_sample = millis(); // reset sample time stamp to current time

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

	ratio = lowpulseoccupancy / (grovesampletime_ms*10.0);  // Integer percentage 0=>100
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

	// **************** FOR PULSE SENSOR ****************

	if ((millis() - starttime_report) > reporttime_ms)//if the sampel time == 1s
	{
		Serial.print("BPM: ");
		Serial.println(BPM);
		starttime_report = millis();

	}


	if ((millis() - starttime_sample) > sampletime_ms)
	{
		starttime_sample = millis();
		Signal = analogRead(pulsePin);              // read the Pulse Sensor 
		sampleCounter += 2;                         // keep track of the time in mS with this variable
		int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

		//  find the peak and trough of the pulse wave
		if (Signal < thresh && N >(IBI / 5) * 3){       // avoid dichrotic noise by waiting 3/5 of last IBI
			if (Signal < T){                        // T is the trough
				T = Signal;                         // keep track of lowest point in pulse wave 
			}
		}

		if (Signal > thresh && Signal > P){          // thresh condition helps avoid noise
			P = Signal;                             // P is the peak
		}                                        // keep track of highest point in pulse wave

		//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
		// signal surges up in value every time there is a pulse
		if (N > 250){                                   // avoid high frequency noise
			if ((Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3)){
				Pulse = true;                               // set the Pulse flag when we think there is a pulse
				IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
				lastBeatTime = sampleCounter;               // keep track of time for next pulse

				if (secondBeat){                        // if this is the second beat, if secondBeat == TRUE
					secondBeat = false;                  // clear secondBeat flag
					for (int i = 0; i <= 9; i++){             // seed the running total to get a realisitic BPM at startup
						rate[i] = IBI;
					}
				}

				if (firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
					firstBeat = false;                   // clear firstBeat flag
					secondBeat = true;                   // set the second beat flag
					sei();                               // enable interrupts again
					return;                              // IBI value is unreliable so discard it
				}


				// keep a running total of the last 10 IBI values
				word runningTotal = 0;                  // clear the runningTotal variable    

				for (int i = 0; i <= 8; i++){                // shift data in the rate array
					rate[i] = rate[i + 1];                  // and drop the oldest IBI value 
					runningTotal += rate[i];              // add up the 9 oldest IBI values
				}

				rate[9] = IBI;                          // add the latest IBI to the rate array
				runningTotal += rate[9];                // add the latest IBI to runningTotal
				runningTotal /= 10;                     // average the last 10 IBI values 
				BPM = 60000 / runningTotal;               // how many beats can fit into a minute? that's BPM!
				QS = true;                              // set Quantified Self flag 
				// QS FLAG IS NOT CLEARED INSIDE THIS ISR
			}
		}

		if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
			Pulse = false;                         // reset the Pulse flag so we can do it again
			amp = P - T;                           // get amplitude of the pulse wave
			thresh = amp / 2 + T;                    // set thresh at 50% of the amplitude
			P = thresh;                            // reset these for next time
			T = thresh;
		}

		if (N > 2500){                           // if 2.5 seconds go by without a beat
			thresh = 512;                          // set thresh default
			P = 512;                               // set P default
			T = 512;                               // set T default
			lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
			firstBeat = true;                      // set these to avoid noise
			secondBeat = false;                    // when we get the heartbeat back
		}
	}
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

