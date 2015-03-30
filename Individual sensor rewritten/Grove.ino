// timer interrupt based grove dust sensor program
// timer1 will interrupt at 1Hz

// JST Pin 1 (Black Wire) = > Arduino GND
// JST Pin 3 (Red wire) = > Arduino 5VDC
// JST Pin 4 (Yellow wire) = > Arduino Digital Pin 8

//toggle variables to monitor timer interrupt status using oscilliscope
boolean toggle1 = 0;

int pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 1000;//sample 1s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

// mode variable, if 0 then enable interrupt, if 1 disable all interrupts
boolean mode = 0; 

void setup(){

	//set pins as outputs
	Serial.begin(9600);
	pinMode(7, INPUT);
	pinMode(8, INPUT);
	pinMode(13, OUTPUT);
	starttime = millis();//get the current time;

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
	//generates pulse wave of the same fequency as the interrupt, monitor using oscilliscope
	if (toggle1){
		digitalWrite(13, HIGH);
		toggle1 = 0;
	}
	else{
		digitalWrite(13, LOW);
		toggle1 = 1;
	} // THIS PART NEEDS TO BE COMMENTED OUT IN FINAL CODE!

	ratio = lowpulseoccupancy / (sampletime_ms*10.0);  // Integer percentage 0=>100
	concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
	Serial.print("Concentration: ");
	Serial.println(concentration);
	lowpulseoccupancy = 0;
}


void loop(){
	duration = pulseIn(pin, LOW);
	lowpulseoccupancy = lowpulseoccupancy + duration;

	mode = digitalRead(7);

	if (mode == 1) // when mode is 1, rider on, do polling, disable all interrupts
	{
		TIMSK1 = 0; // disable timer1 interrupt
		if ((millis() - starttime) > sampletime_ms)//if the sampel time == 1s
		{
			ratio = lowpulseoccupancy / (sampletime_ms*10.0);  // Integer percentage 0=>100
			concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
			Serial.print("Concentration: ");
			Serial.println(concentration);
			lowpulseoccupancy = 0;
			starttime = millis();
		}
	}
	else // when mode is 0, rider off, enable all interrupts
	{
		TIMSK1 |= (1 << OCIE1A);
	}
}