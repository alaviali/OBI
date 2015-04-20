//  VARIABLES
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
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
unsigned long reporttime_ms = 1000; // report every 1s ;
unsigned long sampletime_ms = 20; // sample every 20ms


void setup()
{
	Serial.begin(9600);

	starttime_report = millis(); // reset report time stamp to current time
	starttime_sample = millis(); // reset sample time stamp to current time
}


void loop()
{
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
