/*
	Arduino code for Sharp dust sensor and Groove dust sensor
	Open source license: MIT
	Meng Lei, lmeng@pdx.edu
	
	Sensors: 	Groove Dust Sensor: PPD42NS
				Sharp Dust Sensor: GP2Y1010AU0F
*/

// ID pins
#define GrovePin 7	// digital pin for grove dust sensor
#define SharLedPin 6 // digital pin for controlling internal LED on Sharp dust sensor

// Setup variables
//	// Grove dust variables
float GroveConc = 999;	// in pcs/0.01cf (of particles >1um)
float GroveRatio = 0;	// Initialize calculated ratio
unsigned long GroveLowCount = 0;	// Number of samples that are low in interval
unsigned long GroveHightCount = 0;	// Number of samples that are high in interval

//	// Sharp dust variables
int SharpVal = 999;	// Analog in from sharp dust sensor (0-5V)

// Initialization
void setup()
{
	Serial.begin(9600);	// Serial port speed; could be 115200
	
	pinMode(GrovePin, INPUT);
	pinMode(SharLedPin, OUTPUT);
	
	analogReference(DEFAULT);
}

void main()
{
	
}