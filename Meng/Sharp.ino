#define DUST_PIN 0
int dustVal = 0;

#define LED_POWER 2
int delayTime = 280;
int delayTime2 = 40;
float offTime = 9680;
void setup()
{
	Serial.begin(9600);
	pinMode(LED_POWER, OUTPUT);
	pinMode(4, OUTPUT);
}

void loop()
{
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
