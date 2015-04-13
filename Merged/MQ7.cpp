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

