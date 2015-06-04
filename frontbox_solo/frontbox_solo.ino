// OBI Captsone 2015
// Jan. 2015 - June 2015
// Team members: 


//  Sensor PINOUTS
//  PINS		ARDUINO		ATMEGA
//  GROVE		D3		5
//  SHARP		A3		26
//  SHARP_LED	        D4		6
//  MQ7			A0		23
//  MQ7_VOLTAGE	        D2		4
//  TEMP		A1		24
//  PULSE		A2		25

// Memory PINOUTS
// Arduino-- SD shield
// D13 <------> SCK
// D12 <------> MISO
// D11 <------> MOSI
// D10 <------> CS
// 5V  <------> 5V
// GND <------> GND

//#define DEBUG 
//#define _GPS_NO_STATS  // prevent statistics collection code from being compiled

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <DHT.h>
#include <RTClib.h>
#include <TinyGPS.h>


// **************** FOR GPS ******************
TinyGPS gps;
float flat=0, flon=0;
unsigned long age;
int gps_year;
byte gps_month, gps_day, gps_hour, gps_minute, gps_second;
bool haveClock=false;

// **************** FOR SD CARD ******************
#define CHIP_SELECT_PIN 10
File data_file;
String file_name;

// **************** FOR GROVE SENSOR ****************
#define GROVE_READ_PIN 3 // Digital PIN 3 ATMEGA 5
unsigned long grove_duration;
unsigned long grove_start_time;
unsigned long grove_stop_time;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;


// **************** FOR SHARP SENSOR ****************
#define SHARP_READ_PIN 3 // Analog PIN 3 ATMEGA 26
int sharp_dust_val = 0;
float sharp_voltage=0;

#define SHARP_LED_PIN 4      // Digital PIN 4 ATMEGA 6


// **************** FOR MQ7 SENSOR ****************
#define CO_PWR_SEL_PIN 2    // Digital PIN 2 ATMEGA PIN 4
#define CO_READ_PIN 0       // Analong PIN 0 ATMEGA PIN 23
#define CO_HEAT_TIME 60000  // 60 seconds heat time
#define CO_READ_TIME 90000  // 90 seconds reat time

unsigned long CO_timer_start;
unsigned long CO_switch_time;
boolean heaterInHighPhase;
unsigned int gasLevel;

// **************** FOR TEMP & HUMIDITY SENSOR ****************

#define DHT_PIN A1          // Analong PIN 1 ATMEGA 24
#define DHTTYPE DHT22       // DHT 22  (AM2302)
DHT dht(DHT_PIN, DHTTYPE);  // create DHT object


// **************** RTC ****************
RTC_DS1307 rtc;
bool haveRTC=false; // if we have time info from gps
DateTime now;

// **************** timer for sensor data collection ****************
unsigned long sensors_read_time_start;
unsigned num_records = 0;
#define SENSORS_READ_TIME  2000    // ms 


void setup()
{
  Serial.begin(9600); // now using for GPS 

  // **************** FOR RTC ****************
  Wire.begin(); // Join I2C bus as a master
  rtc.begin();  // Initialize clock

  // **************** FOR SHARP SENSOR ****************
  pinMode(SHARP_LED_PIN, OUTPUT);

  // **************** FOR GROVE SENSOR ****************
  pinMode(8, INPUT);
  pinMode(13, OUTPUT);
  grove_start_time=millis();

  // **************** FOR MQ7 SENSOR ****************
  pinMode(CO_PWR_SEL_PIN, OUTPUT);
  digitalWrite(CO_PWR_SEL_PIN, LOW);// 5v phase
  heaterInHighPhase = true;
  CO_timer_start = millis();
  CO_switch_time = CO_timer_start + CO_HEAT_TIME;

  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  dht.begin();
  
  // **************** FOR MEMORY ****************
  // must be left as an output or the SD library functions will not work. 
  pinMode(SS, OUTPUT);
  
  // filename will be mm-dd-hh.txt
  DateTime rtc_time=rtc.now();
  file_name = String( rtc_time.month() ) + '-' + String( rtc_time.day() ) + '-' + 
      String( rtc_time.hour() ) + ".txt";
#ifndef DEBUG
  SD.begin(CHIP_SELECT_PIN);
  delay(100); // otherwise open() will fail

  // write header
 
  data_file = SD.open(file_name.c_str(), FILE_WRITE);
  if (data_file) {
    data_file.print("Rec. #\tMM/DD/YYYY\tHH:MM:SS (UTC)\tlatitude\tlongitude\t");
    data_file.println("Sharp\tGrove\tCO read\tCO heat\tHumidity\tTemp (celsius)");
    data_file.close();
  }

 #endif

  // **************** FOR SENOR READING TIMER ****************
  sensors_read_time_start = millis();

} // setup


void loop()
{
  // for gps
  bool newData = false;
#ifdef DEBUG
  unsigned long chars;
  unsigned short sentences, failed;
#endif 
  
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial.available()) {
      char c = Serial.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

    if (newData) {
      gps.f_get_position(&flat, &flon, &age);
      
      if(!haveClock) { // set clock
        gps.crack_datetime(&gps_year, &gps_month, &gps_day, &gps_hour, &gps_minute, &gps_second);
         // adjust UTC FIXME!
         //if ( gps_hour < 7) {  // don't subtract from unsigned types
         //  gps_day -= 1;
         //  gps_hour += 17;     // instead add 24 - 7
         //} else
         //   gps_hour -=7;     // OK to subtract here
         //  Wants years since 1900?
         rtc.adjust( DateTime(gps_year-2000, gps_month, gps_day, gps_hour, gps_minute, gps_second));
         haveClock=true;
      }
    }  
 
 #ifdef DEBUG
 gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  Serial.flush();
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
#endif
 
  // **************** FOR SHARP SENSOR ****************
  // nothing

  // **************** FOR GROVE SENSOR ****************
  // continuous reading
  grove_duration = pulseIn(GROVE_READ_PIN, LOW);
  lowpulseoccupancy += grove_duration;
  grove_stop_time=millis();  
  
  // **************** FOR MQ7 SENSOR ****************
  // nothing

  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  // Be sure to wait 2 seconds between measurements.

  // **************** READ SENSORS ****************
#ifdef DEBUG
    Serial.print("Time since last reading: ");
    Serial.println(millis() - sensors_read_time_start);
    Serial.flush();
#endif

  if( (millis() - sensors_read_time_start) > SENSORS_READ_TIME ) {
    RecordSensors();
    sensors_read_time_start = millis();
  }
    
} // void loop()


void RecordSensors()
{

#ifdef DEBUG
  unsigned long read_start = millis(); // how long does this function take to run?
#endif

  // **************** SENSOR READING STAMP TIME ****************
  now = rtc.now();


  // **************** FOR SHARP SENSOR ****************
  digitalWrite(SHARP_LED_PIN, LOW); // power on the LED
  delayMicroseconds(280);  // wait 280us
  sharp_dust_val = analogRead(SHARP_READ_PIN); // read the dust value via pin 5 on the sensor
  delayMicroseconds(40);  // wait 40us, may not be necessary
  digitalWrite(SHARP_LED_PIN, HIGH); // turn the LED off
  sharp_voltage = sharp_dust_val*4.9/1024;   // convert to voltage
  sharp_voltage = (sharp_voltage - 0.0356) * 120000; // convert to particles per .01 ft^3
  if( sharp_voltage < 0 )
    sharp_voltage = 0;


  // **************** FOR MQ7 SENSOR ****************
  if (millis() > CO_switch_time) {         // time to switch
    if (heaterInHighPhase) { 
      digitalWrite(CO_PWR_SEL_PIN, HIGH);  // 1.4v phase
      heaterInHighPhase = false;
      CO_switch_time = millis() + CO_READ_TIME;
    } else {                               // switch to 5V phase
        digitalWrite(CO_PWR_SEL_PIN, LOW);   // 5v phase
        heaterInHighPhase = true;
        CO_switch_time = millis() + CO_HEAT_TIME;
      }
  }
  // Read gas level regardless
  gasLevel = analogRead(CO_READ_PIN);

  
  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  float h = dht.readHumidity();
  float f = dht.readTemperature(false); // change to true for Farenheit
  if (isnan(h) || isnan(f)) { // Check if any reads failed and set to error value.
    h=255;    f=255;
  }


  // **************** FOR GROVE SENSOR ****************
    // Integer percentage 0=>100
  ratio = lowpulseoccupancy / ((grove_stop_time - grove_start_time) * 10.0);
    // using spec sheet curve, gives particles per 0.01 cubic foot
  concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62;
  lowpulseoccupancy = 0;  // reset
  grove_start_time=millis();


  // ********** KEEP TRACK OF NUMBER OF RECORDS *********
  num_records++;

#ifdef DEBUG
  Serial.print("Time to read sensors(ms): ");
  Serial.println(millis() - read_start);
  Serial.flush();
#endif


#ifdef DEBUG
  // **************** log data to serial output ****************

    Serial.print("Rec. # ");    Serial.print(num_records);  Serial.print('\t');
    
    Serial.print(now.month());  Serial.print('/');
    Serial.print(now.day());    Serial.print('/');
    Serial.print(now.year());   Serial.print('\t');
    Serial.print(now.hour());   Serial.print(':');
    Serial.print(now.minute()); Serial.print(':');
    Serial.print(now.second()); Serial.print('\t');

    Serial.flush();
    
    Serial.print(flat, 6); Serial.print('\t');
    Serial.print(flon, 6); Serial.print('\t');
    
    Serial.flush();
    
    Serial.print("\tSharp: ");         Serial.print(sharp_voltage);
    Serial.print("\tGrove: ");         Serial.print(concentration);
    if(heaterInHighPhase)
      Serial.print("\tCO heat: ");
    else
      Serial.print("\tCO read: ");
    Serial.print(gasLevel);
    Serial.print("\tHumidity: ");      Serial.print(h);
    Serial.print("\tTemp: ");          Serial.println(f);
    
    Serial.flush();
#endif

  
  // **************** WRITE TO SD CARD ****************
#ifdef DEBUG
  Serial.println("Logging to SD");
  Serial.flush();
#endif

#ifndef DEBUG
  data_file = SD.open(file_name.c_str(), FILE_WRITE);

  // record number
  data_file.print(num_records); data_file.print('\t');

  // date and time
  data_file.print(String(now.month()).length()==1 ? String('0') + String(now.month()) : now.month() );
  data_file.print('/');
  data_file.print(String(now.day()).length()==1 ? String('0') + String(now.day()) : now.day() );
  data_file.print('/');
  data_file.print(now.year());
  data_file.print('\t');
  data_file.print(String(now.hour()).length()==1 ? String('0') + String(now.hour()) : now.hour() );
  data_file.print(':');
  data_file.print(String(now.minute()).length()==1 ? String('0') + String(now.minute()) : now.minute() );
  data_file.print(':');
  data_file.print(String(now.second()).length()==1 ? String('0') + String(now.second()) : now.second() );
  data_file.print('\t');

  // gps
  data_file.print(flat,6); data_file.print('\t');
  data_file.print(flon,6); data_file.print('\t');
  // altitude, speed?

  // sharp sensor value
  data_file.print(sharp_voltage); data_file.print('\t');

  //  grove sensor value
  data_file.print(concentration);  data_file.print('\t');

  // MQ7 CO sensor value
   if(heaterInHighPhase) {
    data_file.print("--\t");
    data_file.print(gasLevel);
  } else {
    data_file.print(gasLevel); data_file.print('\t');
    data_file.print("--");
  }
  data_file.print('\t');
  
  
  // DHT22 sensor values
  data_file.print(h);  data_file.print('\t');
  data_file.print(f);  data_file.println();
  data_file.close();
#endif  

  // Reset the grove sample time
  grove_start_time=millis();

} // RecordSensors()

