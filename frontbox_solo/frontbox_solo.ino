// OBI Captsone 2015
// Jan. 2015 - June 2015
// Team members: 


//  Sensor PINOUTS
//  PINS		ARDUINO		ATMEGA
//  GROVE		D3			5
//  SHARP		AI3			26
//  SHARP_LED	D4			6
//  MQ7			AI0			23
//  MQ7_VOLTAGE	D2			4
//  TEMP		AI1			24
//  PULSE		AI2			25

// Memory PINOUTS
// 1-4 GND
// 2 MISO ATMEGA PIN 18
//   Arduino Digital PIN 12
// 3 NC
// 5 MOSI ATMEGA PIN 17
//   Arduino Digital PIN 11
// 6 SCK ATMEGA PIN 19
//   Arduino Digital PIN 13
// 7 V_BAT CR2032 
// 8 5V Power
// Wire Routing
// Arduino-- 23LC1024
// D13 <------> SCK
// D12 <------> MISO
// D11 <------> MOSI
// D10 <------> CS
// 5V  <------> VCC
// 5V  <------> HOLD
// 5V  <-10KR-> CS  //  pullup resistor not needed
// GND <------> VSS

//#define DEBUG 

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <DHT.h>
#include <RTClib.h>
#include <TinyGPS.h>


// **************** FOR GPS ******************
TinyGPS gps;
//SoftwareSerial gps_serial(9, 8);
float flat=0, flon=0;
unsigned long age;
int gps_year;
byte gps_month, gps_day, gps_hour, gps_minute, gps_second;
bool haveClock=false;

// **************** FOR SD CARD ******************
#define CHIP_SELECT_PIN 10
File myFile;
String file_name;

// **************** FOR GROVE SENSOR ****************
#define GROVE_READ_PIN 3 // Digital PIN 3 ATMEGA 5
unsigned long grove_duration;
unsigned long grove_sample_time = 1000; //sample 1s ;  FIXME- used in calulation
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

// **************** FOR SHARP SENSOR ****************
#define SHARP_READ_PIN 3 // Analog PIN 3 ATMEGA 26
int sharp_dust_val = 0;

#define SHARP_LED_PIN 4      // Digital PIN 4 ATMEGA 6
#define SHARP_DELAY_TIME 280 

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
unsigned long num_records = 0;
#define SENSORS_READ_TIME  2000    // ms 


void setup()
{
  Serial.begin(9600); // now using for GPS
  //gps_serial.begin(9600);

  // **************** FOR RTC ****************
  Wire.begin(); // Join I2C bus as a master
  rtc.begin();  // Initialize clock

  // **************** FOR SHARP SENSOR ****************
  pinMode(SHARP_LED_PIN, OUTPUT);

  // **************** FOR GROVE SENSOR ****************
  pinMode(8, INPUT);
  pinMode(13, OUTPUT);

  // **************** FOR MQ7 SENSOR ****************
  pinMode(CO_PWR_SEL_PIN, OUTPUT);
  CO_timer_start = millis();
  digitalWrite(CO_PWR_SEL_PIN, LOW);// 5v phase
  heaterInHighPhase = true;
  CO_switch_time = millis() + CO_HEAT_TIME;

  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  dht.begin();
  
  // **************** FOR MEMORY ****************
  // must be left as an output or the SD library functions will not work. 
  pinMode(SS, OUTPUT);
  
  DateTime saved_time=rtc.now();
  file_name = "obi-"+ String(saved_time.month() ) + '-' + String(saved_time.day()) +
      ".txt";
  
  if ( !SD.begin(CHIP_SELECT_PIN) ) 
#ifdef DEGUB
    Serial.println("SD card initialization failed!");
#endif
  delay(100); // otherwise open() will fail
  // write header
  myFile = SD.open(file_name.c_str(), FILE_WRITE);


  if (myFile) {
#ifdef DEBUG
    Serial.println("Writing header to SD");
#endif
    myFile.print("latitude\tlongitude\tDD/MM/YYYY HH:MM:SS");
    myFile.println("\tSharp value\tGrove value\tCO value\tHumidity\tTemp (celsius)");
    myFile.close();
  } else {
#ifdef DEBUG
    Serial.println("Error opening SD file to write header");
#endif
  }

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
  //gps_serial.listen(); // select the GPS softserial port for listening
  
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
         char tmp = gps_hour - 7; // adjust UTC FIXME!
         if (tmp < 0) {
           gps_day -= 1;
           gps_hour = tmp+24;
         }
         //
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
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
#endif
 
  // **************** FOR SHARP SENSOR ****************
  // nothing

  // **************** FOR GROVE SENSOR ****************
  // continuous reading
  grove_duration = pulseIn(GROVE_READ_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + grove_duration;

  // **************** FOR MQ7 SENSOR ****************
  // nothing

  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  // Be sure to wait 2 seconds between measurements.

  // **************** READ SENSORS ****************
#ifdef DEBUG
    Serial.print("Time since last reading: ");
    Serial.println(millis() - sensors_read_time_start);
#endif

  if( (millis() - sensors_read_time_start) > SENSORS_READ_TIME ) {
    ReadSensors();
    sensors_read_time_start = millis();
  }
    
} // void loop()


void ReadSensors()
{

#ifdef DEBUG
  unsigned long read_start = millis(); // how long does this function take to run?
#endif

  // **************** SENSOR READING STAMP TIME ****************
  now = rtc.now();

  // **************** FOR SHARP SENSOR ****************
  digitalWrite(SHARP_LED_PIN, LOW); // power on the LED
  delay(SHARP_DELAY_TIME);  // wait 280ms
  sharp_dust_val = analogRead(SHARP_READ_PIN); // read the dust value via pin 5 on the sensor
  digitalWrite(SHARP_LED_PIN, HIGH); // turn the LED off


  // **************** FOR GROVE SENSOR ****************
  ratio = lowpulseoccupancy / (grove_sample_time*10.0);  // Integer percentage 0=>100
  // using spec sheet curve
  concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62;
  lowpulseoccupancy = 0;  // reset


  // **************** FOR MQ7 SENSOR ****************
  
    if (millis() > CO_switch_time) {  // switch to 1.4V phase
      if (heaterInHighPhase) { 
        digitalWrite(CO_PWR_SEL_PIN, HIGH); // 1.4v phase
        heaterInHighPhase = false;
        CO_switch_time = millis() + CO_READ_TIME;
      } else { // switch to 5V phase
        digitalWrite(CO_PWR_SEL_PIN, LOW);// 5v phase
        heaterInHighPhase = true;
        CO_switch_time = millis() + CO_HEAT_TIME;
      }
  }
  // readGasLevel
  gasLevel = analogRead(CO_READ_PIN);
 // if (heaterInHighPhase)
 //   gasLevel = 0xFFFF;

  
  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  // Reading can take 200 milliseconds, can only be done every 2 seconds
  float h = dht.readHumidity();
  float f = dht.readTemperature(false); // change to true for Farenheit
  if (isnan(h) || isnan(f)) { // Check if any reads failed and set to error value.
    h=255;    f=255;
  }

#ifdef DEBUG
  Serial.print("Time to read sensors(ms): ");
  Serial.println(millis() - read_start);
#endif

  // **************** log data to serial output ****************
  
#ifdef DEBUG

    Serial.print(now.month());  Serial.print('/');
    Serial.print(now.day());    Serial.print('/');
    Serial.print(now.year());   Serial.print(' ');
    Serial.print(now.hour());   Serial.print(':');
    Serial.print(now.minute()); Serial.print(':');
    Serial.print(now.second()); Serial.print('\t');

    Serial.print(flat, 6); Serial.print(" ,");
    Serial.print(flon, 6); Serial.print(' ');
    
    Serial.print("\tSharp value: ");         Serial.print(sharp_dust_val);
    Serial.print("\tGrove Concentration: "); Serial.print(concentration);
    Serial.print("\tCO: ");                  Serial.print(gasLevel);
    Serial.print("\tHumidity: ");            Serial.print(h);
    Serial.print("\tTemp: ");                Serial.println(f);
    Serial.flush();
#endif

  
  // **************** WRITE TO SD CARD ****************
#ifdef DEBUG
  Serial.println("Logging a record to SD"); 
#endif

  myFile = SD.open(file_name.c_str(), FILE_WRITE);

#ifdef DEBUG 
  if (myFile) {
    Serial.println("File open");
  } else {
    Serial.println("Error opening SD file. Record not logged.");
    return;
  }
#endif

  // gps
  myFile.print(flat,6); myFile.print('\t');
  myFile.print(flon,6); myFile.print('\t');
  // altitude, speed?
  
  // date and time
  myFile.print(String(now.month()).length()==1 ? String('0') + String(now.month()) : now.month() );
  myFile.print('/');
  myFile.print(String(now.day()).length()==1 ? String('0') + String(now.day()) : now.day() );
  myFile.print('/');
  myFile.print(now.year());
  myFile.print(' ');
  myFile.print(String(now.hour()).length()==1 ? String('0') + String(now.hour()) : now.hour() );
  myFile.print(':');
  myFile.print(String(now.minute()).length()==1 ? String('0') + String(now.minute()) : now.minute() );
  myFile.print(':');
  myFile.print(String(now.second()).length()==1 ? String('0') + String(now.second()) : now.second() );
  myFile.print('\t');
  
  // sharp sensor value
  myFile.print(sharp_dust_val); myFile.print('\t');

  //  grove sensor value
  myFile.print(concentration);  myFile.print('\t');

  // MQ7 CO sensor value
  myFile.print(gasLevel);
  if(heaterInHighPhase)
    myFile.print(" (H)");
  myFile.print('\t');
  
  
  // DHT22 sensor value
  myFile.print(h);  myFile.print('\t');
  myFile.print(f);  myFile.println();
  myFile.close();

  // Update how many records we have
  num_records++;

#ifdef DEBUG
  Serial.print(" Num records: "); Serial.println(num_records);
#endif

} //ReadSensors // ISR

