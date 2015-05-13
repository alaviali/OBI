
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

#define DEBUG // comment out for release!

#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <RTClib.h>

//SRAM opcodes
#define RDSR        5 // not used
#define WRSR        1 // not used
#define READ        3
#define WRITE       2

// Counters for memory module
uint32_t memory_head = 0;
uint32_t memory_tail = 0;

unsigned long num_records=0;

// **************** FOR MEMORY ******************
// Byte transfer functions
// Put these is a .cpp file?!
uint8_t Spi23LC1024Read8(uint32_t address) {
  uint8_t read_byte;

  PORTB &= ~(1 << PORTB2);        //set SPI_SS low
  SPI.transfer(READ);
  SPI.transfer((uint8_t)(address >> 16) & 0xff);
  SPI.transfer((uint8_t)(address >> 8) & 0xff);
  SPI.transfer((uint8_t)address);
  read_byte = SPI.transfer(0x00);
  PORTB |= (1 << PORTB2);         //set SPI_SS high
  return read_byte;
}

void Spi23LC1024Write8(uint32_t address, uint8_t data_byte) {
  PORTB &= ~(1 << PORTB2);        //set SPI_SS low
  SPI.transfer(WRITE);
  SPI.transfer((uint8_t)(address >> 16) & 0xff);
  SPI.transfer((uint8_t)(address >> 8) & 0xff);
  SPI.transfer((uint8_t)address);
  SPI.transfer(data_byte);
  PORTB |= (1 << PORTB2);         //set SPI_SS high
}


// **************** FOR GROVE SENSOR ****************
#define GROVE_READ_PIN 3 // Digital PIN 3 ATMEGA 5
unsigned long grove_duration;
unsigned long grove_sample_time = 1000;//sample 1s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

// **************** FOR SHARP SENSOR ****************
#define SHARP_READ_PIN 3 // Analog PIN 3 ATMEGA 26
int sharp_dust_val = 0;

#define SHARP_LED_PIN 4 // Digital PIN 4 ATMEGA 6
#define SHARP_DELAY_TIME 280 //

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


// **************** FOR PULSE SENSOR ****************
//  VARIABLES
#define PULSE_PIN 2                 // Analog PIN 2 ATMEGA 25
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
volatile int IBI = 600;             // holds the time between beats, must be seeded! 
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
unsigned long pulse_last_sample;
unsigned long pulse_sample_delay = 20; // sample every 20ms

// create a RTC object
RTC_DS1307 rtc;
bool haveRTC=false; // if we have time info from back box

// timer for sensor data collection
unsigned long sensors_read_time_start;
#define SENSORS_READ_TIME  2000    // ms


void setup()
{
  Serial.begin(57600);
  // Default timeout is 1000ms but 9600 baud is 1 byte every 8.3ms
  Serial.setTimeout(10); 

  // **************** FOR RTC ****************
  Wire.begin(); // Join I2C bus as a master
  rtc.begin();  // Make sure clock is running

  // **************** FOR SHARP SENSOR ****************
  pinMode(SHARP_LED_PIN, OUTPUT);

  // **************** FOR GROVE SENSOR ****************
  pinMode(8, INPUT);
  pinMode(13, OUTPUT);

  // **************** FOR MQ7 SENSOR ****************
  pinMode(CO_PWR_SEL_PIN, OUTPUT);
  CO_timer_start = millis();
  turnHeaterHigh();

  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  dht.begin();

  // **************** FOR PULSE SENSOR ****************
  pulse_last_sample = millis(); // reset sample time stamp to current time

  // **************** FOR MEMORY ****************
  PORTB |= (1 << PORTB2); // pull CS high
  SPI.begin();

  // **************** FOR SENOR READING TIMER ****************
  sensors_read_time_start = millis();
  
/*  
  cli();//stop interrupts

  //set timer1 interrupt at 0.25Hz
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
*/

} // setup


void ReadSensors()
//ISR(TIMER1_COMPA_vect)
{
    DateTime now = rtc.now();

#ifdef DEBUG
    unsigned long read_start = millis(); // how long does this function take to run?
    Serial.print(now.month(), DEC);  Serial.print('/');
    Serial.print(now.day(), DEC);    Serial.print('/');
    Serial.print(now.year(), DEC);   Serial.print(' ');
    Serial.print(now.hour(), DEC);   Serial.print(':');
    Serial.print(now.minute(), DEC); Serial.print(':');
    Serial.print(now.second(), DEC);
#endif

  // **************** FOR SHARP SENSOR ****************

  // ledPower is any digital pin on the arduino connected to Pin 3 on the sensor
  digitalWrite(SHARP_LED_PIN, LOW); // power on the LED
  delay(SHARP_DELAY_TIME);  // 280ms
  sharp_dust_val = analogRead(SHARP_READ_PIN); // read the dust value via pin 5 on the sensor
  digitalWrite(SHARP_LED_PIN, HIGH); // turn the LED off

#ifdef DEBUG
  Serial.print(" Sharp value: ");
  Serial.print(sharp_dust_val);
#endif

  // **************** FOR GROVE SENSOR ****************
  ratio = lowpulseoccupancy / (grove_sample_time*10.0);  // Integer percentage 0=>100
  concentration = 1.1*pow(ratio, 3) - 3.8*pow(ratio, 2) + 520 * ratio + 0.62;
  // using spec sheet curve
#ifdef DEBUG
  Serial.print(" Grove Concentration: ");
  Serial.print(concentration);
#endif
  lowpulseoccupancy = 0;  // reset

  // **************** FOR MQ7 SENSOR ****************
  if (heaterInHighPhase) { // 5v phase of cycle. see if need to switch low yet
    if (millis() > CO_switch_time) {
      turnHeaterLow();
    }
  }
  else {    // 1.4v phase of cycle. see if need to switch high yet
    if (millis() > CO_switch_time) {
      turnHeaterHigh();
    }
  }

  readGasLevel(gasLevel); // pass by reference
  
  // **************** FOR TEMP & HUMIDITY SENSOR ****************
  // Reading can take 200 milliseconds, can only be done every 2 seconds
  float h = dht.readHumidity();
  float f = dht.readTemperature(true);
  // Check if any reads failed and set to error value.
  if (isnan(h) || isnan(f)) {
    h=255;    f=255;
  }

#ifdef DEBUG
  Serial.print(" Humidity: "); Serial.print(h);
  Serial.print(" Temp: ");  Serial.print(f);
#endif

  // **************** PULSE SENSOR ****************
#ifdef DEBUG
    Serial.print(" BPM: "); Serial.println(BPM);
    Serial.flush();
#endif

  // **************** STORE IN MEMORY ****************
  
  // date and time
  Spi23LC1024Write8(memory_head++, (uint8_t)now.month());
  Spi23LC1024Write8(memory_head++, (uint8_t)now.day());
  Spi23LC1024Write8(memory_head++, (uint8_t)(now.year()-2000));
  Spi23LC1024Write8(memory_head++, (uint8_t)now.hour());
  Spi23LC1024Write8(memory_head++, (uint8_t)now.minute());
  Spi23LC1024Write8(memory_head++, (uint8_t)now.second());

  
  // split and store sharp sensor value
  uint8_t sharp_dust_val_upper8 = (uint8_t)(sharp_dust_val >> 8);
  uint8_t sharp_dust_val_lower8 = (uint8_t)(sharp_dust_val);
  Spi23LC1024Write8(memory_head++, sharp_dust_val_upper8);
  Spi23LC1024Write8(memory_head++, sharp_dust_val_lower8);

  // split and store grove sensor value
  uint16_t concentration_16 = (uint16_t)concentration;
  uint8_t concentration_upper8 = (uint8_t)(concentration_16 >> 8);
  uint8_t concentration_lower8 = (uint8_t)(concentration_16);
  Spi23LC1024Write8(memory_head++, concentration_upper8);
  Spi23LC1024Write8(memory_head++, concentration_lower8);

  // split and store MQ7 CO sensor value
  uint16_t gasLevel_16 = (uint16_t)gasLevel;
  uint8_t gasLevel_upper8 = (uint8_t)(gasLevel_16 >> 8);
  uint8_t gasLevel_lower8 = (uint8_t)(gasLevel_16);
  Spi23LC1024Write8(memory_head++, gasLevel_upper8);
  Spi23LC1024Write8(memory_head++, gasLevel_lower8);

  // split and store the DHT22 sensor value
  uint8_t humidity_8 = (uint8_t)h;
  uint8_t fahrenheit_8 = (uint8_t)f;
  Spi23LC1024Write8(memory_head++, humidity_8);
  Spi23LC1024Write8(memory_head++, fahrenheit_8);

  // split and store the pulse sensor value
  uint8_t BPM_8 = (uint8_t)BPM;
  Spi23LC1024Write8(memory_head++, BPM_8);

  // Update how many records we have
  num_records++;

#ifdef DEBUG
  Serial.print("Memory head after write: "); Serial.print(memory_head);
  Serial.print(" Num records: "); Serial.println(num_records);
#endif

  
#ifdef DEBUG
  Serial.print("Time to read sensors(ms): ");
  Serial.println(millis() - read_start);
#endif

} //ReadSensors // ISR


void loop()
{
  // **************** CHECK FOR SERIAL DATA ****************
  while( Serial.available() )
    ProcessSerialData();

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

  // **************** FOR PULSE SENSOR ****************
  // should be an interrupt
  if ( (millis() - pulse_last_sample) > pulse_sample_delay) {
    SamplePulse();
    pulse_last_sample = millis();
  }

  // **************** READ SENSORS ****************
 if( (millis() - sensors_read_time_start) > SENSORS_READ_TIME ) {
 #ifdef DEBUG
   Serial.print("Time since last reading: ");
   Serial.println(millis() - sensors_read_time_start);
 #endif
   ReadSensors();
   sensors_read_time_start = millis();
 }

} // void loop()


void ProcessSerialData()
{
// Command bytes are
#define SERIAL_SET_CLOCK 0xAA
#define SERIAL_NUM_RECORD 0xBB
#define SERIAL_REQUEST_RECORD 0XCC
#define SERIAL_ACK  0xEE
#define SERIAL_ERR  0XxFF
  
  int command = Serial.read();
 
    if (command == 'A') { //replace with 0xAA

        if(! rtc.isrunning() ) {
#ifdef DEBUG
            Serial.println("RTC is not running!");
#endif
         }
         byte month = Serial.parseInt();
         byte day = Serial.parseInt();
         byte year = Serial.parseInt();
         byte hour = Serial.parseInt();
         byte minute = Serial.parseInt();
         byte second = Serial.parseInt();
   
#ifdef DEBUG
       Serial.print("Setting rtc time to ");    Serial.print(month); Serial.print("/");
       Serial.print(day); Serial.print("/");    Serial.print(year+2000); Serial.print(" ");
       Serial.print(hour); Serial.print(":");   Serial.print(minute); Serial.print(":");
       Serial.println(second);
#endif
   
       rtc.adjust( DateTime(year+100, month, day, hour, minute, second));
       // TODO: send ack
    } else if (command == 'B') { // replace
       Serial.print(num_records);
    } else if (command == 'C') {  // request record
      if (num_records == 0) 
        SerialSendError();
        else {  // uncompress the bytes
          
          uint8_t month = Spi23LC1024Read8(memory_tail++);
          uint8_t day = Spi23LC1024Read8(memory_tail++);
          uint16_t year = Spi23LC1024Read8(memory_tail++) + 2000;
          uint8_t hour = Spi23LC1024Read8(memory_tail++);
          uint8_t minute = Spi23LC1024Read8(memory_tail++);
          uint8_t second = Spi23LC1024Read8(memory_tail++);
          
          uint16_t sharp_value = Spi23LC1024Read8(memory_tail++); // upper byte
          sharp_value <<= 8;
          sharp_value |= Spi23LC1024Read8(memory_tail++);

          uint16_t grove_value = Spi23LC1024Read8(memory_tail++); // upper byte
          grove_value <<= 8;
          grove_value |= Spi23LC1024Read8(memory_tail++);
          
          uint16_t co_value = Spi23LC1024Read8(memory_tail++); // upper byte
          co_value <<= 8;
          co_value |= Spi23LC1024Read8(memory_tail++);

          uint8_t humidity = Spi23LC1024Read8(memory_tail++);
          uint8_t temp = Spi23LC1024Read8(memory_tail++);
          uint8_t bpm = Spi23LC1024Read8(memory_tail++);
          
          // send the data as text
          Serial.print(month); Serial.print(day); Serial.print(year);
          Serial.print(hour); Seriial.print(minute); Serial.print(second);
          Serial.print(sharp_value); Serial.print(grove_value);
          Serial.print(co_value); Serial.print(humidity);
          Serial.print(temp); Serial.print(bpm);
          Serial.flush();
          
          num_records--;
          
      } // send record

    } else {   // unrecognized command
       SerialSendError();
#ifdef DEBUG
       Serial.print("Undefined command code: ");
       Serial.println(command);
#endif
  }
  
} // ProcessSerialData()


void SerialSendError()
{
  // read out any pending serial data
  while (Serial.available())
    Serial.read();
  // write error code
  Serial.write(0xEE);
  Serial.flush();
}


void SamplePulse()
{
  pulse_last_sample = millis();
  volatile int Signal = analogRead(PULSE_PIN); // read the Pulse Sensor raw data
  sampleCounter += pulse_sample_delay;        // keep track of the time in mS with this variable
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
        for (int i = 0; i <= 9; i++){        // seed the running total to get a realisitic BPM at startup
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
} // SamplePulse()


// **************** FOR MQ7 SENSOR ****************
void turnHeaterHigh(){
  // 5v phase
  digitalWrite(CO_PWR_SEL_PIN, LOW);
  heaterInHighPhase = true;
  CO_switch_time = millis() + CO_HEAT_TIME;
}

void turnHeaterLow(){
  // 1.4v phase
  digitalWrite(CO_PWR_SEL_PIN, HIGH);
  heaterInHighPhase = false;
  CO_switch_time = millis() + CO_READ_TIME;
}

void readGasLevel(unsigned int &gasLevel_temp){
  gasLevel_temp = analogRead(CO_READ_PIN);
  unsigned int time = (millis() - CO_timer_start) / 1000;

  if (heaterInHighPhase)
  {
#ifdef DEBUG
    Serial.print(" CO: ");
    Serial.print(time);
    Serial.print(",");
    Serial.print("-");
#endif
    gasLevel_temp = 0xFFFF;
  }
  else
  {
#ifdef DEBUG
    Serial.print(" CO: ");
    Serial.print(time);
    Serial.print(",");
    Serial.print(gasLevel_temp);
#endif
  }

} // readGasLevel

