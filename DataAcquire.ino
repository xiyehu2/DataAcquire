/***
 * This code is a data acquisition program for the research project in PHYS398, UIUC FA2018.
 * This code is largely based on multifle test codes written by Professor George Gollion.
 * 
 * Simon Hu, Qier An, Charlie Xiao
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SPI.h>
#include "SdFat.h"
#include <Keypad.h>
#include <LiquidCrystal.h>

//BME variables
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

/////////////////////////// Anemometer Parameters ///////////////////////////
const int analogInPin = A6;
int sensorValue = 0;
float sensorVoltage = 0.0;
const float voltageConversionConstant = 0.004882814;
float windSpeed;

/////////////////////////// LCD parameters ///////////////////////////
// The LCD is a GlobalFontz 16 x 2 device.
// initialize the LCD library by associating LCD interface pins
// with the arduino pins to which they are connected. first define
// the pin numbers: reset (rs), enable (en), etc.
const int rs = 12, en = 11, d4 = 36, d5 = 34, d6 = 32, d7 = 30;

// instantiate an LCD object named "lcd" now. We can use its class
// functions to do good stuff, e.g. lcd.print("the text").
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// total number of characters in one line of the LCD display
#define LCDWIDTH 16

/////////////////////////// SD card parameters ///////////////////////////
// "#define" is an instruction to the compiler. In this case, it will
// replace all occurrences of "SD_CS_PIN" with "SS", whose value is
// already known to the compiler.
#define SD_CS_PIN SS
// create a file object to which I'll write data
File myFile;
// file name
char filename[ ] = "bme_and_time_data.txt";
// instantiate a file system object
SdFat SD;
// used for file writing purposes, not related to the realtime clock.
int timeCounter;
char truncate[64];

/////////////////////////// Keypad parameters ///////////////////////////
// our keypad has four rows and three columns. since this will never change
// while the program is runniong, declare these as constants so that they 
// will live in flash (program code) memory instead of the much smaller
// SRAM. 
const byte ROWS = 4; 
const byte COLS = 3;
// keypad layout
char keys[ROWS][COLS] = {
{'1','2','3'},
{'4','5','6'},
{'7','8','9'},
{'*','0','#'}
};
// looking down on the keyboard from above (the side with the keys), the pins are
// numbered 1 - 8, going from left to right, though 8 is not used. 
// Since I am using an Arduino Mega 2560 with a number of breakout boards, 
// I have the following Arduino pin assignments in order to allow the column pins to 
// generate interrupts in some future version of this program.
byte Arduino_colPins[COLS] = {2, 3, 18}; 
byte Arduino_rowPins[ROWS] = {31, 33, 35, 37};

// now instantiate a Keypad object, call it kpd. Also map its pins.
Keypad kpd = Keypad( makeKeymap(keys), Arduino_rowPins, Arduino_colPins, ROWS, COLS );

// character returned when I query the keypad (might be empty)
char query_keypad_char = NO_KEY;

// last non-null character read from keypad
char last_key_char = NO_KEY;

////////////////////////// DS3231 real time clock parameters ////////////////////////
// this is an I2C device on an Adafruit breakout board. It is a cute little thing 
// with a backup battery.

#include "RTClib.h"

// instantiate a real time clock object named "rtc":
RTC_DS3231 rtc;
// instantiate a Date Time object named now
DateTime now;

// names of the days of the week:
char daysOfTheWeek[7][4] = 
  {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

// declare type for a few RTC-related global variables
int RTC_second;
int RTC_day_of_month;
int RTC_day, RTC_month, RTC_year;
int GPS_day, GPS_month, GPS_year;
int GPS_minute, GPS_second, GPS_millisecond;
bool GPS_got_satellites;
int RTC_hour, RTC_minute, RTC_seconds, RTC_milliseconds;

// used for time interval calculatino for measurement
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 100;

// have we already set the RTC clock from the GPS?
bool already_set_RTC_from_GPS;

int GPS_PPS_pin = 43;

   
/////////////////////////////// GPS parameters /////////////////////////////
// Set GPSECHO1 and GPSECHO2 to 'false' to turn off echoing the GPS data to the Serial console
// that are useful for debugging. Set GPSECHO3 to 'false' to prevent final navigation result 
// from being printed to monitor. Set one or more to 'true' if you want to debug and listen 
//to the raw GPS sentences. GPSECHO1 yields more verbose echoing than GPSECHO2.
#define GPSECHO1 false
#define GPSECHO2 false
#define GPSECHO3 false

// get the header file in which lots of things are defined:
#include <Adafruit_GPS.h>

// also define some more stuff relating to update rates. See 
// https://blogs.fsfe.org/t.kandler/2013/11/17/set-gps-update-
// rate-on-arduino-uno-adafruit-ultimate-gps-logger-shield/
#define PMTK_SET_NMEA_UPDATE_10SEC "$PMTK220,10000*2F"
#define PMTK_SET_NMEA_UPDATE_5SEC "$PMTK220,5000*2F"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_2HZ  "$PMTK220,500*2B"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"

// let's use the Arduino's second serial port to communicate with the GPS device.
#define GPSSerial Serial2

// Connect to the GPS via the Arduino's hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// we don't expect a valid GPS "sentence" to be longer than this...
#define GPSMAXLENGTH 120
// or shorter than this:
#define GPSMINLENGTH 55

// declare variables which we'll use to store the value (0 or 1). 
int GPS_PPS_value, GPS_PPS_value_old;

// define some time variables.
unsigned long  time_ms_GPS_PPS_last_changed, time_ms_new_GPS_available;
unsigned long  time_ms_parsable_GPS_available;
unsigned long  time_ms_finished_parsing_GPS_sentence;
unsigned long  time_ms_GPS_PPS_last_became_1;
unsigned long  time_ms_bumped_RTC_time_ready;
unsigned long  time_ms_returned_from_RTC_setting;

// keep track of whether or not we have set the RTC using satellite-informed GPS data
bool good_RTC_time_from_GPS_and_satellites;

// flag when we have set the RTC using the GPS clock, but without considering whether
// the GPS has just updated its own clock from the satellite system
bool good_RTC_time_from_GPS_not_considering_satellites;

// flag to see if we've read a time from the GPS clock
bool have_read_GPS_clock;

// last sentence read from the GPS:
char* GPS_sentence;

// we'll also want to convert the character array to a string for convenience
String GPS_sentence_string;

String GPS_command;

// pointers into parts of a GPRMC GPS data sentence:

const int GPRMC_hour_index1 = 8;
const int GPRMC_hour_index2 = GPRMC_hour_index1 + 2;

const int GPRMC_minutes_index1 = GPRMC_hour_index2;
const int GPRMC_minutes_index2 = GPRMC_minutes_index1 + 2;
      
const int GPRMC_seconds_index1 = GPRMC_minutes_index2;
const int GPRMC_seconds_index2 = GPRMC_seconds_index1 + 2;
      
const int GPRMC_milliseconds_index1 = GPRMC_seconds_index2 + 1;   // skip the decimal point
const int GPRMC_milliseconds_index2 = GPRMC_milliseconds_index1 + 3;
      
const int GPRMC_AV_code_index1 = 19;
const int GPRMC_AV_code_index2 = GPRMC_AV_code_index1 + 1;
      
const int GPRMC_latitude_1_index1 = 21;
const int GPRMC_latitude_1_index2 = GPRMC_latitude_1_index1 + 4;
      
const int GPRMC_latitude_2_index1 = GPRMC_latitude_1_index2 + 1;   // skip the decimal point
const int GPRMC_latitude_2_index2 = GPRMC_latitude_2_index1 + 4;

const int GPRMC_latitude_NS_index1 = 31;
const int GPRMC_latitude_NS_index2 = GPRMC_latitude_NS_index1 + 1;

const int GPRMC_longitude_1_index1 = 33;
const int GPRMC_longitude_1_index2 = GPRMC_longitude_1_index1 + 5;    // 0 - 180 so we need an extra digit
      
const int GPRMC_longitude_2_index1 = GPRMC_longitude_1_index2 + 1;   // skip the decimal point
const int GPRMC_longitude_2_index2 = GPRMC_longitude_2_index1 + 4;
      
const int GPRMC_longitude_EW_index1 = 44;
const int GPRMC_longitude_EW_index2 = GPRMC_longitude_EW_index1 + 1;

// pointers into a GPGGA GPS data sentence:

const int GPGGA_hour_index1 = 8;
const int GPGGA_hour_index2 = GPGGA_hour_index1 + 2;

const int GPGGA_minutes_index1 = GPGGA_hour_index2;
const int GPGGA_minutes_index2 = GPGGA_minutes_index1 + 2;
      
const int GPGGA_seconds_index1 = GPGGA_minutes_index2;
const int GPGGA_seconds_index2 = GPGGA_seconds_index1 + 2;
      
const int GPGGA_milliseconds_index1 = GPGGA_seconds_index2 + 1;   // skip the decimal point
const int GPGGA_milliseconds_index2 = GPGGA_milliseconds_index1 + 3;
      
const int GPGGA_latitude_1_index1 = 19;
const int GPGGA_latitude_1_index2 = GPGGA_latitude_1_index1 + 4;
      
const int GPGGA_latitude_2_index1 = GPGGA_latitude_1_index2 + 1;   // skip the decimal point
const int GPGGA_latitude_2_index2 = GPGGA_latitude_2_index1 + 4;

const int GPGGA_latitude_NS_index1 = 29;
const int GPGGA_latitude_NS_index2 = GPGGA_latitude_NS_index1 + 1;

const int GPGGA_longitude_1_index1 = 31;
const int GPGGA_longitude_1_index2 = GPGGA_longitude_1_index1 + 5;    // 0 - 180 so we need an extra digit
      
const int GPGGA_longitude_2_index1 = GPGGA_longitude_1_index2 + 1;   // skip the decimal point
const int GPGGA_longitude_2_index2 = GPGGA_longitude_2_index1 + 4;
      
const int GPGGA_longitude_EW_index1 = 42;
const int GPGGA_longitude_EW_index2 = GPGGA_longitude_EW_index1 + 1;

const int GPGGA_fix_quality_index1 = 44;
const int GPGGA_fix_quality_index2 = GPGGA_fix_quality_index1 + 1;

const int GPGGA_satellites_index1 = 46;
const int GPGGA_satellites_index2 = GPGGA_satellites_index1 + 2;

// keep track of how many times we've read a character from the GPS device. 
long GPS_char_reads = 0;

// bail out if we exceed the following number of attempts. when set to 1,000,000 this corresponds
// to about 6 seconds. we need to do this to keep an unresponsive GPS device from hanging the program.
const long GPS_char_reads_maximum = 1000000;

// define some of the (self-explanatory) GPS data variables. Times/dates are UTC.
String GPS_hour_string;
String GPS_minutes_string;
String GPS_seconds_string;
String GPS_milliseconds_string;
int GPS_hour;
int GPS_minutes;
int GPS_seconds;
int GPS_milliseconds;

// this one tells us about data validity: A is good, V is invalid.
String GPS_AV_code_string;

// latitude data
String GPS_latitude_1_string;
String GPS_latitude_2_string;
String GPS_latitude_NS_string;
int GPS_latitude_1;
int GPS_latitude_2;

// longitude data
String GPS_longitude_1_string;
String GPS_longitude_2_string;
String GPS_longitude_EW_string;
int GPS_longitude_1;
int GPS_longitude_2;

// velocity information; speed is in knots! 
String GPS_speed_knots_string;
String GPS_direction_string;
float GPS_speed_knots;
float GPS_direction;

String GPS_date_string;

String GPS_fix_quality_string;
String GPS_satellites_string;
int GPS_fix_quality;
int GPS_satellites;

String GPS_altitude_string;
float GPS_altitude;
int GPS_ready = -1;
//end of GPS variables.

//Personal variables.
int testInterval;
char the_key;

// a diagnostic print flag
bool debug_echo = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

 /////////////////////////// LCD setup ////////////////////////////
  // set up the LCD's number of columns and rows, then print a message:
  lcd.begin(16, 2);
  LCD_message("LCD setup","");
  Serial.println("LCD done");

  // delay a bit so I have time to see the display.
  delay(1000);
  
////////////////////////////////// GPS and RTC setup //////////////////////////////

  // fire up the DS3231 real time clock breakout board.
  rtc.begin();
  
  // the following line sets the RTC to the date & time this sketch was compiled.
  // It'll lag behind the true (local) time by a few seconds. Uncomment it if you
  // want to do this. (This program sets the RTC to UTC.)
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // fire up the serial monitor
  Serial.begin(115200);

  // initialize some time variables
  GPS_hour = 0;
  GPS_minute = 0;
  GPS_seconds = 0;
  GPS_milliseconds = 0;
  GPS_day = 0;
  GPS_month = 0;
  GPS_year = 0;
  GPS_got_satellites = false;

  RTC_hour = 0;
  RTC_minute = 0;
  RTC_seconds = 0;
  RTC_milliseconds = 0;
  RTC_day = 0;
  RTC_month = 0;
  RTC_year = 0;

  // initialize whether or not we have set the RTC using satellite-informed GPS data
  good_RTC_time_from_GPS_and_satellites = false;

// initialize whether or not we have set the RTC using the GPS clock, but without considering whether
// the GPS has just updated its own click from the satellite system
  good_RTC_time_from_GPS_not_considering_satellites = false;

  // initialize a flag indicating if we've read a time from the GPS clock
  have_read_GPS_clock = false;

  // declare the GPS PPS pin to be an Arduino input 
  pinMode(GPS_PPS_pin, INPUT);

  // read the GPS PPS pin status.
  GPS_PPS_value_old = digitalRead(GPS_PPS_pin);
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's. Note that this 
  // is different from the baud rate for writing to your laptop's serial monitor window.
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // Set the update rate to once per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Now looking for ");
  lcd.setCursor(0, 1);
  lcd.print("GPS satellites  ");

  // print a message to the serial monitor
  DateTime now = rtc.now();

  LCD_message("RTC setup done","");
  Serial.println("RTC done");
  delay(500);
  //end of clock set up  

/////////////////////////// BME setup /////////////////////////
  /*if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    LCD_message("BME problem", "# to restart");
    delay(100);
    while(true) {
      if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {
        the_key =  kpd.key[0].kchar;
        if (the_key == '#') {
          setup();
          break;
        }
      }
    }
  }*/
  bme.begin();
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(0,0); // 320*C for 150 ms
  LCD_message("BME setup done","");
  Serial.println("BME done");

  delay(500);
  //end of BME setup

/////////////////////////// SD Card setup /////////////////////////
  // Open serial communications and wait for port to open:
  while (!Serial) { }
  // push a message to the serial monitor window
  // fire up the SD file system object and check that all is fine so far.
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD initialization failed!");
    LCD_message("Plug in SD!","");
    // delay a bit to give the serial port time, then bail out...
    delay(100);
    while(true) {
      if(SD.begin(SD_CS_PIN)) {
        break;
      }
    }
  }
  Serial.println("SD initialization done.");
  LCD_message("SD done","");

  // if the SD file exists already, delete it.
  if(SD.exists(filename)) {
    SD.remove(filename); 
        
    // wait a bit just to make sure we're finished with the file remove
    delay(100);   
  }

  // open the new file then check that it opened properly.
  myFile = SD.open(filename, O_CREAT | O_WRITE);
 
  if (myFile) {
    Serial.print("Writing to "); Serial.print(filename); Serial.println("...");
    myFile.println("Data written below has such format:");
    myFile.println("hour(UTC), minute, second, millisecond");
    myFile.println("Temperature(°C), pressure(hPa), humidity(%), altitude(m)");   
    myFile.println();

  } else {
       
    // if the file didn't open, print an error:
    Serial.print("error opening "); Serial.println(filename); 

    // delay a bit to give the serial port time to display the message...
    delay(100);
  }
  //end of SD setup
  LCD_message("Press #","to Start");
  Serial.println("Press # to start");
  //Start the measurement on '#' press.
  while (true) {
    if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {
      the_key =  kpd.key[0].kchar;
      if (the_key == '#') {
        //GPS_query();
        //rtc.adjust(DateTime(RTC_year, RTC_month, RTC_day_of_month,
                            //GPS_hour, GPS_minutes, GPS_seconds));
        //timeCounter = GPS_milliseconds;
        //delay(1000-timeCounter);
        timeCounter = 0;
        startMillis = millis();
        LCD_message("Measuring...","");
        Serial.println("measurement started");
        break;
      }
    }
  }


}

void loop() {

    // self-explanatory
  if (!bme.performReading()) {
    myFile.println("Failed to perform reading :(");
    Serial.println("Failed to perform reading :(");
    myFile.close();
    LCD_message("BME problem", "# to restart");
    delay(100);
    while(true) {
      if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {
        the_key =  kpd.key[0].kchar;
        if (the_key == '#') {
          setup();
          break;
        }
      }
    }
  }

  if (good_RTC_time_from_GPS_and_satellites) {

    //Serial.println(">>=-> Satellite-informed GPS clock loaded into RTC so we're done.");

    //Serial.print("Delay between GPS PPS 0 -> 1 transition and RTC accepting new setting (ms) = ");
    //Serial.println(time_ms_returned_from_RTC_setting - time_ms_GPS_PPS_last_became_1, DEC);

    //Serial.print("Current RTC reading (UTC): ");

    DateTime now = rtc.now();
    /*Serial.print(now.year(), DEC);
    Serial.print('/');
    if(now.month() < 10) Serial.print(0);
    Serial.print(now.month(), DEC);
    Serial.print('/');
    if(now.day() < 10) Serial.print(0);
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    if(now.hour() < 10) Serial.print(0);
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    if(now.minute() < 10) Serial.print(0);
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    if(now.second() < 10) Serial.print(0);
    Serial.print(now.second(), DEC);
    Serial.println();*/

    // set the LCD cursor to column 0, line 0 and then display the latitude/longitude.
    /*lcd.setCursor(0, 0);
    lcd.print("!Lat "), lcd.print(GPS.latitude, 4); lcd.print(" "); lcd.print(GPS.lat); 
    lcd.setCursor(0, 1);
    lcd.print("!Lon "), lcd.print(GPS.longitude, 4); lcd.print(" "); lcd.print(GPS.lon);

    delay(1000);

    lcd.setCursor(0, 0);
    lcd.print(now.year(), DEC);
    lcd.print('/');
    if(now.month() < 10) lcd.print(0);
    lcd.print(now.month(), DEC);
    lcd.print('/');
    if(now.day() < 10) lcd.print(0);
    lcd.print(now.day(), DEC);
    lcd.print(" ");
    lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);

    lcd.setCursor(0, 1);
    if(now.hour() < 10) lcd.print(0);
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    if(now.minute() < 10) lcd.print(0);
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    if(now.second() < 10) lcd.print(0);
    lcd.print(now.second(), DEC);
    lcd.print(" UTC           ");

    while (good_RTC_time_from_GPS_and_satellites) {

      // let's just keep reading the RTC and displaying it to the LCD.
      DateTime now = rtc.now();

      lcd.setCursor(0, 0);
      lcd.print(now.year(), DEC);
      lcd.print('/');
      if(now.month() < 10) lcd.print(0);
      lcd.print(now.month(), DEC);
      lcd.print('/');
      if(now.day() < 10) lcd.print(0);
      lcd.print(now.day(), DEC);
      lcd.print(" ");
      lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
  
      lcd.setCursor(0, 1);
      if(now.hour() < 10) lcd.print(0);
      lcd.print(now.hour(), DEC);
      lcd.print(':');
      if(now.minute() < 10) lcd.print(0);
      lcd.print(now.minute(), DEC);
      lcd.print(':');
      if(now.second() < 10) lcd.print(0);
      lcd.print(now.second(), DEC);
      lcd.print(" UTC           ");
  
      delay(25);      
    
      }*/
  }

  // we get to here if we haven't already set the RTC to a satellite-informed GPS clock.
  
  // read and report the GPS PPS pin status.
  GPS_PPS_value = digitalRead(GPS_PPS_pin);

  if (GPS_PPS_value != GPS_PPS_value_old) {

    // here when the PPS pin has changed value.
    time_ms_GPS_PPS_last_changed = millis();

    // update the PPS value...
    GPS_PPS_value_old = GPS_PPS_value;

    // if the PPS pin has become 1, the GPS clock has just advanced to the next second.
    
    if (GPS_PPS_value == 1) {

      // The GPS clock values obtained the last time a full "sentence" was received will 
      // have arrived BEFORE the PPS pin changed state just now. 

      time_ms_GPS_PPS_last_became_1 = time_ms_GPS_PPS_last_changed;

      // now set the real time clock to the bumped-by-one-second value that we have already calculated.
      // See the  code a few dozen lines below. To set the RTC with an explicit date & time, for example 
      // January 21, 2014 at 3am you would call
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
      
      rtc.adjust(DateTime(2000 + RTC_year, RTC_month, RTC_day, RTC_hour, RTC_minute, RTC_seconds));

      // take note of when we're back from setting the real time clock:
      time_ms_returned_from_RTC_setting = millis();

      // also set a pair of flags...
      good_RTC_time_from_GPS_not_considering_satellites = true;
      good_RTC_time_from_GPS_and_satellites = GPS_got_satellites;
      
      if (debug_echo) {
        Serial.print("\nGPS_PPS_value just changed to 1 at t = ");
        Serial.println(time_ms_GPS_PPS_last_became_1); 

        Serial.print("Previous GPS time parsed from a data sentence: ");
        Serial.print(GPS_hour, DEC); Serial.print(':');
        Serial.print(GPS_minute, DEC); Serial.print(':');
        Serial.print(GPS_seconds, DEC); Serial.print('.');
        Serial.print(GPS_milliseconds);
        
        Serial.print("   Date (dd/mm/yyyy): ");
        Serial.print(GPS_day, DEC); Serial.print('/');
        Serial.print(GPS_month, DEC); Serial.print("/20");
        Serial.println(GPS_year, DEC);

        // now read the RTC value to check that the DS3231 took it:
        DateTime now = rtc.now();
      
        Serial.println("Just set the RTC. Now read it back to make sure it took it.");
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(" (");
        Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
        Serial.print(") ");
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println();
  
        Serial.print("time between GPS_PPS 0 -> 1 and return from RTC setting (ms) = ");
        Serial.println(time_ms_returned_from_RTC_setting - time_ms_GPS_PPS_last_became_1, DEC);

      }
    }
  }

  // *******************************************************************************
  // we're now finished dealing with the GPS PPS pin changing state. 
  // *******************************************************************************

  // *******************************************************************************
  // read data from the GPS; do this one character per pass through function loop.
  // we do this to keep the GPS data buffer from overflowing.
  // *******************************************************************************
  
  char c = GPS.read();
  
  // if a complete sentence has been received, we can parse it...
  if (GPS.newNMEAreceived()) {

    time_ms_new_GPS_available = millis();

    if (debug_echo) {
      Serial.print("GPS.newNMEAreceived() true at t = ");
      Serial.println(time_ms_new_GPS_available); 
    }
  
    // if we fail to parse the sentence we should just bail out. We'll reenter loop and
    // read the next character. We'll keep doing this until the parsing code decides we
    // have an entire, parsable data sentence.
    if (!GPS.parse(GPS.lastNMEA())) return; 

    // we get to here when we have just finished reading a complete, parsable GPS sentence.
    time_ms_parsable_GPS_available = millis();

    // uptake the GPS date/time information. These were set by the GPS.parse command, above.  
    GPS_hour = GPS.hour;
    GPS_minute = GPS.minute;
    GPS_seconds = GPS.seconds;
    GPS_milliseconds = GPS.milliseconds;
    GPS_day = GPS.day;
    GPS_month = GPS.month;
    GPS_year = GPS.year;

    time_ms_finished_parsing_GPS_sentence = millis();

    // set a flag so we know that we've read the GPS clock
    have_read_GPS_clock = true;
    
    if (GPS.fix) {
      GPS_got_satellites = true;
      }else{
      GPS_got_satellites = false;
      }

    /*
      // debugging stuff to check the roll-overs...
      Serial.println("debugging...");
      GPS_hour = 23;
      GPS_minute = 59;
      GPS_seconds = 59;
      GPS_day = 28;
      GPS_month = 2;
      GPS_year = 20;
*/    

    ///////////////////////////// for debugging ////////////////////////////

    if (debug_echo) {
      Serial.print("GPS.parse true at t = ");
      Serial.println(time_ms_parsable_GPS_available, DEC); 
      
      Serial.print("The new GPS data time = ");
      Serial.print(GPS_hour, DEC); Serial.print(':');
      Serial.print(GPS_minute, DEC); Serial.print(':');
      Serial.print(GPS_seconds, DEC); Serial.print('.');
      Serial.print(GPS_milliseconds);

      Serial.print(" Date (dd/mm/yyyy) = ");
      Serial.print(GPS_day, DEC); Serial.print('/');
      Serial.print(GPS_month, DEC); Serial.print("/20");
      Serial.println(GPS_year, DEC);

      Serial.print("Finished parsing the above time/date and loading into variables at t = "); 
      Serial.println(time_ms_finished_parsing_GPS_sentence);
    
      Serial.print("Satellite fix? (yes/no is 1/0) "); Serial.println((int)GPS.fix);

      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
  
        // set the LCD cursor to column 0, line 0
        lcd.setCursor(0, 0);
        lcd.print("Lat. "), lcd.print(GPS.latitude, 4); lcd.print(" "); lcd.print(GPS.lat); 
        lcd.setCursor(0, 1);
        lcd.print("Lon. "), lcd.print(GPS.longitude, 4); lcd.print(" "); lcd.print(GPS.lon);
  
        }else{
  
        Serial.println("No satellite data yet.");
        lcd.setCursor(0, 0);
        lcd.print("still looking   ");
        lcd.setCursor(0, 1);
        lcd.print("for  satellites ");
      
      }
    }     // end of debug_echo if block

    ///////////////////////////////////////////////////////////////////

    // now that we have a newly parsed GPS-based time, add one second to it for later use, as soon
    // as the PPS pin goes positive. Note that we're going to need to handle roll-overs from 59 
    // seconds to 0, and so forth.

      bool bump_flag;
      int place_holder;
      
      RTC_milliseconds = 0;
      RTC_seconds = GPS_seconds + 1;

      // use "place_holder" this way so the timings through the two branches of the if blocks 
      // are the same
      place_holder = GPS_seconds + 1;
      
      if(int(RTC_seconds) >= 60) {
        bump_flag = true;
        RTC_seconds = 0;
        }else{
        bump_flag = false;
        RTC_seconds = place_holder;
        }
        
      place_holder = GPS_minute + 1;
      
      // do we also need to bump the minutes?  
      if (bump_flag) {
        RTC_minute = place_holder;
        }else{
        RTC_minute = GPS_minute;
        }

      // again, do this to equalize the time through the two branches of the if block
      place_holder = RTC_minute;
      
      if(int(RTC_minute) >= 60) {
        bump_flag = true;
        RTC_minute = 0;
        }else{
        bump_flag = false;
        RTC_minute = place_holder;
        }

      place_holder = GPS_hour + 1;
      
      // do we also need to bump the hours?  
      if (bump_flag) {
        RTC_hour = place_holder;
        }else{
        RTC_hour = GPS_hour;
        }

      place_holder = RTC_hour;

      if(int(RTC_hour) >= 24) {
        bump_flag = true;
        RTC_hour = 0;
        }else{
        bump_flag = false;
        RTC_hour = place_holder;
        }

      place_holder = GPS_day + 1;
      
      // do we also need to bump the days?  
      if (bump_flag) {
        RTC_day = place_holder;
        }else{
        RTC_day = GPS_day;
        }

      // do we need to bump the month too? Note the stuff I do to make both paths
      // through the if blocks take the same amount of execution time.
      
      int nobody_home;
      int days_in_month = 31;

      // 30 days hath September, April, June, and November...
      if (int(GPS_month) == 9 || int(GPS_month) == 4 || int(GPS_month) == 6 || int(GPS_month) == 11) {
        days_in_month = 30;
      }else{
        nobody_home = 99;
      }
        
      // ...all the rest have 31, except February...
      if (int(GPS_month) == 2 && (int(GPS_year) % 4)) {
        days_in_month = 28;
      }else{
        nobody_home = 99;
      }
      
      // ...leap year!
      if (int(GPS_month) == 2 && !(int(GPS_year) % 4)) {
        days_in_month = 29;
      }else{
        nobody_home = 99;
      }

      place_holder = RTC_day;
      
      if(int(RTC_day) > days_in_month) {
        bump_flag = true;
        RTC_day = 1;
        }else{
        bump_flag = false;
        RTC_day = place_holder;
        }

      if (bump_flag) {
        RTC_month = GPS_month + 1;
        }else{
        RTC_month = GPS_month;
        }

      place_holder = RTC_month;
                
      //... and also bump the year?
      
      if(int(RTC_month) > 12) {
        bump_flag = true;
        RTC_month = 1;
        }else{
        bump_flag = false;
        RTC_month = place_holder;
        }

      if (bump_flag) {
        RTC_year = GPS_year + 1;
        }else{
        RTC_year = GPS_year;
        }

      // keep track of when we have the proposed RTC time value ready for loading
      time_ms_bumped_RTC_time_ready = millis();

      if (debug_echo) {
        // now print the newly bumped time:
        Serial.print("Now have a proposed (1 second bumped) time ready at (ms) ");
        Serial.println(time_ms_bumped_RTC_time_ready, DEC);       
        Serial.print("Proposed (1 second bumped) time: ");
        Serial.print(RTC_hour, DEC); Serial.print(':');
        Serial.print(RTC_minute, DEC); Serial.print(':');
        Serial.print(RTC_seconds, DEC); Serial.print('.');
        Serial.print(RTC_milliseconds);
        Serial.print("   Date (dd/mm/yyyy): ");
        Serial.print(RTC_day, DEC); Serial.print('/');
        Serial.print(RTC_month, DEC); Serial.print("/20");
        Serial.println(RTC_year, DEC);
      }
    
  }       // end of GPS.newNMEAreceived if block


  sensorValue = analogRead(analogInPin);
  sensorVoltage = sensorValue; // Convert from 0...1024 to 0...5v
  //windSpeed = seonsorVoltage / 5 * 32.4;
  /*Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print("Sensor Voltage: ");
  Serial.print(sensorVoltage);*/
  /*unsigned long endTime = bme.beginReading();
  testInterval = endTime - millis();
  Serial.println(testInterval);
  bme.endReading();
  delay(500);*/

  // Test GPS
  // Serial.print("\nGPS_hour (UTC) = "); Serial.print(GPS_hour); 
  // Serial.print("   GPS_minutes = "); Serial.print(GPS_minutes); 
  // Serial.print("   GPS_seconds = "); Serial.print(GPS_seconds); 
  // Serial.print("   GPS_milliseconds = "); Serial.println(GPS_milliseconds);
  // delay(500);


  // Below to test RTC
  // talk to the realtime clock and print its information. note that unless you've 
  // synchronized the RTC and GPS, they won't necessarily agree.
  // DS3231_query();
  // Serial.print("RTC_hour = "); Serial.print(rtc.now().hour());
  // Serial.print("   RTC_minute = "); Serial.print(rtc.now().minute());
  // Serial.print("   RTC_second = "); Serial.println(rtc.now().second());
  // delay(500);
  
  // this is to make sure measurements are taking in 100ms intervals. Period = 100ms.
  while (true) {
    currentMillis = millis();
    if (currentMillis - startMillis >= period) {
      testInterval = currentMillis - startMillis;
      timeCounter += testInterval;
      startMillis = currentMillis;
      break;
    }
  }
  // Write time data
  //myFile.println(currentMillis % 1000);
  RTC_hour = rtc.now().hour();
  RTC_minute = rtc.now().minute();
  RTC_second = rtc.now().second();
  myFile.print(RTC_hour); myFile.print(',');
  myFile.print(RTC_minute); myFile.print(',');
  myFile.print(RTC_second); myFile.print(',');
  //myFile.println(timeCounter % 1000);
  myFile.println(RTC_milliseconds);

  //myFile.println(testInterval);

  // Write BME data. temperature in °C, pressure in hPa, humidity in %, altitude in m
  myFile.print(bme.temperature);
  myFile.print(",");
  myFile.print(bme.pressure / 100.0);
  myFile.print(",");
  myFile.println(bme.humidity);
  //myFile.print(",");
  //myFile.println(bme.readAltitude(SEALEVELPRESSURE_HPA));

  // Write Anemometer voltage in V.
  myFile.println(sensorVoltage);

  myFile.println();
  myFile.flush();

  // To stop recording data if * was pressed
  if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {
    the_key =  kpd.key[0].kchar;
    if (the_key == '*') {
      Serial.println("Data acquisition stopped from keyboard");
      myFile.println("Data acquisition stopped from keyboard");
      myFile.flush();
      myFile.close();
      LCD_message("Stopped", "# to restart");
      delay(100);
      while(true) {
        if (kpd.getKeys() && kpd.key[0].kstate == PRESSED) {
          the_key =  kpd.key[0].kchar;
          if (the_key == '#') {
            setup();
            break;
          }
        }
      }
    }
  }

  // Record data every 0.1s.
  // delay(100);
}

//////////////////////////////////////////////////////////////////////
////////////////////////// LCD_message function //////////////////////
//////////////////////////////////////////////////////////////////////

void LCD_message(String line1, String line2)
{
  // write two lines (of 16 characters each, maximum) to the LCD display.
  // I assume an object named "lcd" has been created already, has been 
  // initialized in setup, and is global.

  // set the cursor to the beginning of the first line, clear the line, then write.
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(line1);

  // now do the next line.
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(line2);

  return;
}


//////////////////////////////////////////////////////////////////////
////////////////////// DS3231_query function /////////////////////////
//////////////////////////////////////////////////////////////////////

/*void DS3231_query()
{
  // read from the DS3231 real time clock.

  // these were already declared as global variables.
  // int RTC_minute, RTC_second;
  
  // we have already instantiated the device as an object named "now" so we can
  // call its class functions.
  DateTime now = rtc.now();

  lcd.setCursor(0, 0);
  lcd.print("Realtime clock  ");
  
  // write time information to LCD. Year, etc. first, after setting LCD to second line.
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(now.year(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.day(), DEC);

  // delay a second
  delay(1000);

  // now display the day of the week and time.
  RTC_hour = now.hour();
  RTC_minute = now.minute();
  RTC_second = now.second();  
  
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
  lcd.print(". ");
  
  lcd.print(RTC_hour, DEC);
  lcd.print(':');
  if (RTC_minute < 10) {lcd.print('0');}
  lcd.print(RTC_minute, DEC);
  lcd.print(':');
  if (RTC_second < 10) {lcd.print('0');}
  lcd.print(RTC_second, DEC);

  // there are other functions available, such as 
  //    now.unixtime();
  //    DateTime future (now + TimeSpan(7,12,30,6));
  //    future.year(); etc. etc.

}*/



//////////////////////////////////////////////////////////////////////
////////////////////// GPS_query function /////////////////////////
//////////////////////////////////////////////////////////////////////

int GPS_query()
{

  // return 0 if we found good GPS navigational data and -1 if not.
  
  // The GPS device has its own microprocessor and, once we have loaded its parameters,
  // free-runs at a 1 Hz sampling rate. We do not trigger its registration of
  // latitude and longitude, rather we just read from it the last data record
  // it has stored. And we do it one character at a time!

  // I will keep reading from the GPS until I have a complete sentence carrying valid
  // navigational data, with a maximum number of reads to prevent the program from hanging. Once
  // the GPS reports its updates latitude/longitude information, we'll push this to the 
  // LCD display, then return to the main loop. 
   
  // zero out (or set to defaults) values returned by the GPS device in case we can't 
  // get it to respond.
  GPS_hour = GPS_minutes = GPS_seconds = GPS_milliseconds = 0;

  GPS_AV_code_string = "V";

  GPS_latitude_1 = GPS_latitude_2 = 0;
  GPS_latitude_NS_string = "x";
  
  GPS_longitude_1 = GPS_longitude_2 = 0;
  GPS_longitude_EW_string = "y";
    
  GPS_speed_knots = 0.;
  GPS_direction = 0.;
    
  GPS_date_string = "000000";
  
  GPS_fix_quality = GPS_satellites = 0;
  
  GPS_altitude = 0.;

  // initialize the number-of-reads counter since we might find ourselves
  // with an unparseable data record, or an unresponsive GPS, and want to keep trying.
  // This will let me protect against the data logger's program hanging. 
  GPS_char_reads = 0;

  // set a flag saying we want to keep trying; we'll use this to keep looking for
  // useful navigation when the GPS is working fine, but still looking for satellites.
  bool keep_trying = true;
  
  // Stay inside the following loop until we've read a complete GPS sentence with
  // good navigational data, or else the loop times out. With GPS_char_reads_maximum 
  // set to a million this'll take about 6 seconds to time out. 
  
  while (true) {
  
    // if we get back to this point but the keep_trying flag is false, we'll want
    // to declare failure and quit.

    if(!keep_trying) return -1;
    
    // this gets the last sentence read from GPS and clears a newline flag in the Adafruit 
    // library code.
    GPS_sentence = GPS.lastNMEA();
  
    while(GPS_char_reads <= GPS_char_reads_maximum) 
      {
  
      // try to read a single character from the GPS device.
      char single_GPS_char = GPS.read();
  
      // bump the number of times we've tried to read from the GPS.
      GPS_char_reads++;
  
      // now ask if we've received a complete data sentence. If yes, break
      // out of this loop.
  
      if(GPS.newNMEAreceived()) break;
  
      }
  
      
    // if we hit the limit on the number of character reads we'e tried, print a message and bail out.
    if (GPS_char_reads >= GPS_char_reads_maximum) 
      {

      keep_trying = false;
      
      Serial.println("GPS navigation data not yet available. Try again later.");
      //LCD_message("GPS navigation  ", "data unavailable");
        
      return -1;
        
      }

    // get the last complete sentence read from GP; this automatically clears a newline flag inside 
    // the Adafruit library code.
    GPS_sentence = GPS.lastNMEA();
    
    // convert GPS data sentence from a character array to a string.
    GPS_sentence_string = String(GPS_sentence);
  
    // now do a cursory check that the sentence we've just read is OK. Check that there is only
    // one $, as the first character in the sentence, and that there's an asterisk (which commes 
    // immediately before the checksum).
     
    // sentence starts with a $? 
    bool data_OK = GPS_sentence_string.charAt(1) == '$';    
  
    // sentence contains no other $? The indexOf call will return -1 if $ is not found.
    data_OK = data_OK && (GPS_sentence_string.indexOf('$', 2) <  0);
    
    // now find that asterisk...
    data_OK = data_OK && (GPS_sentence_string.indexOf('*', 0) >  0);
  
    if (GPSECHO1) 
      {
      Serial.println("\n******************\njust received a complete sentence, so parse stuff. Sentence is");
      Serial.println(GPS_sentence_string);
      }
  
    // now parse the GPS sentence. I am only interested in sentences that begin with
    // $GPGGA ("GPS fix data") or $GPRMC ("recommended minimum specific GPS/Transit data").
  
    if(GPSECHO1)
      {
      Serial.print("length of GPS_sentence_string just received...");
      Serial.println(GPS_sentence_string.length());
      }
  
    // now get substring holding the GPS command. Only proceed if it is $GPRMC or $GPGGA.
    GPS_command = GPS_sentence_string.substring(0, 7);
  
    // also trim it to make sure we don't have hidden stuff or white space sneaking in.
    GPS_command.trim();
  
    if(GPSECHO1) 
      {
      Serial.print("GPS command is "); Serial.println(GPS_command);
      }   
  
    // if data_OK is true then we have a good sentence. but we also need the sentence
    // to hold navigational data we can use, otherwise we'll want to keep listening.
    // we can only work with GPRMC and GPGGA sentences. 
  
    bool command_OK = GPS_command.equals("$GPRMC") || GPS_command.equals("$GPGGA"); 
    
    // if we have a sentence that, upon cursory inspection, is well formatted AND might
    // hold navigational data, continue to parse the sentence. If the GPS device
    // hasn't found any satellites yet, we'll want to go back to the top of the loop
    // to keep trying, rather than declaring defeat and returning.
  
    //////////////////////////////////////////////////////////////////////
    /////////////////////////// GPRMC sentence ///////////////////////////
    //////////////////////////////////////////////////////////////////////
    
     if (data_OK && GPS_command.equals("$GPRMC"))
        {
            
        if(GPSECHO2) 
          {
          Serial.print("\nnew GPS sentence: "); Serial.println(GPS_sentence_string);
          }
    
        // parse the time. these are global variables, already declared.

        GPS_hour_string = GPS_sentence_string.substring(GPRMC_hour_index1, GPRMC_hour_index2);
        GPS_minutes_string = GPS_sentence_string.substring(GPRMC_minutes_index1, GPRMC_minutes_index2);
        GPS_seconds_string = GPS_sentence_string.substring(GPRMC_seconds_index1, GPRMC_seconds_index2);
        GPS_milliseconds_string = GPS_sentence_string.substring(GPRMC_milliseconds_index1, 
          GPRMC_milliseconds_index2);
        GPS_AV_code_string = GPS_sentence_string.substring(GPRMC_AV_code_index1, GPRMC_AV_code_index2);
    
        GPS_hour = GPS_hour_string.toInt();
        GPS_minutes = GPS_minutes_string.toInt();
        GPS_seconds = GPS_seconds_string.toInt();
        GPS_milliseconds = GPS_milliseconds_string.toInt();
    
        if(GPSECHO2)
          {
          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
          Serial.print("A/V code is "); Serial.println(GPS_AV_code_string);
          }
    
        // now see if the data are valid: we'll expect an "A" as the AV code string.
        // We also expect an asterisk two characters from the end. Also check that the sentence 
        // is at least as long as the minimum length expected.
    
        data_OK = GPS_AV_code_string == "A";
    
        // now look for the asterisk after trimming any trailing whitespace in the GPS sentence.
        // the asterisk preceeds the sentence's checksum information, which I won't bother to check.
        int asterisk_should_be_here = GPS_sentence_string.length() - 4; 
    
        data_OK = data_OK && (GPS_sentence_string.charAt(asterisk_should_be_here) == '*');

        if(GPSECHO2)
          {
          Serial.print("expected asterisk position "); Serial.print(asterisk_should_be_here); 
          Serial.print(" at that position: "); Serial.println(GPS_sentence_string.charAt(asterisk_should_be_here));
          }
    
        // now check that the sentence is not too short.      
        data_OK = data_OK && (GPS_sentence_string.length() >= GPSMINLENGTH);
    
        if (!data_OK) 
          {

          keep_trying = true;
           
          if (GPSECHO1)
            {
            Serial.print("GPS sentence not good for navigation: "); Serial.println(GPS_sentence_string);
            Serial.println("I will keep trying...");
            }
            
          lcd.setCursor(0, 0);
          lcd.print("GPS navigation  ");
          lcd.setCursor(0, 1);
          lcd.print("data not present");
          
          }
    
        // if data are not good, go back to the top of the loop by breaking out of this if block.
        // we've already set keep_trying to be true.
        
        if (!data_OK) break;
            
        // so far so good, so keep going...
        
        // now parse latitude 
        
        GPS_latitude_1_string = GPS_sentence_string.substring(GPRMC_latitude_1_index1, 
          GPRMC_latitude_1_index2);
        GPS_latitude_2_string = GPS_sentence_string.substring(GPRMC_latitude_2_index1, 
          GPRMC_latitude_2_index2);
        GPS_latitude_NS_string = GPS_sentence_string.substring(GPRMC_latitude_NS_index1, 
          GPRMC_latitude_NS_index2);
    
        GPS_latitude_1 = GPS_latitude_1_string.toInt();      
        GPS_latitude_2 = GPS_latitude_2_string.toInt();      
    
        if(GPSECHO2)
          {
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.println(GPS_latitude_NS_string);
          }
          
        // now parse longitude 
        
        GPS_longitude_1_string = GPS_sentence_string.substring(GPRMC_longitude_1_index1, 
          GPRMC_longitude_1_index2);
        GPS_longitude_2_string = GPS_sentence_string.substring(GPRMC_longitude_2_index1, 
          GPRMC_longitude_2_index2);
        GPS_longitude_EW_string = GPS_sentence_string.substring(GPRMC_longitude_EW_index1, 
          GPRMC_longitude_EW_index2);
    
        GPS_longitude_1 = GPS_longitude_1_string.toInt();      
        GPS_longitude_2 = GPS_longitude_2_string.toInt();      
          
        if(GPSECHO2)
          {
          Serial.print("Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.println(GPS_longitude_EW_string); 
          }
    
        // now parse speed and direction. we'll need to locate the 7th and 8th commas in the
        // data sentence to do this. so use the indexOf function to find them.
        // it returns -1 if string wasn't found. the number of digits is not uniquely defined 
        // so we need to find the fields based on the commas separating them from others.
        
        int comma_A_index = GPRMC_longitude_EW_index2;
        int comma_B_index = GPS_sentence_string.indexOf(",", comma_A_index + 1);
        int comma_C_index = GPS_sentence_string.indexOf(",", comma_B_index + 1);
    
        GPS_speed_knots_string = GPS_sentence_string.substring(comma_A_index + 1, comma_B_index); 
        GPS_direction_string = GPS_sentence_string.substring(comma_B_index + 1, comma_C_index); 
        
        GPS_speed_knots = GPS_speed_knots_string.toFloat();
        GPS_direction = GPS_direction_string.toFloat();
    
        if(GPSECHO2)
          {
          Serial.print("Speed (knots) = "); Serial.println(GPS_speed_knots);
          Serial.print("Direction (degrees) = "); Serial.println(GPS_direction);
          }
          
        // now get the (UTC) date, in format DDMMYY, e.g. 080618 for 8 June 2018.
        GPS_date_string = GPS_sentence_string.substring(comma_C_index+ + 1, comma_C_index + 7);
        
        if(GPSECHO2)
          {
          Serial.print("date, in format ddmmyy = "); Serial.println(GPS_date_string);    
          }
    
        // Write message to LCD now. It will look like this (no satellite data in this record):
        //     Sats: 4006.9539N
        //     N/A  08815.4431W
        
        lcd.setCursor(0, 0);
        lcd.print("Sats: ");
        lcd.setCursor(6, 0);
        lcd.print(GPS_latitude_1_string); lcd.print("."); lcd.print(GPS_latitude_2_string); 
        lcd.print(GPS_latitude_NS_string); 
    
        lcd.setCursor(0, 1);
        lcd.print("N/A ");
        lcd.setCursor(5, 1);
        lcd.print(GPS_longitude_1_string); lcd.print("."); lcd.print(GPS_longitude_2_string); 
        lcd.print(GPS_longitude_EW_string);

        // print a summary of the data and parsed results:
        if(GPSECHO3)
          {
          Serial.print("GPS sentence: "); Serial.println(GPS_sentence_string);

          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
        
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.print(" "); Serial.print(GPS_latitude_NS_string);

          Serial.print("    Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.print(" "); Serial.println(GPS_longitude_EW_string); 

          Serial.print("Speed (knots) = "); Serial.print(GPS_speed_knots);
          Serial.print("     Direction (degrees) = "); Serial.println(GPS_direction);

          Serial.println("There is no satellite or altitude information in a GPRMC data sentence.");
              
          }
      
        // all done with this sentence, so return.
        return 0;
          
        }  // end of "if (data_OK && GPS_command.equals("$GPRMC"))" block

    //////////////////////////////////////////////////////////////////////
    /////////////////////////// GPGGA sentence ///////////////////////////
    //////////////////////////////////////////////////////////////////////
    
      if (data_OK && GPS_command.equals("$GPGGA"))
        {

        if(GPSECHO2) 
          {
          Serial.print("\nnew GPS sentence: "); Serial.println(GPS_sentence_string);
          }
    
        // parse the time
    
        GPS_hour_string = GPS_sentence_string.substring(GPGGA_hour_index1, GPGGA_hour_index2);
        GPS_minutes_string = GPS_sentence_string.substring(GPGGA_minutes_index1, GPGGA_minutes_index2);
        GPS_seconds_string = GPS_sentence_string.substring(GPGGA_seconds_index1, GPGGA_seconds_index2);
        GPS_milliseconds_string = GPS_sentence_string.substring(GPGGA_milliseconds_index1, 
          GPGGA_milliseconds_index2);
    
        GPS_hour = GPS_hour_string.toInt();
        GPS_minutes = GPS_minutes_string.toInt();
        GPS_seconds = GPS_seconds_string.toInt();
        GPS_milliseconds = GPS_milliseconds_string.toInt();
    
        if(GPSECHO2)
          {
          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
          }
    
        // now get the fix quality and number of satellites.
    
        GPS_fix_quality_string = GPS_sentence_string.substring(GPGGA_fix_quality_index1, 
          GPGGA_fix_quality_index2);
        GPS_satellites_string = GPS_sentence_string.substring(GPGGA_satellites_index1, 
          GPGGA_satellites_index2);
    
        int GPS_fix_quality = GPS_fix_quality_string.toInt();      
        int GPS_satellites = GPS_satellites_string.toInt();      
    
        if(GPSECHO2)
          {
          Serial.print("fix quality (1 for GPS, 2 for DGPS) = "); Serial.println(GPS_fix_quality);
          Serial.print("number of satellites = "); Serial.println(GPS_satellites);
          }
    
        // now see if the data are valid: we'll expect a fix, and at least three satellites.
    
        bool data_OK = (GPS_fix_quality > 0) && (GPS_satellites >= 3); 
    
        // now look for the asterisk.
        int asterisk_should_be_here = GPS_sentence_string.length() - 4; 
    
        data_OK = data_OK && (GPS_sentence_string.charAt(asterisk_should_be_here) == '*');
    
        // now check that the sentence is not too short.      
        data_OK = data_OK && (GPS_sentence_string.length() >= GPSMINLENGTH);

        if (!data_OK) 
          {

          keep_trying = true;
           
          if (GPSECHO1)
            {
            Serial.print("GPS sentence not good for navigation: "); Serial.println(GPS_sentence_string);
            Serial.println("I will keep trying...");
            }
            
          lcd.setCursor(0, 0);
          lcd.print("GPS navigation  ");
          lcd.setCursor(0, 1);
          lcd.print("data not present");
          
          }
    
        // if data are not good, go back to the top of the loop by breaking out of this if block.
        
        if (!data_OK) break;
            
        // so far so good, so keep going...
        
        // now parse latitude 
        
        String GPS_latitude_1_string = GPS_sentence_string.substring(GPGGA_latitude_1_index1, 
        GPGGA_latitude_1_index2);
        String GPS_latitude_2_string = GPS_sentence_string.substring(GPGGA_latitude_2_index1, 
        GPGGA_latitude_2_index2);
        String GPS_latitude_NS_string = GPS_sentence_string.substring(GPGGA_latitude_NS_index1, 
        GPGGA_latitude_NS_index2);
    
        int GPS_latitude_1 = GPS_latitude_1_string.toInt();      
        int GPS_latitude_2 = GPS_latitude_2_string.toInt();      
    
        if(GPSECHO2)
          {
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.println(GPS_latitude_NS_string);
          }
          
        // now parse longitude 
        
        String GPS_longitude_1_string = GPS_sentence_string.substring(GPGGA_longitude_1_index1, 
        GPGGA_longitude_1_index2);
        String GPS_longitude_2_string = GPS_sentence_string.substring(GPGGA_longitude_2_index1, 
        GPGGA_longitude_2_index2);
        String GPS_longitude_EW_string = GPS_sentence_string.substring(GPGGA_longitude_EW_index1, 
        GPGGA_longitude_EW_index2);
    
        int GPS_longitude_1 = GPS_longitude_1_string.toInt();      
        int GPS_longitude_2 = GPS_longitude_2_string.toInt();      
    
        if(GPSECHO2)
          {         
          Serial.print("Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.println(GPS_longitude_EW_string); 
          }
          
        // let's skip the "horizontal dilution" figure and go straight for the altitude now.
        // this begins two fields to the right of the num,ber of satellites so find this
        // by counting commas. use the indexOf function to find them.
        int comma_A_index = GPS_sentence_string.indexOf(",", GPGGA_satellites_index2 + 1);
        int comma_B_index = GPS_sentence_string.indexOf(",", comma_A_index + 1);
    
        String GPS_altitude_string = GPS_sentence_string.substring(comma_A_index + 1, comma_B_index); 
        
        float GPS_altitude = GPS_altitude_string.toFloat();
    
        if(GPSECHO2)
          {
          Serial.print("Altitude (meters) = "); Serial.println(GPS_altitude);
          }
    
        // Write message to LCD now. It will look like this:
        //     Sats: 4006.9539N
        //       10 08815.4431W
             
        lcd.setCursor(0, 0);
        lcd.print("Sats: ");
        lcd.setCursor(6, 0);
        lcd.print(GPS_latitude_1_string); lcd.print("."); lcd.print(GPS_latitude_2_string); 
        lcd.print(GPS_latitude_NS_string); 
    
        lcd.setCursor(0, 1);
        lcd.print("      ");
        lcd.setCursor(2, 1);
        lcd.print(GPS_satellites);
        lcd.setCursor(5, 1);
        lcd.print(GPS_longitude_1_string); lcd.print("."); lcd.print(GPS_longitude_2_string); 
        lcd.print(GPS_longitude_EW_string);

        // print a summary of the data and parsed results:
        if(GPSECHO3) {
          Serial.print("GPS sentence: "); Serial.println(GPS_sentence_string);

          Serial.print("Time (UTC) = "); Serial.print(GPS_hour); Serial.print(":");
          Serial.print(GPS_minutes); Serial.print(":");
          Serial.print(GPS_seconds); Serial.print(".");
          Serial.println(GPS_milliseconds);
        
          Serial.print("Latitude x 100 = "); Serial.print(GPS_latitude_1); Serial.print(".");
          Serial.print(GPS_latitude_2); Serial.print(" "); Serial.print(GPS_latitude_NS_string);

          Serial.print("    Longitude x 100 = "); Serial.print(GPS_longitude_1); Serial.print(".");
          Serial.print(GPS_longitude_2); Serial.print(" "); Serial.println(GPS_longitude_EW_string); 

          Serial.print("Speed (knots) = "); Serial.print(GPS_speed_knots);
          Serial.print("     Direction (degrees) = "); Serial.println(GPS_direction);

          Serial.print("Number of satellites: "); Serial.print(GPS_satellites);
          Serial.print("       Altitude (meters): "); Serial.println(GPS_altitude);
              
        }
       
        // all done with this sentence, so return.
        return 0;
       
    }   // end of "if (data_OK && GPS_command.equals("$GPGGA"))" block
  
    // we'll fall through to here (instead of returning) when we've read a complete 
    // sentence, but it doesn't have navigational information (for example, an antenna 
    // status record).
    
  }
}
