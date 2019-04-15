/*
  Initial release:  02/21/2019
  updated: 03/01/2019 (relay output)
  Author: Matt FowlerFinn
  Platforms: ESP32 oled board by heltec automation
  ------------------------------------------------------------------------
  Description: 
  espresso pid

  Sensor:
  ds18b20 on i2c bus
  ------------------------------------------------------------------------
  License:
  Released under the MIT license. Please check LICENSE.txt for more
  information.  All text above must be included in any redistribution. 
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>             //"ADAFRUIT GFX Library" by "Adafruit" v1.2.3
#include <Adafruit_SSD1306.h>         //"ADAFRUIT SSD1306" library by "Adafruit" v1.1.2
#include <Fonts/FreeSans12pt7b.h>     //"ADAFRUIT GFX Library" by "Adafruit" v1.2.3
#include <Fonts/FreeSansBold9pt7b.h>  //"ADAFRUIT GFX Library" by "Adafruit" v1.2.3
//#include <Arduino.h>
//#include <analogWrite.h>
const int ledPin = D4; //pull low for light

#define OLED_RESET 16

#define SSRPin D2 // CHECK PIN!!try13
#define ONE_WIRE_BUS D1
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
//OneWire  ds(D1);  // CHECK PIN!! on pin 11 (a 4.7K resistor is necessary)
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

#define SERIAL_BAUD 115200
Adafruit_SSD1306 display(OLED_RESET);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
float celsius, fahrenheit;
double Setpoint, Input, Output;
double aggKp=30, aggKi=10, aggKd=90;
double consKp=60, consKi=0.1, consKd=0;
int mode, duty;
boolean active = true;
PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, P_ON_M, DIRECT);



//freq to run pid and switch on/off the heater AND OUTPUT SCALE
int WindowSize = 1000; //in microseconds
int pidMaxValue = 100;
unsigned long windowStartTime;

void setup()   {     
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  //while(!Serial) {} // Wait
  pinMode(SSRPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(SSRPin, LOW);

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 12);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

//  analogWriteResolution(SSRPin, 8);
//  analogWriteFrequency(5000); // set frequency to 5 KHz for all pins

  Wire.begin(D5,D6);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3c
  display.setTextColor(WHITE);
  display.clearDisplay();
  Setpoint = 103; //99c-107c for 95c at grouphead
  myPID.SetTunings(aggKp, aggKi, aggKd);
  myPID.SetOutputLimits(0, pidMaxValue);
  windowStartTime = millis();
  myPID.SetMode(AUTOMATIC);
  
  display.setTextColor(WHITE);
  display.clearDisplay();
  
  //version annotation
  display.setCursor(0,0);
  display.println("espresso pid");
  display.setCursor(0,15);
  display.println("v1.0");
  display.setCursor(0,50);
  display.println("Matt FowlerFinn");
  display.display();
  display.setFont(&FreeSansBold9pt7b);
  display.setTextSize(1);
  delay(1000); 
  
}

//////////////////////////////////////////////////////////////////
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
 /*
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  */
  celsius = tempC;
}

void sampleTemp() {
  sensors.requestTemperatures(); // Send the command to get temperatures
  printTemperature(insideThermometer); // Use a simple function to print out the data
  /*
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  ds.reset_search();
  if ( !ds.search(addr)) {
//    lcd.clear();
//    lcd.println(" DS18 not found ");
    ds.reset_search();
    delay(250);
    return;
  }
  
//  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
 //   Serial.write(' ');
//    Serial.print(addr[i], HEX);
  }

 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //lcd.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 0);        // start conversion, without parasite power on at the end
  
  delay(100);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

 // Serial.print("  Data = ");
 // Serial.print(present, HEX);
 // Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
 //   Serial.print(data[i], HEX);
 //   Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }

  */
  
  //celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Input = celsius;
}

void calcPID() {
  
  myPID.Compute();
  int scaler = ( WindowSize / pidMaxValue );
  int scaledOutput = (Output * scaler);
  
  if (millis() - windowStartTime > WindowSize)
  { 
    myPID.Compute();
    windowStartTime += WindowSize; //time to shift the Relay Window
  }
  if ((scaledOutput > millis() - windowStartTime) && (celsius < Setpoint)) {
    digitalWrite(SSRPin, HIGH);
    Serial.print("ON , ");
  }
  else {
    Serial.print("OFF, ");
    digitalWrite(SSRPin, LOW);
  }
  
  duty = (Output);

  /*
  Serial.print(scaledOutput);
  Serial.print(" scaledOutput, ");
  Serial.print(duty);
  Serial.print(" duty, ");
  */

  //LED indicator
  if( (abs(Setpoint - celsius)) >= 2 ) { //NOT READY
    digitalWrite(ledPin, HIGH);
    }
    else { //READY TO BREW
      digitalWrite(ledPin, LOW);
    }
 /*
  Serial.print(Output, 1);
  Serial.print(" Output, "); 
  
  Serial.print(windowStartTime, 1);
  Serial.print(" windowStartTime, "); 

  Serial.print(millis());
  Serial.print(" millis, "); 
  
  //Serial.print(" Output, ");
  //Serial.print(lite);
  //Serial.print(" led pin, ");
  */
}
  
void refresh() {
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0,18);
  display.print(celsius,1);
  display.println("c");

  display.setCursor(0,45);
  display.print(duty,1);
  display.println("% heat");
  
  display.display();

  Serial.print(celsius);
  Serial.print("C, ");
  Serial.print(duty,1);
  Serial.print("% heat");
}

void loop(){
    sampleTemp();
    adaptTune();
    calcPID();
    refresh();
    Serial.println();
    }

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


void adaptTune() {
  if (celsius > (Setpoint))
  {  //TURN OFF PID IF OVER SETPOINT
  myPID.SetMode(MANUAL);
  Output = 0;
  Serial.print("OVERTEMP!!, ");
  }
  else
  {
   myPID.SetMode(AUTOMATIC); 
  }
}
