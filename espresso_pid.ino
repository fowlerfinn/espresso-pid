/*
  Initial release:  02/21/2019
  updated: 06/04/2019 (relay output)
  Author: Matt FowlerFinn
  Platforms: WEMOS D1 lite & W
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

/*TODO:
 * remove adafruit(now broken w/ remapped i2c), port to u8g2
 * add gaggia logo on startup
 * add status display
 */
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>            
#include <Adafruit_SSD1306.h>     
#include <Fonts/FreeSans12pt7b.h>   
#include <Fonts/FreeSansBold9pt7b.h>

#if (ESP32)  // --HELTEC WIFI 32 (ESP32)-- 
  #warning "compiling for HELTEC WIFI 32 (ESP32)"
  static const uint8_t Vext = 21;
  static const uint8_t LED  = 25;
  static const uint8_t RST_OLED = 16;
  static const uint8_t SCL_OLED = 15;
  static const uint8_t SDA_OLED = 4;
  #define DISPLAY_HEIGHT 64
  #define DISPLAY_WIDTH  128
  int OLED_RESET = RST_OLED;
  #define SSRPin A2 
  #define ONE_WIRE_BUS A1

#else // --WEMOS D1 Lite (ESP8266)-- 
  #warning "compiling for WEMOS D1 Lite (ESP8266)"
  #define SSRPin D2 
  #define ONE_WIRE_BUS D1
  int OLED_RESET = 16;
  #define DISPLAY_WIDTH 128 // OLED display width, in pixels
  #define DISPLAY_HEIGHT 64 // OLED display height, in pixels  
  static const uint8_t SCL_OLED = SCL;
  static const uint8_t SDA_OLED = SDA;
  
#endif
//both boards can use LED_BUILTIN
TwoWire twi = TwoWire(1);
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &twi, OLED_RESET);


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

#define SERIAL_BAUD 115200


double Setpoint = 103; //99c-107c for 95c at grouphead
int toleranceWindow = 2; // +/- degrees for acceptable operation feedback 
double aggKp=100, aggKi=0, aggKd=0;
double Kp=10, Ki=.5, Kd=95; //was 30,10,90

float celsius;
double Input, Output;

PID myPID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);

//freq to run pid and switch on/off the heater AND OUTPUT SCALE
int WindowSize = 1000; //in microseconds
int pidMaxValue = 100;
unsigned long windowStartTime;


void setup()   {     
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high  
  
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  pinMode(SSRPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(SSRPin, LOW);

  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 12 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 12);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  twi.begin(SDA_OLED, SCL_OLED); //SDA,SCL
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3c
  display.setTextColor(WHITE);
  display.clearDisplay();
  
  myPID.SetTunings(aggKp, aggKi, aggKd);
  myPID.SetOutputLimits(0, pidMaxValue);
  windowStartTime = millis();
  myPID.SetMode(AUTOMATIC);
  
  display.setTextColor(WHITE);
  display.clearDisplay();
  
  //version annotation
  Serial.println("espresso pid - v1.1");
  display.setCursor(0,0);
  display.println("espresso pid");
  display.setCursor(0,15);
  display.println("v1.1");
  display.setCursor(0,50);
  display.println("Matt FowlerFinn");
  display.display();
  display.setFont(&FreeSansBold9pt7b);
  display.setTextSize(1);
  delay(1000); 
  
}

//////////////////////////////////////////////////////////////////
void sampleTemp() {
  sensors.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensors.getTempC(insideThermometer);
  celsius = tempC;
  Input = celsius;
}

void calcPID() {
  //myPID.Compute();
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
}
  
void refreshDisplay() {
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0,18);
  display.print(celsius,1);
  display.println("c");
  display.display();

  Serial.print(celsius);
  Serial.print("C, ");
}

void loop(){
    sampleTemp();
    watchDog();
    adaptTune();
    calcPID();
    statusLight();
    refreshDisplay();
    Serial.println();
    delay (500);
    }

void watchDog() {
  if (celsius > (Setpoint+(toleranceWindow*2)))
  {  //TURN OFF PID IF OVER SETPOINT
    myPID.SetMode(MANUAL);
    Output = 0;
    Serial.println("OVERTEMP!!");
    delay(10000);
  }
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void statusLight() {
    //LED indicator
  if( (abs(Setpoint - celsius)) < toleranceWindow ) { 
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("READY TO BREW, ");
    }
    else { //NOT READY
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("TEMP NOT CLOSE ENOUGH, ");
    }
}

void adaptTune() {
  if (celsius < 85) {
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  else {
    myPID.SetTunings(Kp, Ki, Kd);
  }
  
  if (celsius > (Setpoint))
  {  //TURN OFF PID IF OVER SETPOINT
    myPID.SetMode(MANUAL);
    Output = 0;
  }
  else
  {
    myPID.SetMode(AUTOMATIC); 
  }
}
