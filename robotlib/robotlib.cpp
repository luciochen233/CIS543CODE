/* robotlib- Library for SU robot.
Created by William C. Tetley
March 30, 2017
*/

#include "Arduino.h"
#include "robotlib.h"
#include <SPI.h>
#include <string.h>
#include <VL53L0X.h>
#include <Wire.h>

void robotlib::initial()
{
	// First we initialize the SPI part of the robot (motors and LCD)
pinMode(13,OUTPUT); //SPI pin SCK (clock)
pinMode(11,OUTPUT); //SPI pin MOSI (data) no retuurn data
digitalWrite(11,HIGH);
delay(100);

SPISettings settingsA(100000, MSBFIRST, SPI_MODE1);

SPI.begin();
SPI.beginTransaction(settingsA);
 
SPI.transfer(1); // Makes all of the PIC chips perform their initialization subroutines
delay(10);
SPI.transfer(2); //This initalizes the LCD Display
delay(10);

	// Next we initialize the I2C portion of the robot (Sensors)

extern VL53L0X FRsensor;
extern VL53L0X FSsensor;
extern VL53L0X SIsensor;
extern VL53L0X BSsensor;	
// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

#define LONG_RANGE
// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
#define HIGH_ACCURACY

  pinMode(1,OUTPUT);
  digitalWrite(1,LOW);
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);
  pinMode(3,OUTPUT);
  digitalWrite(3,LOW);
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);

 
  Serial.begin(9600);
  Wire.begin();

//Initialize VL53L0X
  // Initialize the Front  sensor
  pinMode(1,INPUT);
  delay(100);
  
  FRsensor.init();
  FRsensor.setTimeout(500);
  FRsensor.setAddress((uint8_t)22);

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  FRsensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  FRsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  FRsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  FRsensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  FRsensor.setMeasurementTimingBudget(200000);
  #endif

// Initialize the Front Right sensor
  pinMode(2,INPUT);
  delay(10);
  FSsensor.init();
  FSsensor.setTimeout(500);
  FSsensor.setAddress((uint8_t)23);
  #if defined LONG_RANGE
  FSsensor.setSignalRateLimit(0.1);
  FSsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  FSsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
  #if defined HIGH_SPEED
  FSsensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
  FSsensor.setMeasurementTimingBudget(200000);
  #endif

// Initialize the SIde sensor
  pinMode(3,INPUT);
  delay(10);
  SIsensor.init();
  SIsensor.setTimeout(500);
  SIsensor.setAddress((uint8_t)24);
  #if defined LONG_RANGE
  SIsensor.setSignalRateLimit(0.1);
  SIsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  SIsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
  #if defined HIGH_SPEED
  SIsensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
  SIsensor.setMeasurementTimingBudget(200000);
  #endif

// Initialize the Back Side sensor
  pinMode(4,INPUT);
  delay(10);
  BSsensor.init();
  BSsensor.setTimeout(500);
  BSsensor.setAddress((uint8_t)25);
  #if defined LONG_RANGE
  BSsensor.setSignalRateLimit(0.1);
  BSsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  BSsensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif
  #if defined HIGH_SPEED
  BSsensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
  BSsensor.setMeasurementTimingBudget(200000);
  #endif

 // Turn on the sensors   
FRsensor.startContinuous();
FSsensor.startContinuous();
SIsensor.startContinuous();
BSsensor.startContinuous();
	
}

// Motor functions

void robotlib::forward(int delta)
{
 SPI.transfer(0x0F);
delayMicroseconds(delta); 
}

void robotlib::backward(int delta)
{
SPI.transfer(0x0C);
delayMicroseconds(delta); 
}

void robotlib::left1(int delta)
{
SPI.transfer(0x09);
delayMicroseconds(delta);
}

void robotlib::left2(int delta)
{
SPI.transfer(0x0D);
delayMicroseconds(delta);
}

void robotlib::right1(int delta)
{
SPI.transfer(0x0A);
delayMicroseconds(delta);
}

void robotlib::right2(int delta)
{
SPI.transfer(0x0E);
delayMicroseconds(delta);
}

void robotlib::TURN(int radius, int delta)
{
int i;	// Make sure radius is between -10 and 10
    if (radius < -10 )
    radius = -10;
    if (radius > 10)
    radius = 10;
 if (radius < 0)  //turn to the left
 {
    radius = -radius;
    for (int i1=0;i1<radius;i1++)
    left1(delta);
    for(int i1=0;i1<11-radius;i1++)
    forward(delta);
 }
 else if (radius >0)  //turn to the right
 {	 
	for (int i1=0;i1<radius;i1++)
    right1(delta);
    for(int i1=0;i1<11-radius;i1++)
    forward(delta);
 }
  else if (radius == 0)  //go straight
  {
      for (i=0;i<10;i++)
      forward(delta);
  }
}

//Detector Functions
float robotlib::readsensor(String detector)
{
	extern VL53L0X FRsensor;
	extern VL53L0X FSsensor;
	extern VL53L0X SIsensor;
	extern VL53L0X BSsensor;
	int a =0;
	if (detector == "FR")
	a = FRsensor.readRangeContinuousMillimeters();
	if (detector == "FS")
	a = FSsensor.readRangeContinuousMillimeters();
	if (detector == "SI")
	a = SIsensor.readRangeContinuousMillimeters();
	if (detector == "BS")
	a = BSsensor.readRangeContinuousMillimeters();
	float b = float(a)/25.4;
    return b;
}
void robotlib::detectorgain(char a)
{
	if(a == 'H')
		SPI.transfer(0x19); 
	else
		SPI.transfer(0x18); 
}

void robotlib::detectorselect(RobotDirection a)
{
	if(a ==0)
		SPI.transfer(0x1F);	//Activates the front sensors
	if(a==1)
		SPI.transfer(0x1E);	//Ativates the side
	if(a==2)
		SPI.transfer(0x1D);   	//Activates the floor 
}

void robotlib::detectorread()
{
	for(int i=0; i<4; i++)
		Read[i]=analogRead(i);
	return;
}

//LCD Display Functions

void robotlib::LCDclear()
{
	SPI.transfer(3); // Clears the LCD Display
	delay(10);
	SPI.transfer(0x40); // 0x40 moves the cursor to 0,0
	delay(10);	
}

void robotlib::LCDchar(String mystr, int length)
{
	for(int i=0;i<length;i++)
{
		SPI.transfer(0x80+mystr[i]);
		delay(10);
}
}
void robotlib::LCDstring(String mystr)
{
	for(int i=0;i<mystr.length();i++)
{
		SPI.transfer(0x80+mystr[i]);
		delay(10);
}
}

void robotlib::LCDnum(float a)
{ 
	String mystr = String(a);
	for(int i=0;i<mystr.length();i++)
{
		SPI.transfer(0x80+mystr[i]);
		delay(10);
}
}