/*
robotlib.h -Library for the SU robot.
Created by William C. Tetley
Dated March 30, 2017
This library is used to change over the robot from the freescale microcontr4oller to the Teensy
LC controller (Arduinio).
*/
#ifndef robotlib_h
#define robotlib_h
#include <VL53L0X.h>
#include "Arduino.h"
#include <SPI.h>
#include <string.h>
enum RobotDirection {STRA, SIDE, FLOO};
class robotlib
{
	public:
		VL53L0X FRsensor;
		VL53L0X FSsensor;
		VL53L0X SIsensor;
		VL53L0X BSsensor;
		void initial();
		
		void forward(int delta);
		void backward(int delta);
		void left1(int delta);
		void left2(int delta);
		void right1(int delta);
		void right2(int delta);
		void TURN(int radius, int delta);
		
		float readsensor(String detector);
		void detectorgain(char a);
		void detectorread();
		void detectorselect(RobotDirection a);
		int Read[4];
		
		void LCDclear();
		void LCDchar(String mystr, int length);
		void LCDstring(String mystr);
		void LCDnum(float a);
		};
#endif