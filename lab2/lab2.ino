/*This code is to test the use of the LIDAR sensors, the VL53L0X

I2C communicates with each of the units*/

#include <string.h>
#include <SPI.h>
#include <robotlib.h>
#include <math.h>
robotlib Robot;
#include <Wire.h>

// VL53L0X needed stuff
#include <VL53L0X.h>
VL53L0X FRsensor;
VL53L0X FSsensor;
VL53L0X SIsensor;
VL53L0X BSsensor;

int mspeed = 8000; //This is the time in microseconds between steps

void setup() {
  // put your setup code here, to run once:
  Robot.initial();
  delay(100);
}
float a;


float sense(float in){
  in = pow(in,-1.4);
  in *= 22550;
  return in;
}

void forward(int distance) {
  double _speed_offset = 28.93;
//  Robot.LCDclear();
//  Robot.LCDstring("Forward ");
  distance *= _speed_offset;
  for (int j = 0; j < distance; j++)
    Robot.forward(mspeed);
}

void backward(int distance) {
//  Robot.LCDclear();
//  Robot.LCDstring("Backward ");
  for (int j = 0; j < distance; j++)
    Robot.backward(mspeed);
}


void local_turn(int degree){
  //we set turning right is positive
  //and turning left is negative
  float Wheel_size = 2.2;
  float Wheel_distance = 8.5;
  float pi = 3.1415926;
  
  if(degree ==0) return;
  else if(degree > 0){
    int steps = ((pi*Wheel_distance*degree/360)/(pi*Wheel_size))*200;
//    Robot.LCDclear();
//    Robot.LCDstring("Right2 ");
//    Robot.LCDnum(steps);
//    Robot.LCDnum(degree);
    
    
    for (int j = 0; j < steps; j++)
    Robot.right2(mspeed);
  }
  else if(degree < 0){
    degree = -degree;
    int steps = ((pi*Wheel_distance*degree/360)/(pi*Wheel_size))*200;

//    Robot.LCDclear();
//    Robot.LCDstring("Left2 ");
//    Robot.LCDnum(steps);
    for (int j = 0; j < steps; j++)
    Robot.left2(mspeed);
  }
}

void lab2_test(){
  a = analogRead(0); // FRont sensor
  Serial.print(a);
  Serial.print("   ");
  Serial.print(sense(a));
  Serial.print("   ");
  Robot.LCDnum(a);
  Robot.LCDstring(",  ");
  Robot.LCDnum(sense(a));
  Robot.LCDstring(",  ");
  
  a = analogRead(1); // FrontSide sensor
  Serial.print(a);
  Serial.print("   ");
  Robot.LCDnum(a);
  Robot.LCDstring(",  ");
  
  a = analogRead(2); // SIde sensor
  Serial.print(a);
  Serial.print("   ");
  Robot.LCDnum(a);
  Robot.LCDstring(",  ");
  
  a = analogRead(3); //BackSide sensor
  Serial.print(a);
  Serial.print("   ");
  Serial.println();
  Robot.LCDnum(a);
  Robot.LCDstring(",  ");
  
  Serial.println();
  delay(500);
}
enum states{NADA,FORWARD,LTURN,RFORWARD,RTURN,FTURN,STOP};
states robot = NADA;
int turn = 0;
float shift_distance = 0;
float move_distance = 0;
float total_distance = 120;

void loop() {
  // put your main code here, to run repeatedly:
  //Read and print the VL53L0x to the LCD and the Serial Monitor
  
//  Robot.LCDclear();
//  delay(10);
  float front = analogRead(0);
  float right = analogRead(2);
  Serial.print(robot);

  switch(robot){
    case NADA:{
      if(front < 300) robot = FORWARD;
      break;
    }
    case FORWARD:{
      forward(2);
      move_distance += 2;
      if(front > 400) robot = LTURN; 
      if(move_distance >= total_distance){
        robot = STOP;
      }
      turn = 0;
      break;
    }
    
    case LTURN:{
      turn++;
      local_turn(-87);
      robot = RFORWARD;
      break;
    }
    case RFORWARD:{

      if(front > 400){
        robot = LTURN;
        break;
      }
      
      if(right < 250) {
        forward(6);
        if(turn % 2 == 1){
          shift_distance += 6;
        } else {
          move_distance += 6;
        }
        
        robot = RTURN; 
        break;
      }

      if(turn %2 == 0){
        move_distance += 2;
      } else {
        shift_distance += 2;
      }
      forward(2);
      break;
    }
    
    case RTURN:{
      turn++;
      local_turn(90);
      forward(12);
      
      if(turn % 2 == 0){
        move_distance += 12;
        robot = RFORWARD;
        break;
      } else {
        shift_distance -= 12;
        robot = FTURN;
      }
      break;
    }

    case FTURN:{
      forward(shift_distance);
      local_turn(-87);
      robot = FORWARD;

      break;
    }
    case STOP:{
      
      break;
    }
    
  }
  
  
  
  
  
}
