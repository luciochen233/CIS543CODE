#include <SPI.h>
#include <robotlib.h>
robotlib Robot;

VL53L0X FRsensor;
VL53L0X FSsensor;
VL53L0X SIsensor;
VL53L0X BSsensor;

int mspeed = 4000; //This is the time in microseconds between steps
int total_distance = 0;

void setup() {
  // put your setup code here, to run once:

  Robot.initial();  // This initiallizes the PIC microcontrollers and the Sensors
  Serial.begin(9600); // sends data out on the USB

}

void foward(int distance) {
  double _speed_offset = 28.93;
  Robot.LCDclear();
  Robot.LCDstring("Forward ");
  distance *= _speed_offset;
  for (int j = 0; j < distance; j++)
    Robot.forward(mspeed);
}

void backward(int distance) {
  Robot.LCDclear();
  Robot.LCDstring("Backward ");
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
    Robot.LCDclear();
    Robot.LCDstring("Right2 ");
    Robot.LCDnum(steps);
    Robot.LCDnum(degree);
    
    
    for (int j = 0; j < steps; j++)
    Robot.right2(mspeed);
  }
  else if(degree < 0){
    degree = -degree;
    int steps = ((pi*Wheel_distance*degree/360)/(pi*Wheel_size))*200;

    Robot.LCDclear();
    Robot.LCDstring("Left2 ");
    Robot.LCDnum(steps);
    for (int j = 0; j < steps; j++)
    Robot.left2(mspeed);
  }
}

void turn_90(){
  int off = 193;
  Robot.LCDclear();
  Robot.LCDstring("Left 90 degree ");
  for (int j = 0; j < 193; j++)
    Robot.left2(mspeed);

}

void angle_move(int direction, int speed){
  Robot.LCDclear();
  Robot.LCDstring("TURN(-5) ");
  for (int j = 0; j < 25; j++)
    Robot.TURN(-5, mspeed);

  Robot.LCDclear();
  Robot.LCDstring("TURN(5) ");
  for (int j = 0; j < 25; j++)
    Robot.TURN(5, mspeed);

  Robot.LCDclear();
  Robot.LCDstring("TURN(0) ");
  for (int j = 0; j < 25; j++)
    Robot.TURN(0, mspeed);
}

void loop() {
  // put your main code here, to run repeatedly:
  // This code runs the motor forward and backward
  foward(60);
  //Robot.LCDchar("This is the end ", 16);  //Writes the string, needs the length

  delay(1000);
  local_turn(-90);
  delay(1000);
  
  int aa = 27;


  Robot.LCDnum(aa); // This writes an integer (no length required

  Robot.LCDstring("   ");
  Robot.LCDstring("Maybe not");  // This writes a string no length required.
  delay(2000);

}
