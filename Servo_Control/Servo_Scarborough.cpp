#include "Servo_Scarborough.h"
#include <Arduino.h>

Servo_Scarborough::Servo_Scarborough(int _m0, int _m1, int _m2, int _m3, int _m4, int _m5){

	//Attach the servo to the pins
	m0.attach(_m0);
	m1.attach(_m1);
	m2.attach(_m2);
	m3.attach(_m3);
	m4.attach(_m4);
	m5.attach(_m5);


	


}


void Servo_Scarborough::init_motors(){

	killed = false;
	m0.writeMicroseconds(1500); // send "stop" signal to ESC.
	m1.writeMicroseconds(1500); // send "stop" signal to ESC.
	m2.writeMicroseconds(1500); // send "stop" signal to ESC.
	m3.writeMicroseconds(1500); // send "stop" signal to ESC.
	m4.writeMicroseconds(1500); // send "stop" signal to ESC.
	m5.writeMicroseconds(1500); // send "stop" signal to ESC.
	
	delay(1000); //stop and wait


}

void Servo_Scarborough::stop_motors(){
  motors[0] = 1500;
  motors[1] = 1500;
  motors[2] = 1500;
  motors[3] = 1500;
  motors[4] = 1500;
  motors[5] = 1500;
  
  m0.writeMicroseconds(motors[0]);
  m1.writeMicroseconds(motors[1]);
  m2.writeMicroseconds(motors[2]);
  m3.writeMicroseconds(motors[3]);
  m4.writeMicroseconds(motors[4]);
  m5.writeMicroseconds(motors[5]);
  killed = true;
}

void Servo_Scarborough::okay_to_operate(){
	killed = false;
    m0.writeMicroseconds(motors[0]);
    m1.writeMicroseconds(motors[1]);
    m2.writeMicroseconds(motors[2]);
    m3.writeMicroseconds(motors[3]);
    m4.writeMicroseconds(motors[4]);
    m5.writeMicroseconds(motors[5]);
 
}

//servo speed is from 1100 to 1900. 1500 is 0 speed
void Servo_Scarborough::set_speed(int _motor, int signal){
	if(!killed){
		switch(_motor){
			case 0:
           motors[0] = signal;
			break;

			case 1:
			     motors[1] = signal;
			break;

			case 2:
				 motors[2] = signal;
			break;

			case 3:
				 motors[3] = signal;
			break;

			case 4:
				 motors[4] = signal;
			break;

			case 5:
				 motors[5] = signal;
			break;

			default:
			break;
			
		}
	}
 else{
    // do nothing
 }

}
