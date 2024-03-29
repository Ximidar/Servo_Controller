#include <i2c_t3.h>
#include "Depth_Sensor.h"
#include "i2c_dealer.h"
#include "power.h"
#include "Servo_Scarborough.h"
                        //_m0, _m1, _m2, _m3, _m4,  _m5
Servo_Scarborough servos( 7    ,3,  4,   5,   6,   2);
//we may want to change pins 5, 7, and 8 as they lie on the RX TX lines
//At least pin 5 as it lies on the RX1 line which may be being used at startup.
double depth_in = 0.00;
double depth_last = depth_in;
int reference[3];
int counter = 0;
Power power(1);//Killswitch is Digital Pin 1
MS5837 depth_Sensor;
String sender = "";
int reg = 0;
int led = 13;

void setup() {  

  //set up i2c slave on teensy pins 18 and 19 
  Wire.begin(I2C_SLAVE, 0x04, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100); //set up the teensy as a slave
  Wire.onReceive(readROS); //function for handling receiving instructions
  Wire.onRequest(writeROS); //function for handling writing to ROS

  //set up Wire1 as a master
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_INT, I2C_RATE_400);
  servos.init_motors();


  //initialize depth sensor
  depth_Sensor.init();
  depth_Sensor.setFluidDensity(997);
  
  

  //set up led
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  delay(200);
  digitalWrite(led, HIGH);

  //Serial.begin(115200);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Main Loop////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){
  monitor_power();
}

void read_depth_sensor(){
  depth_Sensor.read();
  depth_in = depth_Sensor.depth() * (3.28084 / 1);//convert meters to feet

}

void monitor_power(){
  if(power.return_killswitch()){

   
    servos.okay_to_operate();
    //monitor the killSwtich
    power.monitor_killswitch();
    digitalWrite(led, HIGH);
    
  }
  else{
    servos.stop_motors();
    //monitor the killSwtich
    power.monitor_killswitch();
    digitalWrite(led, LOW);
   
  }

}

// read ROS is used to write to the registers that are in the switch statement.
void readROS(size_t byteC){
  int count = 0;
  for(int i = 0; i < 2 ; i++){
     reference[i] = 0;
  }

  //This while loop reads all available bytes over the i2c bus then stores them into reference[]
  //reference is composed of three values, reference[0] is the addressed register, then reference[1] and reference[2] are the numbers for the register
  //look further on to see how to extract the number out of reference.
  while(Wire.available()){

      reference[count] = Wire.read();
      count++;

  }

  
    switch(reference[0]){
      ///////////////////////////////////Write Requests//////////////////////////////


      ///////////////////////////////////motor Writes/////////////////////////////////
      //first number for yaw
      //low | (high<<8)
      case 10:       
        servos.set_speed(0, reference[2] | (reference[1]<<8) );
        //Serial.println(reference[1]);
        //Serial.println(reference[2]);
        //Serial.println(reference[2] | (reference[1]<<8));
        break;
        
     //second number for yaw
      case 11:        
        servos.set_speed(1, reference[2] | (reference[1]<<8) );        
        break;
        
      //first number for pitch
      case 12:        
        servos.set_speed(2, reference[2] | (reference[1]<<8) );        
        break;
        
      //second number for pitch
      case 13:        
        servos.set_speed(3, reference[2] | (reference[1]<<8) );        
        break;
        
      //first number for roll
      case 14:        
        servos.set_speed(4, reference[2] | (reference[1]<<8) );        
        break;
        
      //second number for roll
      case 15:        
       servos.set_speed(5, reference[2] | (reference[1]<<8) );        
        break;

      ////////////////////////////Read Requests///////////////////////////  
    
      //read power state
      case 57:
        reg = 7;
        //monitor_power();
        break;
      //read current depth
      case 58:
        reg = 8;
        read_depth_sensor();
        break;


      //////////////////////////////////////////DEFAULT///////////////////////////////////////////////////// 
      //in case something messes up heres a default value that will be hit
      default:          
        break;
    }
  
}

//This function writes motor values to ROS with strings
void writeROS(){
 
  //initialize number to be sent.
  byte package[2];
  byte low = 0;
  byte high = 0;

 
  //////////////////////////////////////Power Reads/////////////////////////////////////////////////
  if(reg == 7){
    int kill = 999; // default kill to a large number. if this is seen on the other side its a mistake;
    if(power.return_killswitch() == POWER_OFF){
      kill = 0;
    }
    else if(power.return_killswitch() == POWER_ON){
      kill = 1;
    }
    //split the killswitch into low/high bytes
    low = kill & 0xff;
    high = (kill >> 8) & 0xff;
    //package
    package[0] = low;
    package[1] = high;
    //write
    Wire.write(package, 2);
  }
  else if(reg == 8){
    int16_t depth = (int16_t)(depth_in * 1000);
    //split the killswitch into low/high bytes
    low = depth & 0xff;
    high = (depth >> 8) & 0xff;
    //package
    package[0] = low;
    package[1] = high;
    //write
    Wire.write(package, 2);
  }

    
}



