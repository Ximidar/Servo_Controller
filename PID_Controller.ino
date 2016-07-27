#include <i2c_t3.h>

#include "Depth_Sensor.h"
#include "i2c_dealer.h"
#include "PID_v1.h"
#include "power.h"
#include "PID_dealer.h"

i2c i2c;
Power power(2,3,4,6);
MS5837 depth_Sensor;
pid_dealer yaw_pid;
pid_dealer pitch_pid;
pid_dealer roll_pid;
pid_dealer depth_pid;

//variables for the i2c exchange
int reference [4] = {0,0,0,0};
bool positive;
String sender = "";
int reg = 0;


//PID values for YPR
double yaw_desired, pitch_desired, roll_desired, depth_desired,
       yaw_in,      pitch_in,      roll_in,      depth_in,
       yaw_out,     pitch_out,     roll_out,     depth_out;
     

//values for throttle
double throttle;

//values for final motor speed
double m1, m2, m3, m4, m5, m6;

// PID Values for YPRD
double yaw_kp = 0.0, yaw_ki = 0.0, yaw_kd = 0.0;

//PID for Pitch and Roll
double pitch_kp = 0.0, pitch_ki = 0.0, pitch_kd = 0.0;
double roll_kp = 0.0, roll_ki = 0.0, roll_kd = 0.0;

//PID for depth
double depth_kp = 0.0, depth_ki = 0.0 , depth_kd = 0.0;

//double for the max output
double outMIN, outMAX;

//constants that are not really constants yet
double YAW_CONST = 1, 
       PITCH_CONST = 1,
       ROLL_CONST = 1,
       DEPTH_CONST = 1;

//set up LED so we know the board is working
int led = 13;

//All PID being used
PID yaw(&yaw_in, &yaw_out, &yaw_desired, yaw_kp, yaw_ki, yaw_kd, DIRECT);
PID pitch(&pitch_in, &pitch_out, &pitch_desired, pitch_kp, pitch_ki, pitch_kd, DIRECT);
PID roll(&roll_in, &roll_out, &roll_desired, roll_kp, roll_ki, roll_kd, DIRECT);
PID depth(&depth_in, &depth_out, &depth_desired, depth_kp, depth_ki, depth_kd, DIRECT);


void setup() {

  //set up i2c slave on teensy pins 18 and 19 
  Wire.begin(I2C_SLAVE, 0x04, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100); //set up the teensy as a slave
  Wire.onReceive(readROS); //function for handling receiving instructions
  Wire.onRequest(writeROS); //function for handling writing to ROS

  //set up Wire1 as a master
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_INT, I2C_RATE_400);



  //initialize depth sensor
  depth_Sensor.init();
  depth_Sensor.setFluidDensity(997);
  
  //initialize desired values for all axises
  yaw_desired = 0;
  pitch_desired = 0;
  roll_desired = 0;
  depth_desired = 0;
  outMIN = -32000.00;
  outMAX = 32000.00;

  //set limits for all outputs
  yaw.SetOutputLimits(-100.00, 100.00);
  pitch.SetOutputLimits(-100.00, 100.00);
  roll.SetOutputLimits(-100.00, 100.00);
  depth.SetOutputLimits(-100.00,100.00);

  //turn on all calcutrons
  yaw.SetMode(AUTOMATIC);
  pitch.SetMode(AUTOMATIC);
  roll.SetMode(AUTOMATIC);
  depth.SetMode(AUTOMATIC);

  //initialize throttle
  throttle = 0;

  //set up led
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Main Loop////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  if(!power.return_killswitch()){
    //get reference for YPR
    yaw_in = i2c.get_y();  
    pitch_in = i2c.get_p();
    roll_in = i2c.get_r();

    //get reference for depth
    depth_Sensor.read();
    depth_in = depth_Sensor.depth() * (3.28084 / 1);//convert meters to feet

    yaw.Compute();
    pitch.Compute();
    roll.Compute();
    depth.Compute();

    //calculate the final motor speeds
    //left/right
    m1 = (-YAW_CONST * (yaw_out/100.00 * 32000.00)) + (PITCH_CONST * (pitch_out/100.00 * 32000.00)) + throttle;
    m2 = (YAW_CONST * (yaw_out/100.00 * 32000.00)) + (PITCH_CONST * (pitch_out/100.00 * 32000.00)) + throttle;

    //dive motors
    m3 = (PITCH_CONST * (pitch_out/100.00 * 32000.00)) + (ROLL_CONST *(roll_out/100.00 * 32000.00)) + (DEPTH_CONST * (depth_out / 100.00 * 32000.00));
    m4 = (PITCH_CONST * (pitch_out/100.00 * 32000.00)) + (-ROLL_CONST *(roll_out/100.00 * 32000.00)) + (DEPTH_CONST * (depth_out / 100.00 * 32000.00));
    m5 = (-PITCH_CONST * (pitch_out/100.00 * 32000.00)) + (-ROLL_CONST *(roll_out/100.00 * 32000.00)) + (DEPTH_CONST * (depth_out / 100.00 * 32000.00));
    m6 = (-PITCH_CONST * (pitch_out/100.00 * 32000.00)) + (ROLL_CONST *(roll_out/100.00 * 32000.00)) + (DEPTH_CONST * (depth_out / 100.00 * 32000.00));

    //do a final check to see if motor speeds are valid
    m1 = clamper(m1);
    m2 = clamper(m2);
    m3 = clamper(m3);
    m4 = clamper(m4);
    m5 = clamper(m5);
    m6 = clamper(m6);
  
    //monitor the killSwtich
    power.monitor_killswitch();
  }
  else{
    m1 = 0;
    m2 = 0;
    m3 = 0;
    m4 = 0;
    m5 = 0;
    m6 = 0;
    depth_in = 0;
    //monitor the killSwtich
    power.monitor_killswitch(); 
  }

    
}

// read ROS is used to write to the registers that are in the switch statement.
void readROS(size_t byteC){
  int count = 0;
  for(int i = 0; i < 4 ; i++){
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


      ///////////////////////////////////YPR WRITES/////////////////////////////////
     //first number for yaw
      case 0:       
        i2c.first_num(YAW, reference);        
        break;
        
     //second number for yaw
      case 1:        
        i2c.second_num(YAW, reference);        
        break;
        
      //first number for pitch
      case 2:        
        i2c.first_num(PITCH, reference);        
        break;
        
      //second number for pitch
      case 3:        
        i2c.second_num(PITCH, reference);        
        break;
        
      //first number for roll
      case 4:        
        i2c.first_num(ROLL, reference);        
        break;
        
      //second number for roll
      case 5:        
        i2c.second_num(ROLL, reference);        
        break;


      ///////////////////////////////////////////PID WRITES///////////////////////////////
      //update kp
      case 6:        
        //kp = (double)(reference[1] * 256) + reference[2];        
        break;
        
      //update ki
      case 7:        
        //ki = (double)(reference[1] * 256) + reference[2];        
        break;
        
      //update kd
      case 8:        
        //kd = (double)(reference[1] * 256) + reference[2];        
        break;

        


      /////////////////////////////////DESIRED WRITES///////////////////////////////
      //update desired yaw
      case 9:        
        yaw_desired = (double)(reference[1] * 256) + reference[2];
        
        break;        
       
      //update desired pitch
      case 10:
        pitch_desired = (double)(reference[1] * 256) + reference[2];
      break;

      //update desired roll
      case 11:
        roll_desired = (double)(reference[1] * 256) + reference[2];
      break;

       //update desired depth
      case 12:        
        depth_desired = (double)(reference[1] * 256) + reference[2];
      break;
        
      //update throttle
      case 13:
        throttle = (double)(reference[1] * 256) + reference[2];
      break;


      /////////////////////////////////POWER WRITES/////////////////////////////////
      //enable or disable power
      case 14:
        //if the reference == 0 disable power
        if(  ((double)(reference[1] * 256) + reference[2]) == 0 ){
          power.set_killswitch(POWER_OFF);
        }//otherwise if reference = 1 enable power
        else if(  ((double)(reference[1] * 256) + reference[2]) == 1 ){
          power.set_killswitch(POWER_ON);
        }        
      break;

      ///////////////////////////////////PID writes///////////////////////////////

      
      //yaw P
      case 15:
        yaw_pid.first_num(PROPORTIONAL, reference);

      break;

      case 16:
        yaw_pid.second_num(PROPORTIONAL, reference);
        yaw_kp = yaw_pid.get_p();
      break;
      //Pitch I
      case 17:
        yaw_pid.first_num(INTEGRAL, reference);

      break;

      case 18:
        yaw_pid.second_num(INTEGRAL, reference);
        yaw_ki = yaw_pid.get_i();
      break;

      //pitch D
      case 19:
        yaw_pid.first_num(DERIVATIVE, reference);
      break;

      case 20:
        yaw_pid.second_num(DERIVATIVE, reference);
        yaw_kd = yaw_pid.get_d();
      break;
      /////////////////////////////////////////////pitch
      case 21:
        pitch_pid.first_num(PROPORTIONAL, reference);
      break;

      case 22:
        pitch_pid.second_num(PROPORTIONAL, reference);
        pitch_kp = pitch_pid.get_p();
      break;

      case 23:
         pitch_pid.first_num(INTEGRAL, reference);
      break;

      case 24:
        pitch_pid.second_num(INTEGRAL, reference);
        pitch_ki = pitch_pid.get_i();
      break;

      case 25:
        pitch_pid.first_num(DERIVATIVE, reference);
      break;

      case 26:
         pitch_pid.second_num(DERIVATIVE, reference);
         pitch_pid.get_d();
      break;
      ///////////////////////////////////////////////roll
      case 27:
        roll_pid.first_num(PROPORTIONAL, reference);
      break;

      case 28:
         roll_pid.second_num(PROPORTIONAL, reference);
         roll_kp = roll_pid.get_p();
      break;

      case 29:
        roll_pid.first_num(INTEGRAL, reference);
      break;

      case 30:
        roll_pid.second_num(INTEGRAL, reference);
        roll_ki = roll_pid.get_i();
      break;

      case 31:
        roll_pid.first_num(DERIVATIVE, reference);
      break;

      case 32:
       roll_pid.second_num(DERIVATIVE, reference);
       roll_kd = roll_pid.get_d();
      break;

      case 33:
        ////////////////////////////////////////////////depth
        depth_pid.first_num(PROPORTIONAL, reference);
      break;

      case 34:
        depth_pid.second_num(PROPORTIONAL, reference);
        depth_kp = depth_pid.get_p();
      break;
      
      case 35:
        depth_pid.first_num(INTEGRAL, reference);
      break;

      case 36:
        depth_pid.second_num(INTEGRAL, reference);
        depth_ki = depth_pid.get_i();
      break;
      
      case 37:
        depth_pid.first_num(DERIVATIVE, reference);
      break;
      
      case 38:
        depth_pid.second_num(DERIVATIVE, reference);
        depth_kd = depth_pid.get_d();
      break;


      

      ////////////////////////////Read Requests///////////////////////////  
      //motor1
      case 51:
        reg = 1;        
        break;
      //motor2
      case 52:
        reg = 2;        
        break;
      //motor3
      case 53:
        reg = 3;        
        break;
      //motor4
      case 54:
        reg = 4;        
        break;
      //motor5
      case 55:
        reg = 5;        
        break;
      //motor 6
      case 56:
        reg = 6;        
        break;
      //read power state
      case 57:
        reg = 7;
        break;
      //read current depth
      case 58:
        reg = 8;
        break;


      //////////////////////////////////////////DEFAULT///////////////////////////////////////////////////// 
      //in case something messes up heres a default value that will be hit
      default:          
        break;
    }
  
}

//This function writes motor values to ROS with strings
void writeROS(){
 
  sender = "";

  //depending on the value of reg different values will be sent
  if(reg == 1){
    sender = "M1:" + String(m1,0) + ";";  
    Wire.write(sender.c_str());
  }
  
  else if(reg == 2){
     sender = "M2:"+ String(m2,0) + ";";
     Wire.write(sender.c_str());
  }
  else if(reg == 3){
    sender = "M3:" + String(m3,0) + ";";
    Wire.write(sender.c_str());
  }
  else if(reg == 4){
    sender = "M4:" + String(m4,0) + ";";
    Wire.write(sender.c_str());
  }
  else if(reg == 5){
    sender = "M5:" + String(m5,0) + ";";
    Wire.write(sender.c_str());
  }
  else if(reg == 6){
    sender = "M6:" + String(m6,0) + ";";
    Wire.write(sender.c_str());
  }
  //////////////////////////////////////Power Reads/////////////////////////////////////////////////
  else if(reg == 7){
    int kill = 999; // default kill to a large number. if this is seen on the other side its a mistake;
    if(power.return_killswitch() == POWER_OFF){
      kill = 0;
    }
    else if(power.return_killswitch() == POWER_ON){
      kill = 1;
    }
    sender = "K:" + String(kill,0) + ";";
    Wire.write(sender.c_str());
  }
  else if(reg == 8){
    sender = "D:" + String(depth_in,2) + ";";
    Wire.write(sender.c_str());
  }

    
}

double clamper(double output){

  if(output > outMAX){
   output = outMAX;
  }
  else if(output < outMIN){
    output = outMIN;
  }

  return output;
  
}



