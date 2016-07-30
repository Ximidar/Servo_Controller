
#include "i2c_dealer.h"

i2c::i2c(){
  switcher = true;
  switcher1 = true;
  y=0;
  p=0;
  r=0;
  
}

//this takes the first part of the number and stores it to the correct variable
void i2c::first_num(int variable, int reference[4]){
    while(switcher == true){
       
      switch(variable){

        //Yaw case
        case 0:          
          set_y((double)(reference[1] * 256) + reference[2]);
          switcher = false;
          break;
        //Pitch case
        case 1:
          set_p((reference[1] * 256) + reference[2]);
          switcher = false;
          break;
        //Roll case
        case 2:
          set_r((reference[1] * 256) + reference[2]);
          switcher = false;
          break;
        //default case
        default:
          switcher = false;
          break;
      }
    }
    switcher = true;
}

//this takes the second part of the number then correctly adds it to the first number and stores it
void i2c::second_num(int variable, int reference[4]){
  while(switcher1 == true){
      
      switch(variable){

        //Yaw case
        case 0:
          if(get_y() > 0){
            positive = true;
          }
          else{
            positive = false;
            //turn number positive to correctly add the decimal
            set_y(get_y() * -1);
          }
        
          set_y(get_y() + (((double)reference[1] * 256) + (double)reference[2]) / 1000);

          if(positive == false){
            //turn back to negative if number was originally negative
            set_y(get_y() * -1);
          }
          switcher1 = false;
          break;
        //Pitch case
        case 1:
          if(get_p() > 0){
            positive = true;
          }
          else{
            positive = false;
            set_p(get_p() * -1);
          }
        
          set_p(get_p() + (((double)reference[1] * 256) + (double)reference[2]) / 1000);

          if(positive == false){
            set_p(get_p() * -1);
          }
          switcher1 = false;
          break;
        //Roll case
        case 2:
          if(get_r() > 0){
            positive = true;
          }
          else{
            positive = false;
            set_r(get_r() * -1);
          }
        
          set_r(get_r() + (((double)reference[1] * 256) + (double)reference[2]) / 1000);

          if(positive == false){
            set_r(get_r() * -1);
          }
          switcher1 = false;
          break;
        //default case
        default:
          switcher1 = false;
          break;
      }
    }
    switcher1 = true;
  
}

//getters and setters
void i2c::set_y(double num){
  y = num;
}

double i2c::get_y(){
  return y;
}

void i2c::set_p(double num){
   p = num;
}

double i2c::get_p(){
  return p;
}

void i2c::set_r(double num){
   r = num;
}

double i2c::get_r(){
  return r;
}

