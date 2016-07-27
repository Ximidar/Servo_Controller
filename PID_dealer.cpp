#include "PID_dealer.h"

pid_dealer::pid_dealer(){
  switcher = true;
  switcher1 = true;
  p=0;
  i=0;
  d=0;
  
}

//this takes the first part of the number and stores it to the correct variable
void pid_dealer::first_num(int variable, int reference[4]){
  
       
      switch(variable){

        //Yaw case
        case 0:          
          set_p((double)(reference[1] * 256) + reference[2]);
          switcher = false;
          break;
        //Pitch case
        case 1:
          set_i((reference[1] * 256) + reference[2]);
          switcher = false;
          break;
        //Roll case
        case 2:
          set_d((reference[1] * 256) + reference[2]);
          switcher = false;
          break;
        //default case
        default:
          switcher = false;
          break;
      }

}

//this takes the second part of the number then correctly adds it to the first number and stores it
void pid_dealer::second_num(int variable, int reference[4]){
 
      
      switch(variable){

        //Yaw case
        case 0:
          if(get_p() > 0){
            positive = true;
          }
          else{
            positive = false;
            //turn number positive to correctly add the decimal
            set_p(get_p() * -1);
          }
        
          set_p(get_p() + (((double)reference[1] * 256) + (double)reference[2]) / 1000);

          if(positive == false){
            //turn back to negative if number was originally negative
            set_p(get_p() * -1);
          }
          switcher1 = false;
          break;
        //Pitch case
        case 1:
          if(get_i() > 0){
            positive = true;
          }
          else{
            positive = false;
            set_i(get_i() * -1);
          }
        
          set_i(get_i() + (((double)reference[1] * 256) + (double)reference[2]) / 1000);

          if(positive == false){
            set_i(get_i() * -1);
          }
          switcher1 = false;
          break;
        //Roll case
        case 2:
          if(get_d() > 0){
            positive = true;
          }
          else{
            positive = false;
            set_d(get_d() * -1);
          }
        
          set_d(get_d() + (((double)reference[1] * 256) + (double)reference[2]) / 1000);

          if(positive == false){
            set_d(get_d() * -1);
          }
          switcher1 = false;
          break;
        //default case
        default:
          switcher1 = false;
          break;
      }
   
  
}

//getters and setters
void pid_dealer::set_p(double num){
  p = num;
}

double pid_dealer::get_p(){
  return p;
}

void pid_dealer::set_i(double num){
   i = num;
}

double pid_dealer::get_i(){
  return i;
}

void pid_dealer::set_d(double num){
   d = num;
}

double pid_dealer::get_d(){
  return d;
}

