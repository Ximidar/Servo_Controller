#include "power.h"
#include <Arduino.h>

Power::Power(int killSwitch_pin, int relay1_pin, int relay2_pin, int relay3_pin){

  // init killswitch to false on object creation
  killSwitch = POWER_OFF;

  //set killPin to the pin we want the killswitch connected to
  killPin = killSwitch_pin;

  //set relays;
  relay1 = relay1_pin;
  relay2 = relay2_pin;
  relay3 = relay3_pin;
  
  //set up pins to recieve input/output
  pinMode(killPin, INPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  
}

void Power::monitor_killswitch(){
  int killer = digitalRead(killPin);

  if(killer == 0){
    set_killswitch(POWER_ON);
    
  }
  if(killer == 1){
    set_killswitch(POWER_OFF);
  }
}

bool Power::return_killswitch(){
  return killSwitch;
}

void Power::set_killswitch(bool _killSwitch){
  killSwitch = _killSwitch;

  if(killSwitch == POWER_OFF){
    //write to the pins that control power to turn off
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, HIGH);
  }
  else if(killSwitch == POWER_ON){
    //write to pins to turn power on
     digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    digitalWrite(relay3, HIGH);
  }
}

