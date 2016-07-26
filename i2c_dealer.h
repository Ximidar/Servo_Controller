#ifndef I2C_DEALER_H
#define I2C_DEALER_H

#include <Arduino.h>
#include <math.h>

//variable definitions
#define YAW 0
#define PITCH 1
#define ROLL 2
//add more when needed.

class i2c{

  public:
    i2c();
    void first_num(int variale,int reference[4]);
    void second_num(int variable,int reference[4]);

    //setters
    void set_y(double num);
    void set_p(double num);
    void set_r(double num);
   
    //getters
    double get_y();
    double get_p();
    double get_r();

  private:
    //variables
    bool switcher;
    bool switcher1;
    bool positive;
    double y; //yaw
    double p; //pitch
    double r; //roll 

    //todo add depth and desired variables.

};
#endif
