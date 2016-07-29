#ifndef PID_DEALER_H
#define PID_DEALER_H

#include <Arduino.h>
#include <math.h>

//variable definitions
#define PROPORTIONAL 0
#define INTEGRAL 1
#define DERIVATIVE 2
//add more when needed.

class pid_dealer{

  public:
    pid_dealer();
    void first_num(int variale,int reference[4]);
    void second_num(int variable,int reference[4]);

    //setters
    void set_p(double num);
    void set_i(double num);
    void set_d(double num);
   
    //getters
    double get_p();
    double get_i();
    double get_d();

  private:
    //variables
    bool switcher;
    bool switcher1;
    bool positive;
    double p; //proportianal
    double i; //integral
    double d; //derivative

    //todo add depth and desired variables.

};
#endif
