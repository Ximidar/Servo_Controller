#ifndef POWER_H
  #define POWER_H

  #define POWER_OFF true
  #define POWER_ON false

  class Power{
    public:

      Power(int killSwitch_pin, int relay1_pin, int relay2_pin, int relay3_pin);
      bool return_killswitch();
      void set_killswitch(bool _killSwitch);
      void monitor_killswitch();

    private:
      int killPin;
      int relay1;
      int relay2;
      int relay3;
      bool killSwitch;
  };

  #endif
