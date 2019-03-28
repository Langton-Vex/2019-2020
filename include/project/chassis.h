#ifndef _CHASSIS_H_
#define _CHASSIS_H_

struct Peripherals_t;
#include "main.h"

class Chassis{
  public:
    Chassis();
    void user_control();
    void set(int power,int turn);
  protected:
    //Peripherals_t peripherals;
    int motor_speed;
};

#endif
