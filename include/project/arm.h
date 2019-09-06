
#ifndef _ARM_H_
#define _ARM_H_

struct Peripherals_t;
#include "main.h"

class Arm{
  public:
    Arm();
    void user_control();
    void set(int power);
    void set_pos(float position);
  protected:
    //Peripherals_t peripherals;
    int current_power;
  
};


#endif
