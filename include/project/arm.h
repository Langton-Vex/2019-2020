
#ifndef _ARM_H_
#define _ARM_H_

struct Peripherals_t;
#include "main.h"

class Arm{
  public:
    Arm();
    void user_control();
    void set(int power);
    void set_pos(double position);

    unsigned int current_major_position;
    unsigned int current_minor_position;

    double major_positions[4] = {0.0,0.5,1,1.5};
    double minor_positions[4] = {0.0,0.1,0.2,0.2};

    double user_pos_modifier;
    double sensitivity;
  protected:
    //Peripherals_t peripherals;
    int current_power;
    int L1_last = 0;
    int L2_last = 0;


};


#endif
