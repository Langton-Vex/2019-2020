
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



    double user_pos_modifier;
    double sensitivity;
  protected:
    //Peripherals_t peripherals;
    double current_goal_height;


};


#endif
