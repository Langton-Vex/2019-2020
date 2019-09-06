#ifndef _CHASSIS_H_
#define _CHASSIS_H_

struct Peripherals_t;
#include "main.h"

class Chassis{
  public:
    Chassis();
    void user_control();
    void set(int power,int turn);
    void move_forward(double distance,int velocity);
    void point_turn(double angle,int velocity);

    double turn_radius;
    double wheel_circumference;
  protected:
    //Peripherals_t peripherals;
    int motor_speed;
};

#endif
