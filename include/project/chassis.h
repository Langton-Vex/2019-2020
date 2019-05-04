#ifndef _CHASSIS_H_
#define _CHASSIS_H_

struct Peripherals_t;
#include "main.h"

class Chassis{
  public:
    Chassis(double turn_r, double w_circumference);
    void user_control();
    void set(int forward,int right,int turn);
    void move_forward(double distance,int velocity);
    void point_turn(double angle,int velocity);

    double turn_radius;
    double wheel_circumference;
  protected:
    //Peripherals_t peripherals;
    int tank_motor_speed;
    int strafe_motor_speed;
};

#endif
