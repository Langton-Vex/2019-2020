#ifndef _CHASSIS_H_
#define _CHASSIS_H_

struct Peripherals_t;
#include "main.h"

class Chassis {
public:
    static std::shared_ptr<Chassis> get();

    void user_control();
    void set(int power, int turn, int strafe);
    void move_forward(double distance, int velocity);
    void point_turn(double angle, int velocity);

    double power_mult_calc();
    void modify_profiled_velocity(int velocity);

    static void vision_align(void* param);
    std::unique_ptr<pros::Task> align_task;

    double turn_radius;
    double wheel_circumference;
    int motor_speed;

protected:
    Chassis();
    //Peripherals_t peripherals;

    bool slowmode = false;
};

#endif
