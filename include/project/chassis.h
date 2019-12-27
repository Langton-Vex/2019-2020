#ifndef _CHASSIS_H_
#define _CHASSIS_H_

struct Peripherals_t;
#include "main.h"

class Chassis {
public:
    static std::shared_ptr<Chassis> get();

    void user_control();
    void set(float power, float turn, float strafe);

    double power_mult_calc();

    int vision_align();
    std::unique_ptr<pros::Task> align_task;

    double turn_radius;
    double wheel_circumference;
    float motor_speed;

protected:
    Chassis();
    //Peripherals_t peripherals;

    bool slowmode = false;
};

#endif
