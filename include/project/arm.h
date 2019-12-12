
#ifndef _ARM_H_
#define _ARM_H_

struct Peripherals_t;
#include "main.h"

class Arm {
public:
    static std::shared_ptr<Arm> get();

    void user_control();
    void set(int power);
    void set_pos(double position);
    double scale(double x, double min, double max, double a, double b);

    unsigned int current_major_position;
    unsigned int current_minor_position;

    double user_pos_modifier;
    double sensitivity;
    double height_per;

protected:
    Arm();
    //Peripherals_t peripherals;
    double current_goal_height;
};

#endif
