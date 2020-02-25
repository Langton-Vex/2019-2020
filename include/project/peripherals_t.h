#ifndef _PERIPHERALS_T_H_
#define _PERIPHERALS_T_H_

#include "api.h"
#include "okapi/api.hpp"

struct Peripherals_t {
    Peripherals_t(int left, int right, int lefttwo, int rightwo, int intake, int strafe, int leftarm, int rightarm,
        int leftenc_port, int rightenc_port, int midenc_port);
    pros::Motor left_mtr, right_mtr, lefttwo_mtr, righttwo_mtr, intake_mtr, strafe_mtr, leftarm_mtr, rightarm_mtr;
    okapi::ADIEncoder leftenc, rightenc, midenc;
    pros::Controller master_controller;
};

#endif
