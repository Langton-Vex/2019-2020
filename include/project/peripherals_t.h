#ifndef _PERIPHERALS_T_H_
#define _PERIPHERALS_T_H_

#include "api.h"
#include <vector>

struct Peripherals_t{
  Peripherals_t(int left, int right, int arm, int claw);
  pros::Motor left_mtr, right_mtr,arm_mtr,claw_mtr;
  pros::Controller master_controller;
  std::vector<pros::Motor*> motor_list;
};

#endif
