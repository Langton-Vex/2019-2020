#ifndef _PERIPHERALS_T_H_
#define _PERIPHERALS_T_H_

#include "api.h"

struct Peripherals_t{
  Peripherals_t(int left, int right, int lefttwo,int rightwo, int leftintake, int rightintake, int leftarm, int rightarm);
  pros::Motor left_mtr, right_mtr,lefttwo_mtr,righttwo_mtr,leftintake_mtr,rightintake_mtr,leftarm_mtr,rightarm_mtr;
  pros::Controller master_controller;
  int left_port, lefttwo_port, right_port, righttwo_port, leftarm_port, rightarm_port, leftintake_port, rightintake_port;
};

#endif
