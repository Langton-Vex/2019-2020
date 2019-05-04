#include "main.h"


Peripherals_t::Peripherals_t(int left, int right, int strafe) :
  left_mtr( left, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  right_mtr(right, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),
  strafe_mtr(strafe, MOTOR_GEARSET_36,true, MOTOR_ENCODER_ROTATIONS),
  master_controller(pros::E_CONTROLLER_MASTER){};
