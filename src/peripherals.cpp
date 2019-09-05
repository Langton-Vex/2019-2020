#include "main.h"


Peripherals_t::Peripherals_t(int left, int right, int lefttwo, int righttwo) :
  left_mtr( left, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  right_mtr(right, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),
  lefttwo_mtr( lefttwo, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  righttwo_mtr(righttwo, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),

  master_controller(pros::E_CONTROLLER_MASTER){};
