#include "main.h"


Peripherals_t::Peripherals_t(int left, int right, int arm, int claw) :
  left_mtr( left, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  right_mtr(right, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),
  arm_mtr(arm),
  claw_mtr(claw),
  master_controller(pros::E_CONTROLLER_MASTER){
    motor_list = {&left_mtr,&right_mtr,&arm_mtr,&claw_mtr};
  };
