#include "main.h"


Peripherals_t::Peripherals_t(int left, int right, int lefttwo,int righttwo,
    int leftintake, int rightintake, int leftarm, int rightarm) :
  left_mtr( left, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  right_mtr(right, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),
  lefttwo_mtr( lefttwo, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  righttwo_mtr(righttwo, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),


  leftintake_mtr( leftintake, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  rightintake_mtr(rightintake, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),
  leftarm_mtr( leftarm, MOTOR_GEARSET_18,false,MOTOR_ENCODER_ROTATIONS),
  rightarm_mtr(rightarm, MOTOR_GEARSET_18,true, MOTOR_ENCODER_ROTATIONS),


  master_controller(pros::E_CONTROLLER_MASTER){

    left_port = left;
    right_port = right;
    lefttwo_port = lefttwo;
    righttwo_port = righttwo;
  };
