#include "main.h"

Peripherals_t::Peripherals_t(int left, int right, int lefttwo, int righttwo,
    int intake, int strafe, int leftarm, int rightarm) :

    left_mtr(left, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES),
    right_mtr(right, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES),
    lefttwo_mtr(lefttwo, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES),
    righttwo_mtr(righttwo, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES),

    intake_mtr(intake, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS),
    strafe_mtr(strafe, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS),

    leftarm_mtr(leftarm, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES),
    rightarm_mtr(rightarm, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES),

    master_controller(pros::E_CONTROLLER_MASTER){};
