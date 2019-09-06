
#include "main.h"

Arm::Arm(){
  peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void Arm:: user_control(){
      int power = peripherals.master_controller.get_analog(ANALOG_LEFT_Y);
      //int pos_joy = peripherals.master_controller.get_analog(ANALOG_LEFT_Y);
      //float pos_rotations = (float)pos_joy / (127 * 4); // figure out the bounds of the arm,
                                                         //  and set appropriate values

      this->set(power);
}

void Arm::set(int power){
  //if (abs(power) < 10) power = 5;
  peripherals.leftarm_mtr.move(power);
  peripherals.rightarm_mtr.move(power);

}
void Arm::set_pos(float position){
  peripherals.leftarm_mtr.move_absolute(position,127);
    peripherals.rightarm_mtr.move_absolute(position,127);
}
