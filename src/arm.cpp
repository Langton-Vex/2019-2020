#include "main.h"

Arm::Arm(){
  peripherals.arm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void Arm:: user_control(){
      int power = peripherals.master_controller.get_analog(ANALOG_LEFT_Y);
      this->set(power);
}

void Arm::set(int power){
  peripherals.arm_mtr.move(power);

}
