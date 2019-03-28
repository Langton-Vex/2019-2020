#include "main.h"

#define OPEN_POS 200
#define CLOSE_POS -200

Claw::Claw(){
  peripherals.claw_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void Claw:: user_control(){
      int open = peripherals.master_controller.get_digital(DIGITAL_L1);
      int close = peripherals.master_controller.get_digital(DIGITAL_L2);
      int power = (open) ? OPEN_POS : (close)? CLOSE_POS:0; // If up, power = open_pos,
                                                  //elif down, power = close_pos
                                                  //  else, power = 0
      this->set(power);
}

void Claw::set(int power){
  peripherals.claw_mtr.move_velocity(power);
  //peripherals.claw_mtr.move_absolute(position,127);
}
