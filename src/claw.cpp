
#include "main.h"

#define OPEN_POS 200
#define CLOSE_POS -200

Claw::Claw(){}

void Claw:: user_control(){
      int intake = peripherals.master_controller.get_digital_new_press(DIGITAL_UP);
      int eject = peripherals.master_controller.get_digital_new_press(DIGITAL_DOWN);

      power = (intake && (power < 1)) ? 127:0; // If we press the button, and
      power = (eject && (power > -1)) ? -127:0; // it is unpowered, power it
                                               // and vice versa

      this->set(power);
}

void Claw::set(int power){
  peripherals.leftintake_mtr.move_velocity(power);
    peripherals.rightintake_mtr.move_velocity(power);
  //peripherals.claw_mtr.move_absolute(position,127);
}
