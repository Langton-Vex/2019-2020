
#include "main.h"

#define OPEN_POS 200
#define CLOSE_POS -200

Claw::Claw(){}

void Claw:: user_control(){
      int intake = 0;
      int eject = 0;

      int current_intake = peripherals.master_controller.get_digital(DIGITAL_UP);
      int current_eject  = peripherals.master_controller.get_digital(DIGITAL_DOWN);
      if (current_intake && !intake_last)
        intake = 1;

      if (current_eject && !eject_last)
          eject = 1;
      intake_last = current_intake;
      eject_last = current_eject;

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
