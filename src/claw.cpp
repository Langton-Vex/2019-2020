
#include "main.h"

#define OPEN_POS 200
#define CLOSE_POS -200

Claw::Claw(){}

void Claw:: user_control(){
      int intake = 0;
      int eject = 0;

      int current_intake = peripherals.master_controller.get_digital(DIGITAL_L1);
      int current_eject  = peripherals.master_controller.get_digital(DIGITAL_L2);
      if (current_intake && !intake_last)
        intake = 1;

      if (current_eject && !eject_last)
          eject = 1;
      intake_last = current_intake;
      eject_last = current_eject;

      if (intake && (power < 1))
        power = 63;
      else if (intake && (power > 1))
        power = 0;
      else if (eject && (power > -1))
        power = -63;
     else if (eject && (power < -1))
        power = 0;

      this->set(power);
}

void Claw::set(int power){
  peripherals.leftintake_mtr.move(power);
    peripherals.rightintake_mtr.move(power);
  //peripherals.claw_mtr.move_absolute(position,127);
}
