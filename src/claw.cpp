
#include "main.h"

std::shared_ptr<Claw> Claw::get() {
    static std::shared_ptr<Claw> instance(new Claw);
    return instance;
}

Claw::Claw() {
    peripherals->intake_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
}

void Claw::user_control() {
    int intake = 0;
    int eject = 0;

    int current_intake = peripherals->master_controller.get_digital(DIGITAL_L2);
    int current_eject = peripherals->master_controller.get_digital(DIGITAL_L1);

    if (current_intake)
        power = -127;

    else if (current_eject)
        power = 127;
    else
        power = 0;

    this->set(power);
}

void Claw::set(int power) {
    peripherals->intake_mtr.move(power);
    //peripherals->claw_mtr.move_absolute(position,127);
}
