
#include "main.h"

std::shared_ptr<Claw> Claw::get() {
    static std::shared_ptr<Claw> instance(new Claw);
    return instance;
}

Claw::Claw() {
    peripherals->intake_mtr.set_brake_mode(MOTOR_BRAKE_COAST);
}

void one_drop() {
    const int drop_time = 168;
    const int grab_time = 20;
    peripherals->intake_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
    peripherals->intake_mtr.move_velocity(200);
    pros::delay(grab_time);
    peripherals->intake_mtr.move_velocity(0);
    pros::delay(drop_time - (2 * grab_time)); // 66 is the magic number from suvat
    peripherals->intake_mtr.move(-127);
    pros::delay(50);
    peripherals->intake_mtr.set_brake_mode(MOTOR_BRAKE_COAST);
}

void Claw::user_control() {
    int intake = 0;
    int eject = 0;

    int current_intake = peripherals->master_controller.get_digital(DIGITAL_L1);
    int current_eject = peripherals->master_controller.get_digital(DIGITAL_L2);

    if (current_intake)
        power = -127;

    else if (current_eject)
        power = 127;
    else
        power = 0;

    if (peripherals->master_controller.get_digital(DIGITAL_RIGHT)) {
        one_drop();
    }

    this->set(power);
}

void Claw::set(int power) {
    peripherals->intake_mtr.move(power);
    //peripherals->claw_mtr.move_absolute(position,127);
}
