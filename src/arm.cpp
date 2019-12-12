#include "main.h"
#include <sstream>

const double max_height = -850;
const int pot_min = 1642;
const char pot_port = 'A';

pros::ADIAnalogIn arm_pot(pot_port);

using namespace okapi;

extern int left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, leftintake_port, rightintake_port;

extern std::shared_ptr<okapi::AsyncPosIntegratedController> lift;

void lift_stack(int cubes);

std::shared_ptr<Arm> Arm::get() {
    static std::shared_ptr<Arm> instance(new Arm);
    return instance;
}

Arm::Arm() {
    peripherals->leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    peripherals->rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    current_major_position = 0;
    current_minor_position = 0;
    user_pos_modifier = 0;
    sensitivity = 0.0001;
}

void Arm::user_control() {
    int power = peripherals->master_controller.get_analog(ANALOG_LEFT_Y);
    bool tare = peripherals->master_controller.get_digital_new_press(DIGITAL_X);
    //bool stack = peripherals->master_controller.get_digital_new_press(DIGITAL_DOWN);

    if (tare) {
        peripherals->leftarm_mtr.tare_position();
        peripherals->rightarm_mtr.tare_position();
    }

    height_per = scale(arm_pot.get_value(), 150, 1623, 0, 100) / 100;

    // Quick and dirty place to put this
    std::string arm_pos = std::to_string(peripherals->leftarm_mtr.get_position());
    std::string temp = std::to_string(peripherals->leftarm_mtr.get_temperature());
    temp.append(" celcius");
    pros::lcd::set_text(1, arm_pos);

    pros::lcd::set_text(2, temp);
    //double power_mult = (peripherals->leftarm_mtr.get_actual_velocity() > 1 &&
    //height_per < 0.2) ? 0.01:1;

    //double power_mult = 1;
    power = power /* * power_mult*/;
    if (abs(power) > 15) {
        if (!lift->isDisabled())
            lift->flipDisable(true);

        this->set(power);
        lift->setTarget(peripherals->leftarm_mtr.get_position());
    } else {
        if (lift->isDisabled())
            lift->flipDisable(false);
    }
}

void Arm::set(int power) {
    //if (abs(power) < 10) power = 5;
    peripherals->leftarm_mtr.move(power);
    peripherals->rightarm_mtr.move(power);
}
void Arm::set_pos(double position) {
    peripherals->leftarm_mtr.move_absolute(position, 63);
    peripherals->rightarm_mtr.move_absolute(position, 63);
}
//x is the number between current scale, a is the min of the new range, b is the max of the new range, min is the min of the current range, max is the max of the current range
double Arm::scale(double x, double min, double max, double a, double b) {
    return (a + b) - (((b - a) * (x - min)) / (max - min) + a);
}
