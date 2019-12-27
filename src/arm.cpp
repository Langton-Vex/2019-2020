#include "main.h"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"
#include <sstream>

using namespace okapi;

const auto max_height = 46 * okapi::inch;
const int pot_min = 1623;
const int pot_max = 150;
const char pot_port = 'A';
const okapi::IterativePosPIDController::Gains pot_controller_gains = { 0.0030, 0.0000, 0.00013 };

pros::ADIAnalogIn arm_pot(pot_port);

std::shared_ptr<okapi::AsyncPositionController<double, double>> pos_controller;
std::shared_ptr<okapi::AsyncPositionController<double, double>> balance_controller;

std::shared_ptr<Arm> Arm::get() {
    static std::shared_ptr<Arm> instance(new Arm);
    return instance;
}

Arm::Arm() {
    current_major_position = 0;
    current_minor_position = 0;
    user_pos_modifier = 0;
    sensitivity = 0.0001;
}

void Arm::init() {
    peripherals->leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    peripherals->rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    balance_controller = okapi::AsyncPosControllerBuilder()
                             .withMotor(rightarm_port)
                             .build();

    std::shared_ptr<okapi::Potentiometer> arm_pot = std::make_unique<okapi::Potentiometer>(pot_port);

    pos_controller = okapi::AsyncPosControllerBuilder()
                         .withMotor({ (int8_t)-leftarm_port, (int8_t)-rightarm_port })
                         .withSensor(arm_pot)
                         .withGains(pot_controller_gains)
                         .withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory(5, 2, 250_ms))
                         .build();

    pos_controller->flipDisable(true);
}

void Arm::user_control() {
    int power = peripherals->master_controller.get_analog(ANALOG_LEFT_Y);
    bool tare = peripherals->master_controller.get_digital_new_press(DIGITAL_X);

    if (tare) {
        peripherals->leftarm_mtr.tare_position();
        peripherals->rightarm_mtr.tare_position();
    }

    if (!pos_controller->isDisabled())
        pos_controller->flipDisable(true);

    height_per = scale(arm_pot.get_value(), pot_min, pot_max, 0, 100) / 100;

    if (abs(power) > 15) {
        if (!balance_controller->isDisabled())
            balance_controller->flipDisable(true);

        this->set(power);
        balance_controller->setTarget(peripherals->leftarm_mtr.get_position());
    } else if (balance_controller->isDisabled())
        balance_controller->flipDisable(false);
}

void Arm::set(int power) {
    peripherals->leftarm_mtr.move(power);
    peripherals->rightarm_mtr.move(power);
}

void Arm::set_height(okapi::QLength height) {
    int pot_delta = abs(pot_max - pot_min);
    double target = scale(height.convert(meter), (0.5 * okapi::inch).convert(meter), max_height.convert(meter), pot_min, pot_max);
    fprintf(stderr, "arm target: %f", target);
    pos_controller->flipDisable(false);
    balance_controller->flipDisable(true);

    pos_controller->setTarget((int)target);
}
void Arm::waitUntilSettled() {
    pos_controller->waitUntilSettled();
}

void Arm::flipDisable(bool disable) {
    pos_controller->flipDisable(disable);
}

//x is the number between current scale, a is the min of the new range, b is the max of the new range, min is the min of the current range, max is the max of the current range
double Arm::scale(double x, double min, double max, double b, double a) {
    return (a + b) - (((b - a) * (x - min)) / (max - min) + a);
}
