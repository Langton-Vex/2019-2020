#include "main.h"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"
#include <sstream>

const auto max_height = 46 * okapi::inch;
const int pot_min = 1623;
const int pot_max = 150;
const char pot_port = 'A';
const okapi::IterativePosPIDController::Gains pot_controller_gains = { 0.0030, 0.0000, 0.00013 };

pros::ADIAnalogIn arm_pot(pot_port);

using namespace okapi;

extern int8_t left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, leftintake_port, rightintake_port;

std::shared_ptr<okapi::AsyncPositionController<double, double>> lift;
std::shared_ptr<okapi::AsyncPositionController<double, double>> integrated_lift;

void lift_stack(int cubes);

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

    integrated_lift = okapi::AsyncPosControllerBuilder()
                          .withMotor({ leftarm_port, rightarm_port })
                          .build();
    std::shared_ptr<okapi::Potentiometer> arm_pot = std::make_unique<okapi::Potentiometer>(pot_port);
    lift = okapi::AsyncPosControllerBuilder()
               .withMotor({ -leftarm_port, -rightarm_port })
               .withSensor(arm_pot)
               .withGains(pot_controller_gains)
               .withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory(5, 2, 250_ms))
               .build();
    lift->flipDisable(true);
    integrated_lift->flipDisable(true);
}

void Arm::user_control() {
    int power = peripherals->master_controller.get_analog(ANALOG_LEFT_Y);
    bool tare = peripherals->master_controller.get_digital_new_press(DIGITAL_X);
    //bool stack = peripherals->master_controller.get_digital_new_press(DIGITAL_DOWN);

    if (tare) {
        peripherals->leftarm_mtr.tare_position();
        peripherals->rightarm_mtr.tare_position();
    }

    if (!lift->isDisabled())
        lift->flipDisable(true);

    height_per = scale(arm_pot.get_value(), pot_min, pot_max, 0, 100) / 100;

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
        if (!integrated_lift->isDisabled())
            integrated_lift->flipDisable(true);

        this->set(power);
        double average_pos = (peripherals->leftarm_mtr.get_position() + peripherals->rightarm_mtr.get_position()) / 2.0;
        integrated_lift->setTarget(average_pos);
    } else if (integrated_lift->isDisabled())
        integrated_lift->flipDisable(false);
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

void Arm::set_height(okapi::QLength height) {
    int pot_delta = abs(pot_max - pot_min);
    double target = scale(height.convert(meter), (0.5 * okapi::inch).convert(meter), max_height.convert(meter), pot_min, pot_max);
    fprintf(stderr, "arm target: %f", target);
    lift->flipDisable(false);
    lift->setTarget((int)target);
}
void Arm::waitUntilSettled() {
    lift->waitUntilSettled();
    fprintf(stderr, "error: %f", lift->getError());
};
void Arm::flipDisable(bool disable) {
    lift->flipDisable(disable);
}

//x is the number between current scale, a is the min of the new range, b is the max of the new range, min is the min of the current range, max is the max of the current range
double Arm::scale(double x, double min, double max, double b, double a) {
    return (a + b) - (((b - a) * (x - min)) / (max - min) + a);
}

// I don't know where to put this but it can sit here for now and move in the rewrite.
void tune_arm() {
    std::shared_ptr<okapi::Potentiometer> arm_pot = std::make_unique<okapi::Potentiometer>(pot_port);
    auto arm_motors = std::make_shared<okapi::MotorGroup>(okapi::MotorGroup({ -leftarm_port, -rightarm_port }));
    // Initializer lists are funky

    auto ArmTuner = okapi::PIDTunerFactory::create(
        arm_pot, arm_motors, 7 * okapi::second, 500,
        0.001, 0.005, 0.000005, 0.00005, 0.00005, 0.0005,
        5, 4);
    auto tuning = ArmTuner.autotune();
    for (int i = 0; i < 5; i++) {
        printf("\nKp: %f, Ki: %f, Kd: %f\n", tuning.kP, tuning.kI, tuning.kD);
        pros::delay(2000);
    }
}
