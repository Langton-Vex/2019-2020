#include "main.h"
#include "math.h"
#include "stdexcept"

okapi::AverageFilter<5> x_coord_filter;
pros::Vision camera(15, pros::E_VISION_ZERO_CENTER);
auto strafePID = okapi::IterativeControllerFactory::posPID(0.0071, 0.000000, 0.00005);
/*
okapi::EmaFilter x_coord_filter(1);
okapi::DemaFilter x_coord_filter(1,1);
okapi::MedianFilter<5> x_coord_filter;
auto straightPID = okapi::IterativeControllerFactory::posPID(0.0020, 0.000000, 0.0);
auto turnPID = okapi::IterativeControllerFactory::posPID(0.00200, 0.000000, 0.00089);
*/

std::shared_ptr<Chassis> Chassis::get() {
    static std::shared_ptr<Chassis> instance(new Chassis);
    return instance;
}

Chassis::Chassis() {
    pros::motor_gearset_e_t motor_gearset = peripherals->left_mtr.get_gearing();
    if (motor_gearset == MOTOR_GEARSET_06)
        motor_speed = 600.0;
    if (motor_gearset == MOTOR_GEARSET_18)
        motor_speed = 200.0;
    if (motor_gearset == MOTOR_GEARSET_36)
        motor_speed = 100.0;
    //else throw std::invalid_argument("Cannot get gearset of left mtr");
}

void Chassis::user_control() {
    float power = peripherals->master_controller.get_analog(ANALOG_RIGHT_Y);
    float turn = peripherals->master_controller.get_analog(ANALOG_RIGHT_X);

    int slowmode_button = peripherals->master_controller.get_digital_new_press(DIGITAL_B);
    int align_button = peripherals->master_controller.get_digital(DIGITAL_DOWN);
    int strafe_left = peripherals->master_controller.get_digital(DIGITAL_RIGHT);
    int strafe_right = peripherals->master_controller.get_digital(DIGITAL_Y);

    int strafe = (strafe_left) ? -200 : (strafe_right) ? 200 : 0;

    if (slowmode_button == 1)
        slowmode = !slowmode;




    if (align_button)
        this->set(power, 0, vision_align());
    else
        this->set(power, turn, strafe);
}

float Chassis::power_mult_calc() {
    std::shared_ptr<Arm> arm = Arm::get();
    float power_mult = (arm->height_per < 0.5) ? (1.0 - arm->height_per) : 0.5;
    return power_mult;
}

void Chassis::set(float power, float turn, float strafe) {
    float power_mult = (slowmode) ? 0.5 : power_mult_calc();

    float powere = std::copysign(power_mult * motor_speed * pow(power / 127.0, 2),power); // exponential speed function
    float turne =  std::copysign(power_mult * motor_speed * pow(turn / 127.0, 2),turn);

    int left = powere + turne;
    int right = powere - turne;

    peripherals->left_mtr.move_velocity(left);
    peripherals->right_mtr.move_velocity(right);
    peripherals->lefttwo_mtr.move_velocity(left);
    peripherals->righttwo_mtr.move_velocity(right);
    peripherals->strafe_mtr.move_velocity(strafe);
}

// Currently set to only move strafe, as forward/backward align is unreliable / not necessary
int Chassis::vision_align() {
    pros::vision_object_s_t rtn = camera.get_by_size(0);
    if (camera.get_object_count() == 0)
        return 0;

    if (rtn.width > 20) {
        float strafeOut = strafePID.step(x_coord_filter.filter(rtn.x_middle_coord));
        return motor_speed * strafeOut;
    }
    return 0;
}
