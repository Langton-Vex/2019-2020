#include "main.h"
#include "math.h"
#include "stdexcept"

extern int8_t left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, intake_port, strafe_port;

std::shared_ptr<Chassis> Chassis::get() {
    static std::shared_ptr<Chassis> instance(new Chassis);
    return instance;
}

std::atomic<int> vision_distance;

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

Chassis::Chassis() {
    pros::motor_gearset_e_t motor_gearset = peripherals->left_mtr.get_gearing();
    if (motor_gearset == MOTOR_GEARSET_06)
        motor_speed = 600;
    if (motor_gearset == MOTOR_GEARSET_18)
        motor_speed = 200;
    if (motor_gearset == MOTOR_GEARSET_36)
        motor_speed = 100;
    //else throw std::invalid_argument("Cannot get gearset of left mtr");
}

void Chassis::user_control() {
    int power = peripherals->master_controller.get_analog(ANALOG_RIGHT_Y);
    int turn = peripherals->master_controller.get_analog(ANALOG_RIGHT_X);
    int slowmode_button = peripherals->master_controller.get_digital_new_press(DIGITAL_DOWN);
    int align_button = peripherals->master_controller.get_digital(DIGITAL_B);
    int strafe_left = peripherals->master_controller.get_digital(DIGITAL_R1);
    int strafe_right = peripherals->master_controller.get_digital(DIGITAL_R2);

    int strafe;
    if (strafe_left)
        strafe = -127;
    else if (strafe_right)
        strafe = 127;
    else
        strafe = 0;

    if (slowmode_button == 1)
        slowmode = !slowmode;
    //pros::lcd::print(5,"height per %f",arm.height_per);
    double power_mult = power_mult_calc();
    power_mult = (slowmode) ? 0.5 : power_mult;

    power = power * power_mult;
    turn = turn * power_mult;

    if (align_button)
        this->set(power, 0, vision_align());
    else
        this->set(power, turn, strafe);
}

double Chassis::power_mult_calc() {
    std::shared_ptr<Arm> arm = Arm::get();
    double power_mult = (arm->height_per < 0.5) ? (1.0 - arm->height_per) : 0.5;
    return power_mult;
}

void Chassis::modify_profiled_velocity(int velocity) {
    peripherals->left_mtr.modify_profiled_velocity(velocity);
    peripherals->lefttwo_mtr.modify_profiled_velocity(velocity);
    peripherals->right_mtr.modify_profiled_velocity(velocity);
    peripherals->righttwo_mtr.modify_profiled_velocity(velocity);
};

void Chassis::set(int power, int turn, int strafe) {

    int16_t powere = sgn(power) * motor_speed * pow(power / 127.0, 2); // exponential speed function
    int16_t turne = sgn(turn) * motor_speed * pow(turn / 127.0, 2);

    int left = powere + turne;
    int right = powere - turne;

    peripherals->left_mtr.move_voltage(left);
    peripherals->right_mtr.move_voltage(right);
    peripherals->lefttwo_mtr.move_voltage(left);
    peripherals->righttwo_mtr.move_voltage(right);
    if (strafe > -9999)
        peripherals->strafe_mtr.move(strafe);

    std::string left_v = std::to_string(peripherals->left_mtr.get_actual_velocity());
    std::string right_v = std::to_string(peripherals->right_mtr.get_actual_velocity());
}

// Currently set to only move strafe, as forward/backward align is unreliable / not necessary
int Chassis::vision_align() {

    //vision_signature_s_t sig = pros::Vision::signature_from_utility ( 1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0 );
    //vision::signature SIG_1 (1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0);

    pros::vision_object_s_t rtn = camera.get_by_size(0);
    if (camera.get_object_count() == 0)
        return -9999;

    if (rtn.width > 20) {
        vision_distance.store(190 - rtn.width);
        double strafeOut = strafePID.step(x_coord_filter.filter(rtn.x_middle_coord));
        return motor_speed * strafeOut;
    }
    return 0;
}
