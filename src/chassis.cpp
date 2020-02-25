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
auto strafePID = okapi::IterativeControllerFactory::posPID(0.0071, 0.000000, 0.0001);
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
    strafePID.setSampleTime(20 * okapi::millisecond);
}

void Chassis::user_control() {
    int power = peripherals->master_controller.get_analog(ANALOG_RIGHT_Y);
    int turn = peripherals->master_controller.get_analog(ANALOG_RIGHT_X);
    int slowmode_button = peripherals->master_controller.get_digital_new_press(DIGITAL_LEFT);
    int align_button = peripherals->master_controller.get_digital(DIGITAL_B);
    int strafe_left = peripherals->master_controller.get_digital(DIGITAL_R2);
    int strafe_right = peripherals->master_controller.get_digital(DIGITAL_R1);
    int hold_button = peripherals->master_controller.get_digital_new_press(DIGITAL_UP);

    int strafe;
    if (strafe_left)
        strafe = -127;
    else if (strafe_right)
        strafe = 127;
    else
        strafe = 0;

    if (hold_button)
        holdmode = !holdmode;

    if (peripherals->left_mtr.get_temperature() >= 50 || peripherals->strafe_mtr.get_temperature() >= 50) {
        holdmode = false;
    }

    if (slowmode_button == 1)
        slowmode = !slowmode;
    //pros::lcd::print(5,"height per %f",arm.height_per);

    if (holdmode) {
        peripherals->left_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        peripherals->right_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        peripherals->lefttwo_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        peripherals->righttwo_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        peripherals->strafe_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    } else {
        peripherals->left_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        peripherals->right_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        peripherals->lefttwo_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        peripherals->righttwo_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        peripherals->strafe_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    if (align_button) {
        this->set(power, 0, vision_align());
    } else
        this->set(power, turn, strafe);
}

double Chassis::power_mult_calc() {
    std::shared_ptr<Arm> arm = Arm::get();
    double power_mult = (arm->height_per < 0.75) ? (1.0 - arm->height_per) : 0.25;
    return power_mult;
}

void Chassis::modify_profiled_velocity(int velocity) {
    peripherals->left_mtr.modify_profiled_velocity(velocity);
    peripherals->lefttwo_mtr.modify_profiled_velocity(velocity);
    peripherals->right_mtr.modify_profiled_velocity(velocity);
    peripherals->righttwo_mtr.modify_profiled_velocity(velocity);
};

void Chassis::set(int power, int turn, int strafe) {

    double power_mult = power_mult_calc();
    power_mult = (slowmode) ? 0.5 : power_mult;

    float powere = power_mult * sgn(power) * 12000 * pow(power / 127.0, 2); // exponential speed function
    float turne = power_mult * sgn(turn) * 12000 * pow(turn / 127.0, 2);

    int left = (int)powere + (int)turne;
    int right = (int)powere - (int)turne;

    if (abs(left) > 0 || abs(right) > 0 || abs(strafe) > 0) {
        peripherals->left_mtr.move_voltage(left);
        peripherals->right_mtr.move_voltage(right);
        peripherals->lefttwo_mtr.move_voltage(left);
        peripherals->righttwo_mtr.move_voltage(right);
        peripherals->strafe_mtr.move(strafe);
    } else {
        peripherals->left_mtr.move_velocity(0);
        peripherals->right_mtr.move_velocity(0);
        peripherals->lefttwo_mtr.move_velocity(0);
        peripherals->righttwo_mtr.move_velocity(0);
        peripherals->strafe_mtr.move_velocity(0);
    }
}
pros::vision_object_s_t last_obj;
int last_return;
// Currently set to only move strafe, as forward/backward align is unreliable / not necessary
int Chassis::vision_align() {

    //vision_signature_s_t sig = pros::Vision::signature_from_utility ( 1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0 );
    //vision::signature SIG_1 (1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0);

    pros::vision_object_s_t rtn = camera.get_by_size(0);

    if (compare_vision_objects(last_obj, rtn)) {
        return last_return;
    }
    last_obj = rtn;

    if (camera.get_object_count() == 0)
        return -9999;

    if (rtn.width > 20) {
        vision_distance.store(190 - rtn.width);
        double strafeOut = strafePID.step(x_coord_filter.filter(rtn.x_middle_coord));
        last_return = motor_speed * strafeOut;
        return motor_speed * strafeOut;
    }
    return 0;
}
