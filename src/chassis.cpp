#include "main.h"
#include "math.h"
#include "stdexcept"

extern Arm arm;

Chassis::Chassis() {
    pros::motor_gearset_e_t motor_gearset = peripherals.left_mtr.get_gearing();
    if (motor_gearset == MOTOR_GEARSET_06)
        motor_speed = 600;
    else if (motor_gearset == MOTOR_GEARSET_18)
        motor_speed = 200;
    else if (motor_gearset == MOTOR_GEARSET_36)
        motor_speed = 100;
    //else throw std::invalid_argument("Cannot get gearset of left mtr");
}

void Chassis::user_control() {
    int power = peripherals.master_controller.get_analog(ANALOG_RIGHT_Y);
    int turn = peripherals.master_controller.get_analog(ANALOG_RIGHT_X);

    int slowmode_button = peripherals.master_controller.get_digital_new_press(DIGITAL_R1);
    if (slowmode_button == 1)
        slowmode = !slowmode;
    //pros::lcd::print(5,"height per %f",arm.height_per);
    double power_mult = power_mult_calc();
    power_mult = (slowmode) ? 0.5 : power_mult;

    power = power * power_mult;
    turn = turn * power_mult;
    this->set(power, turn);
}

double Chassis::power_mult_calc() {
    double power_mult = (arm.height_per < 0.5) ? (1.0 - arm.height_per) : 0.5;
    return power_mult;
}

void Chassis::set(int power, int turn) {

    //float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
    //float turne = 1/(sgn(turn) * 127) * pow((float)turn,2);

    float powere = (sgn(power) / motor_speed) * pow(((float)power * motor_speed / 127), 2); // exponential speed function
    float turne = (sgn(turn) / motor_speed) * pow((float)(turn * motor_speed / 127), 2);

    int left = (int)powere + (int)turne;
    int right = (int)powere - (int)turne;
    //pros::lcd::print(0, "Left: %d\nRight: %d\n", left,right);

    peripherals.left_mtr.move_velocity(left);
    peripherals.right_mtr.move_velocity(right);
    peripherals.lefttwo_mtr.move_velocity(left);
    peripherals.righttwo_mtr.move_velocity(right);

    std::string left_v = std::to_string(peripherals.left_mtr.get_actual_velocity());
    std::string right_v = std::to_string(peripherals.right_mtr.get_actual_velocity());
    //pros::lcd::set_text(3,left_v);
    //pros::lcd::set_text(4,right_v);
}
