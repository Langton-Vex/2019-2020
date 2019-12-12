#include "main.h"
#include "math.h"
#include "stdexcept"

extern int8_t left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, intake_port, strafe_port;

std::shared_ptr<Chassis> Chassis::get() {
    static std::shared_ptr<Chassis> instance(new Chassis);
    return instance;
}

Chassis::Chassis() {
    pros::motor_gearset_e_t motor_gearset = peripherals->left_mtr.get_gearing();
    if (motor_gearset == MOTOR_GEARSET_06)
        motor_speed = 600;
    else if (motor_gearset == MOTOR_GEARSET_18)
        motor_speed = 200;
    else if (motor_gearset == MOTOR_GEARSET_36)
        motor_speed = 100;
    //else throw std::invalid_argument("Cannot get gearset of left mtr");
}

void Chassis::user_control() {
    int power = peripherals->master_controller.get_analog(ANALOG_RIGHT_Y);
    int turn = peripherals->master_controller.get_analog(ANALOG_RIGHT_X);

    int strafe;

    int current_intake = peripherals->master_controller.get_digital(DIGITAL_RIGHT);
    int current_eject = peripherals->master_controller.get_digital(DIGITAL_Y);
    if (current_intake)
        strafe = -127;
    else if (current_eject)
        strafe = 127;
    else
        strafe = 0;

    int slowmode_button = peripherals->master_controller.get_digital_new_press(DIGITAL_B);
    if (slowmode_button == 1)
        slowmode = !slowmode;
    //pros::lcd::print(5,"height per %f",arm.height_per);
    double power_mult = power_mult_calc();
    power_mult = (slowmode) ? 0.5 : power_mult;

    power = power * power_mult;
    turn = turn * power_mult;

    int align_button = peripherals->master_controller.get_digital(DIGITAL_DOWN);
    if (align_button) {
        this->set(power, 0, 0);
        if (!aligning) {
            aligning = true;
            align_task = std::make_unique<pros::Task>(vision_align, (void*)NULL, TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT, "Vision Align");
        }
    } else {
        aligning = false;
        this->set(power, turn, strafe);
        if (align_task) {
            align_task->remove();
            align_task.reset();
        }
    }

    //slowmode_button = peripherals->master_controller.get_digital_new_press(DIGITAL_Y);
    //power_mult = (slowmode) ? 0.5 : power_mult;
    //strafe = strafe * power_mult;
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

    //float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
    //float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
    //float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
    //float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
    //float turne = 1/(sgn(turn) * 127) * pow((float)turn,2);

    float powere = (sgn(power) / motor_speed) * pow(((float)power * motor_speed / 127), 2); // exponential speed function
    float turne = (sgn(turn) / motor_speed) * pow((float)(turn * motor_speed / 127), 2);

    int left = (int)powere + (int)turne;
    int right = (int)powere - (int)turne;
    //pros::lcd::print(0, "Left: %d\nRight: %d\n", left,right);

    peripherals->left_mtr.move_velocity(left);
    peripherals->right_mtr.move_velocity(right);
    peripherals->lefttwo_mtr.move_velocity(left);
    peripherals->righttwo_mtr.move_velocity(right);

    peripherals->strafe_mtr.move(strafe);

    std::string left_v = std::to_string(peripherals->left_mtr.get_actual_velocity());
    std::string right_v = std::to_string(peripherals->right_mtr.get_actual_velocity());
    //pros::lcd::set_text(3,left_v);
    //pros::lcd::set_text(4,right_v);
}

// Currently set to only move strafe, as forward/backward align is unreliable / not necessary
void Chassis::vision_align(void* param) {
    okapi::AverageFilter<5> x_coord_filter;
    /*
    okapi::EmaFilter x_coord_filter(1);
    okapi::DemaFilter x_coord_filter(1,1);
    okapi::MedianFilter<5> x_coord_filter;
    */
    pros::Vision camera(15, pros::E_VISION_ZERO_CENTER);
    //vision_signature_s_t sig = pros::Vision::signature_from_utility ( 1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0 );
    //vision::signature SIG_1 (1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0);

    auto straightPID = okapi::IterativeControllerFactory::posPID(0.0020, 0.000000, 0.0);
    auto turnPID = okapi::IterativeControllerFactory::posPID(0.00200, 0.000000, 0.00089);
    auto strafePID = okapi::IterativeControllerFactory::posPID(0.0016, 0.000000, 0.0002);
    double leftVelocity, rightVelocity, strafeVelocity;

    while (true) {
        pros::vision_object_s_t rtn = camera.get_by_size(0);
        if (camera.get_object_count() == 0)
            continue;
        if (rtn.width > 30) {
            double straightOut = -straightPID.step(215 - rtn.width);
            double strafeOut = strafePID.step(x_coord_filter.filter(rtn.x_middle_coord));
            //double turnOut = turnPID.step(rtn.width - rtn.height);
            /*
            if (strafePID.isSettled()){
              leftVelocity = 200.0 * straightOut;
              rightVelocity = 200.0 * straightOut;
            }
            */
            strafeVelocity = 200.0 * strafeOut;

            peripherals->left_mtr.move_velocity((int)leftVelocity);
            peripherals->right_mtr.move_velocity((int)rightVelocity);
            peripherals->lefttwo_mtr.move_velocity((int)leftVelocity);
            peripherals->righttwo_mtr.move_velocity((int)rightVelocity);
            peripherals->strafe_mtr.move_velocity((int)strafeVelocity);
        }
        pros::delay(10);
    }
}
