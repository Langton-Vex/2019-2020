

#include "main.h"
#include <iomanip>
#include <sstream>
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

extern int8_t left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, intake_port, strafe_port;

void set_temperature() {

    std::shared_ptr<GUI> gui = GUI::get();

    std::string temp = "Arm: ";
    std::string arm_pos_string = "Arm: ";
    std::string lift_imbalance_str = "Imbalance to the left: ";

    arm_pos_string.append(std::to_string(peripherals->leftarm_mtr.get_position()));

    double chassis_temp = (peripherals->left_mtr.get_temperature() + peripherals->right_mtr.get_temperature() + peripherals->lefttwo_mtr.get_temperature() + peripherals->righttwo_mtr.get_temperature()) / 4;
    double arm_temp = (peripherals->leftarm_mtr.get_temperature() + peripherals->rightarm_mtr.get_temperature()) / 2;
    double claw_temp = (peripherals->intake_mtr.get_temperature());

    double leftarm_pwr = peripherals->leftarm_mtr.get_power();
    double rightarm_pwr = peripherals->rightarm_mtr.get_power();
    double smallest_arm_pwr = std::min(leftarm_pwr, rightarm_pwr);
    double normalised_imbalance = (leftarm_pwr / smallest_arm_pwr) - (rightarm_pwr / smallest_arm_pwr);

    std::stringstream stream;
    stream << lift_imbalance_str << std::fixed << std::setprecision(6) << normalised_imbalance;
    lift_imbalance_str = stream.str();

    peripherals->master_controller.print(0, 0, "A:%d;C:%d", (int)arm_temp, (int)chassis_temp);

    gui->set_line(0, arm_pos_string);
    gui->set_line(1, lift_imbalance_str);

    lv_gauge_set_value(gui->chassis_temp_guage, 0, chassis_temp);
    lv_gauge_set_value(gui->arm_temp_guage, 0, arm_temp);
    lv_gauge_set_value(gui->claw_temp_guage, 0, claw_temp);
}

void opcontrol() {
    /*
    pros::Task temp_task(set_temperature, nullptr, "temp_task");
    pros::delay(20);
    PIDTuning straightTuning = PIDTuning(0.01, 0.0, 0);
    PIDTuning angleTuning = PIDTuning(0, 0, 0);
    PIDTuning turnTuning = PIDTuning(10, 0, 0);
    PIDTuning strafeTuning = PIDTuning(0, 0, 0);
    PIDTuning hypotTuning = PIDTuning(0, 0, 0);
    okapi::MotorGroup leftSide(
        {-left_port,-lefttwo_port });
    okapi::MotorGroup rightSide(
        {-right_port,-righttwo_port});
    okapi::Motor strafeMotor(11);

    std::unique_ptr<ChassisControllerHDrive> cc = std::make_unique<ChassisControllerHDrive>(
        ChassisControllerHDrive(
            straightTuning, angleTuning, turnTuning, strafeTuning, hypotTuning,
            leftSide, rightSide, strafeMotor,
            okapi::AbstractMotor::gearset::green,
            okapi::AbstractMotor::gearset::green,
            { { okapi::inch * 4.3, okapi::millimeter * 370 }, okapi::imev5GreenTPR }));
    */
    //cc->driveStraightAsync(10 * okapi::inch);
    /*
    cc->start_task();
    cc->waitUntilSettled();
    */

    //cc->start_task();
    //cc->driveStraight(5 * okapi::inch);
    //cc->turnAngle(90 * okapi::degree);

    //cc->waitUntilSettled();
    //cc->tune();

    //pros::Task temp_task(set_temperature, nullptr, "temp_task");
    std::shared_ptr<Chassis> chassis = Chassis::get();
    std::shared_ptr<Arm> arm = Arm::get();
    std::shared_ptr<Claw> claw = Claw::get();

    int it = 50;
    while (true) {
        //lv_ta_add_text(GUI::get()->console_box,std::to_string(pros::millis()).c_str());
        //cc->step();
        //macros_update(peripherals->master_controller);

        arm->user_control();
        chassis->user_control();
        claw->user_control();

        it--;
        if (it == 49)
            set_temperature();
        if (it == 0)
            it = 50;

        pros::delay(20);
    }
}
