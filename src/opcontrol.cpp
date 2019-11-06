

#include "main.h"

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

extern int left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, leftintake_port, rightintake_port;

void set_temperature(void* param) {

    std::shared_ptr<GUI> gui = GUI::get();
    std::uint32_t now = pros::millis();

    while (true) {
        std::string temp = std::to_string(peripherals->leftarm_mtr.get_temperature());
        temp.append(" celcius");
        peripherals->master_controller.set_text(1, 1, temp.c_str());

        std::string arm_pos_string = "Arm: ";
        arm_pos_string.append(std::to_string(peripherals->leftarm_mtr.get_position()));

        gui->set_line(0, arm_pos_string);

        double chassis_temp = (peripherals->left_mtr.get_temperature() + peripherals->right_mtr.get_temperature() + peripherals->lefttwo_mtr.get_temperature() + peripherals->righttwo_mtr.get_temperature()) / 4;

        double arm_temp = (peripherals->leftarm_mtr.get_temperature() + peripherals->rightarm_mtr.get_temperature()) / 2;

        double claw_temp = (peripherals->leftintake_mtr.get_temperature() + peripherals->rightintake_mtr.get_temperature()) / 2;

        lv_gauge_set_value(gui->chassis_temp_guage, 0, chassis_temp);
        lv_gauge_set_value(gui->arm_temp_guage, 0, arm_temp);
        lv_gauge_set_value(gui->claw_temp_guage, 0, claw_temp);

        pros::delay(123);
    }
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
        { static_cast<int8_t>(-left_port), static_cast<int8_t>(-lefttwo_port) });
    okapi::MotorGroup rightSide(
        { static_cast<int8_t>(-right_port), static_cast<int8_t>(-righttwo_port) });
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

    pros::Task temp_task(set_temperature, nullptr, "temp_task");
    std::shared_ptr<Chassis> chassis = Chassis::get();
    std::shared_ptr<Arm> arm = Arm::get();
    std::shared_ptr<Claw> claw = Claw::get();

    while (true) {
        //lv_ta_add_text(GUI::get()->console_box,std::to_string(pros::millis()).c_str());
        //cc->step();
        //macros_update(peripherals->master_controller);

        arm->user_control();
        chassis->user_control();
        claw->user_control();

        pros::delay(20);
    }

}
