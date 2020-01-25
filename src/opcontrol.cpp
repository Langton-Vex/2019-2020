

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

void set_temperature(int i) {

    double chassis_temp = (peripherals->left_mtr.get_temperature() + peripherals->right_mtr.get_temperature() + peripherals->lefttwo_mtr.get_temperature() + peripherals->righttwo_mtr.get_temperature()) / 4;
    double claw_temp = (peripherals->intake_mtr.get_temperature());

    if (i == 2)
        peripherals->master_controller.print(1, 0, "%d,%d,%d,%d, %d", (int)peripherals->leftarm_mtr.get_temperature(), (int)peripherals->rightarm_mtr.get_temperature(), (int)chassis_temp, (int)claw_temp, (int)peripherals->strafe_mtr.get_temperature());
    if (i == 1)
        peripherals->master_controller.print(0, 0, "AL,AR,Ch,Cl,st");

    /*
    lv_gauge_set_value(gui->chassis_temp_guage, 0, chassis_temp);
    lv_gauge_set_value(gui->arm_temp_guage, 0, arm_temp);
    lv_gauge_set_value(gui->claw_temp_guage, 0, claw_temp);
    */
}

void opcontrol() {
    std::shared_ptr<Chassis> chassis = Chassis::get();
    std::shared_ptr<Arm> arm = Arm::get();
    std::shared_ptr<Claw> claw = Claw::get();

    peripherals->master_controller.print(-1, 0, ""); //NOTE: This may or may not work

    int it = 11;
    uint32_t time = pros::millis();
    while (true) {
        //macros_update(peripherals->master_controller);

        arm->user_control();
        chassis->user_control();
        claw->user_control();

        it--;
        if (it == 0)
            it = 11;
        if (it <= 10 && it % 5 == 0) // Trying to delay over 50ms but not using tasks ...
            set_temperature(it / 5);

        pros::Task::delay_until(&time, 10);
    }
}
