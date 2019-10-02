

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
extern Chassis chassis;
extern Arm arm;
extern Claw claw;

extern GUI gui;

void set_temperature(void* param){
	std::uint32_t now = pros::millis();
  while (true) {
		std::string temp = std::to_string(peripherals.leftarm_mtr.get_temperature());
		temp.append(" celcius");
		peripherals.master_controller.set_text(1,1,temp.c_str());

    std::string arm_pos_string = "Arm: ";
		arm_pos_string.append(std::to_string(peripherals.leftarm_mtr.get_position()));

		lv_ta_set_text(gui.console_box, arm_pos_string.c_str());

    pros::delay(123);


	}
}

void opcontrol() {

	pros::Task temp_task (set_temperature,nullptr,"temp_task");

	while (true) {
		//macros_update(peripherals.master_controller);

    arm.user_control();
    chassis.user_control();
		claw.user_control();

		pros::delay(20);
	}


}
