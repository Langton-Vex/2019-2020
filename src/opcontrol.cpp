

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

void set_temperature(void* param){
	std::uint32_t now = pros::millis();
  while (true) {
		int arm_temp = round((peripherals.leftarm_mtr.get_temperature() +
		                      peripherals.rightarm_mtr.get_temperature())/2);

		int drive_temp = round((peripherals.left_mtr.get_temperature() +
										      peripherals.right_mtr.get_temperature()+
													peripherals.lefttwo_mtr.get_temperature()+
													peripherals.righttwo_mtr.get_temperature()
											)/4);

		std::string arm_temp_str = std::to_string(arm_temp);
		std::string drive_temp_str = std::to_string(drive_temp);
		arm_temp_str.append(" arm");

		drive_temp_str.append(" drive");
		std::string line = arm_temp_str;
		line.append(" ");
		line.append(drive_temp_str);
		peripherals.master_controller.print(0,0,line.c_str());

    pros::Task::delay(1000);
		peripherals.master_controller.clear();
		pros::Task::delay(50);

	}
}

void opcontrol() {
	pros::Task temp_task (set_temperature);

	while (true) {
		//macros_update(peripherals.master_controller);

    arm.user_control();
    chassis.user_control();
		claw.user_control();

		pros::delay(20);
	}
}
