#include "main.h"
#include "math.h"
#include "stdexcept"

//#define PI 3.14159
#define IN_TO_CM 2.54 // MULTIPLY BY THIS TO CONVERT INCHES TO CM

Chassis::Chassis(double t_r, double w_c){
	// H-Drive

	turn_radius = t_r;
	wheel_circumference = w_c;

	pros::motor_gearset_e_t tank_motor_gearset = peripherals.left_mtr.get_gearing();
	if (tank_motor_gearset == MOTOR_GEARSET_06) tank_motor_speed = 600;
	else if (tank_motor_gearset == MOTOR_GEARSET_18) tank_motor_speed = 200;
	else if (tank_motor_gearset == MOTOR_GEARSET_36) tank_motor_speed = 100;
	else throw std::invalid_argument("Cannot get gearset of left mtr");

	pros::motor_gearset_e_t strafe_motor_gearset = peripherals.strafe_mtr.get_gearing();
	if (strafe_motor_gearset == MOTOR_GEARSET_06) tank_motor_speed = 600;
	else if (strafe_motor_gearset == MOTOR_GEARSET_18) tank_motor_speed = 200;
	else if (strafe_motor_gearset == MOTOR_GEARSET_36) tank_motor_speed = 100;
	else throw std::invalid_argument("Cannot get gearset of left mtr");
}

void Chassis:: user_control(){
	int forward = peripherals.master_controller.get_analog(ANALOG_RIGHT_Y);
	int right = peripherals.master_controller.get_analog(ANALOG_RIGHT_X);
  int turn = peripherals.master_controller.get_analog(ANALOG_LEFT_X);

	this->set(forward,right,turn);
}


void Chassis::set(int forward, int right, int turn){

	//float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
	//float turne = 1/(sgn(turn) * 127) * pow((float)turn,2);
	float powere = (sgn(forward) / tank_motor_speed) * pow(((float)forward*tank_motor_speed / 127),2); // exponential speed function
	float righte = (sgn(right) / strafe_motor_speed) * pow(((float)right*strafe_motor_speed / 127),2);
	float turne = (sgn(turn) / tank_motor_speed) * pow((float) (turn*tank_motor_speed / 127),2);

	int left_speed = (int) round(powere) + (int) round(turne);
	int right_speed = (int) round(powere) - (int) round(turne);
	int strafe_speed = (int) round(righte);

	peripherals.left_mtr.move_velocity(left_speed);
	peripherals.right_mtr.move_velocity (right_speed);
	peripherals.strafe_mtr.move_velocity (strafe_speed);
}

/*
distance: a float, in metres
velocity: an integer, in RPM
void Chassis::move_forward(double distance,int velocity=100){
	double rel_target = distance / wheel_circumference;
	double abs_target = peripherals.left_mtr.get_position() + rel_target;
  pros::lcd::print(0,"%f",rel_target);
  peripherals.left_mtr.move_relative(100,100);
  peripherals.right_mtr.move_relative(100,100);
	//peripherals.left_mtr.move_relative(rel_target,velocity);
	//peripherals.right_mtr.move_relative(rel_target,velocity);

	while (!((peripherals.left_mtr.get_position() < abs_target + 5.0) &&
				   (peripherals.left_mtr.get_position() > abs_target - 5.0))) {
	  // Continue running this loop as long as the motor is not within +-5 units of its goal
	  pros::delay(20);
  }

};

angle: an int, in degrees
velocity: an integer, in RPM
void Chassis::point_turn(double angle,int velocity=100){
	double rel_target = (2*turn_radius*PI * (angle/360)) / wheel_circumference;
	double abs_target = peripherals.left_mtr.get_position() + rel_target;
	peripherals.left_mtr.move_relative(rel_target,velocity);
	peripherals.right_mtr.move_relative(-rel_target,velocity);

	while (!((peripherals.left_mtr.get_position() < abs_target + 5.0) &&
				   (peripherals.left_mtr.get_position() > abs_target - 5.0))) {
	  // Continue running this loop as long as the motor is not within +-5 units of its goal
	  pros::delay(2);
  }

};
*/
