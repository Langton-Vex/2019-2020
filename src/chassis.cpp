#include "main.h"
#include "math.h"
#include "stdexcept"

#define PI 3.14159
#define IN_TO_CM 2.54 // MULTIPLY BY THIS TO CONVERT INCHES TO CM

#define TURN_RADIUS 0.1472184 // ALL UNITS IN METRES
#define WHEEL_CIRCUMFERENCE 0.32916037028 // CLAW BOT NUMBERS

Chassis::Chassis(){

	turn_radius = TURN_RADIUS; // made a macro for convenience,
	wheel_circumference = WHEEL_CIRCUMFERENCE;// But OO is good practice

	pros::motor_gearset_e_t motor_gearset = peripherals.left_mtr.get_gearing();
	if (motor_gearset == MOTOR_GEARSET_06) motor_speed = 600;
	else if (motor_gearset == MOTOR_GEARSET_18) motor_speed = 200;
	else if (motor_gearset == MOTOR_GEARSET_36) motor_speed = 100;
	else throw std::invalid_argument("Cannot get gearset of left mtr");
}

void Chassis:: user_control(){
	int power = peripherals.master_controller.get_analog(ANALOG_RIGHT_Y);
	int turn = peripherals.master_controller.get_analog(ANALOG_RIGHT_X);

			this->set(power,turn);
}


void Chassis::set(int power, int turn){

	//float powere = 1/(sgn(power) * 127) * pow((float)power,2); // exponential voltage function
	//float turne = 1/(sgn(turn) * 127) * pow((float)turn,2);

	float powere = (sgn(power) / motor_speed) * pow(((float)power*motor_speed / 127),2); // exponential speed function
	float turne = (sgn(turn) / motor_speed) * pow((float) (turn*motor_speed / 127),2);
	int left = (int) powere + (int) turne;
	int right = (int) powere - (int) turne;
	pros::lcd::print(0, "Left: %d\nRight: %d\n", left,right);
	peripherals.left_mtr.move_velocity(left);
	peripherals.right_mtr.move_velocity (right);
}

/*distance: a float, in metres
velocity: an integer, in RPM*/
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

/*angle: an int, in degrees
velocity: an integer, in RPM*/
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
