#include "main.h"
#include "math.h"
#include "stdexcept"


Chassis::Chassis(){
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
  peripherals.lefttwo_mtr.move_velocity(left);
  peripherals.righttwo_mtr.move_velocity ( right);
}
