#include "main.h"
#include <sstream>

const double max_height = -2;

using namespace okapi;

extern int left_port, right_port, lefttwo_port, righttwo_port,
             leftarm_port, rightarm_port, leftintake_port,rightintake_port;

double major_positions[4] = {0,250,500,750};
//double minor_positions[4] = {0.0,0.05,0.1,0.1};

const double liftkP = 100.0;
const double liftkI = 1;
const double liftkD = 1;

// auto liftControl = AsyncControllerFactory::posPID(
//  {leftarm_port,rightarm_port},
//  liftkP,liftkI,liftkD);

Arm::Arm(){
  peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  current_major_position = 0;
  current_minor_position = 0;
  user_pos_modifier = 0;
  sensitivity = 0.0001;
}

void Arm:: user_control(){
      int power = peripherals.master_controller.get_analog(ANALOG_LEFT_Y);

      bool arm_up = peripherals.master_controller.get_digital_new_press(DIGITAL_R1);
      bool arm_down = peripherals.master_controller.get_digital_new_press(DIGITAL_R2);
      //int block_up = peripherals.master_controller.get_digital_new_press(DIGITAL_R1);
      //int block_down = peripherals.master_controller.get_digital_new_press(DIGITAL_R2);


      if (arm_up && (current_major_position <= 3)){
        current_major_position++;
      }
      else if (arm_down&&(current_major_position > 0)){
        current_major_position--;
      }
      /*
      if (block_up && (current_minor_position <= 3)){
        current_minor_position++;
      }
      else if (block_down&&(current_minor_position > 0)){
        current_minor_position--;
      }
      */
      //user_pos_modifier += (double)power * sensitivity;
      double final_height = major_positions[current_major_position]/* +
                    minor_positions[current_minor_position]*/;

      //double final_height = user_pos_modifier;
      //if (final_height < max_height)

      height_per = abs(peripherals.leftarm_mtr.get_position()) / abs(max_height);

      // Quick and dirty place to put this
      std::string arm_pos = std::to_string(peripherals.leftarm_mtr.get_position());
      std::string temp = std::to_string(peripherals.leftarm_mtr.get_temperature());
      temp.append(" celcius");
      pros::lcd::set_text(1,arm_pos);

      pros::lcd::set_text(2,temp);
      //peripherals.master_controller.set_text(1,1,temp.c_str());

      //double power_mult = (peripherals.leftarm_mtr.get_actual_velocity() > 1 &&
      //height_per < 0.2) ? 0.01:1;

      //double power_mult = 1;
      power = power /* * power_mult*/;
      if (power > 5 || power < 5)
        this->set(power);
      /*
      else if (final_height != current_goal_height && abs(power) < 5){
        pros::lcd::set_text(3,"Setting target");
        liftControl.setTarget(final_height);
        current_goal_height = final_height;
      }
      */

      //else if(abs(peripherals.leftarm_mtr.get_position()) < -0.1)
      //  this->set(15); // dodgy holding but it works

      //this->set_pos(final_height);
}

void Arm::set(int power){
  //if (abs(power) < 10) power = 5;
  peripherals.leftarm_mtr.move(-power);
  peripherals.rightarm_mtr.move(-power);

}
void Arm::set_pos(double position){
  /*
    if (position < 0.1) {
      peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    else{
      peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }
    */
    peripherals.leftarm_mtr.move_absolute(-position,63);
    peripherals.rightarm_mtr.move_absolute(-position,63);
}
