#include "main.h"
#include <sstream>

const double max_height = 3.0;

Arm::Arm(){
  peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  current_major_position = 0;
  current_minor_position = 0;
  user_pos_modifier = 0;
  sensitivity = 1;
}

void Arm:: user_control(){
      int power = peripherals.master_controller.get_analog(ANALOG_LEFT_Y);
      //int pos_joy = peripherals.master_controller.get_analog(ANALOG_LEFT_Y);
      //float pos_rotations = (float)pos_joy / (127 * 4); // figure out the bounds of the arm,
                                                         //  and set appropriate values

      int arm_up = peripherals.master_controller.get_digital_new_press(DIGITAL_L1);
      int arm_down = peripherals.master_controller.get_digital_new_press(DIGITAL_L2);

      int block_up = peripherals.master_controller.get_digital_new_press(DIGITAL_R1);
      int block_down = peripherals.master_controller.get_digital_new_press(DIGITAL_R2);

      current_major_position = arm_up&(current_major_position <= 3) ?
          current_major_position+1 : current_major_position;
      current_major_position = arm_down&(current_major_position >= 0) ?
          current_major_position-1 : current_major_position;

      current_minor_position = block_up&(current_minor_position <= 3) ?
          current_minor_position+1 : current_minor_position;
      current_minor_position = block_down&(current_minor_position >= 0) ?
          current_minor_position-1 : current_minor_position;

      user_pos_modifier += (double)power * sensitivity;


      double final_height = major_positions[current_major_position] +
                    minor_positions[current_minor_position] +
                    user_pos_modifier;

      if (final_height < max_height)
        this->set_pos(final_height);

      // Quick and dirty place to put this
      std::string arm_pos = std::to_string(peripherals.rightarm_mtr.get_position());
      pros::lcd::set_text(1,arm_pos);

      //this->set(power);
}

void Arm::set(int power){
  //if (abs(power) < 10) power = 5;
  peripherals.leftarm_mtr.move(power);
  peripherals.rightarm_mtr.move(power);

}
void Arm::set_pos(double position){
    if (position < 0.1) {
      peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    else{
      peripherals.leftarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      peripherals.rightarm_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      }

    peripherals.leftarm_mtr.move_absolute(position,63);
    peripherals.rightarm_mtr.move_absolute(position,63);
}
