#include "main.h"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// left, right, arm, claw
Peripherals_t peripherals(1,2,3,4);
Chassis chassis;
Arm arm;
Claw claw;

void initialize() {

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
 // btw statics are cool
  /*
	FILE* config_file = fopen("/usd/config.txt", "r"); // file on SD card
  char buf[50]; // This just needs to be larger than the contents of the file
  fread(buf, 1, 50, config_file); // passing 1 because a `char` is 1 byte, and 50 b/c it's the length of buf
  printf("%s\n", buf); // print the string read from the file
  // Should print "Example text" to the terminal
  fclose(config_file); // always close files when you're done with them
  */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
