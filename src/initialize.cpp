#include "main.h"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// left, right, arm, claw

#define TURN_RADIUS 0.1472184 // ALL UNITS IN METRES
#define WHEEL_CIRCUMFERENCE 0.32916037028 // CLAW BOT NUMBERS

const int left_port = 20;
const int right_port = 17;
const int lefttwo_port = 19;
const int righttwo_port = 18;
const int leftintake_port = 3;
const int rightintake_port = 4;
const int leftarm_port = 1;
const int rightarm_port = 2;



Peripherals_t peripherals(left_port, right_port, lefttwo_port, righttwo_port,
	            leftintake_port, rightintake_port, leftarm_port,rightarm_port);

//Chassis chassis(TURN_RADIUS,WHEEL_CIRCUMFERENCE);
Chassis chassis = Chassis();
Arm arm = Arm();
Claw claw = Claw();

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
