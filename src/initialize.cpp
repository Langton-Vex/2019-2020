#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

/* NOTE: If you manage to find this...
 Things to do:
   * Tune arm PID
   * test motion profiling
   * double check things dont break when switching comp modes
   * Arm PID actually scales pot values to motor encoder values and runs internal PID?
   * Clean up this entire code base
   * lua autons
   * comms to a computer program
 */

extern const int8_t left_port = 20;
extern const int8_t right_port = 17;
extern const int8_t lefttwo_port = 19;
extern const int8_t righttwo_port = 18;
extern const int8_t intake_port = 6;
extern const int8_t strafe_port = 16;
extern const int8_t leftarm_port = 1;
extern const int8_t rightarm_port = 5;
extern const char leftenc_port = 'C';
extern const char rightenc_port = 'G';
extern const char midenc_port = 'E';

/* TODO: Yikes these definitions are getting messy, these need to be moved to
   one file at some point. */

extern std::shared_ptr<okapi::AsyncPositionController<double, double>> lift;
;
extern std::shared_ptr<ChassisControllerHDrive> cc;

// TODO: These need to be mutex managed pointers

std::unique_ptr<Peripherals_t> peripherals;

//Chassis chassis(TURN_RADIUS,WHEEL_CIRCUMFERENCE);

void init_autonomous(); // uh oh global space

extern std::shared_ptr<ChassisControllerHDrive> cc;

void initialize() {
    std::cerr << "Intializing" << std::endl;
    pros::delay(150);

    ConfigManager::get()->load_config();

    peripherals = std::make_unique<Peripherals_t>(left_port, right_port,
        lefttwo_port, righttwo_port, intake_port, strafe_port,
        leftarm_port, rightarm_port, leftenc_port, rightenc_port, midenc_port);

    Arm::get()->init();

    pros::delay(20);
    lift->flipDisable(true);
    init_autonomous();

    pros::delay(100);
    GUI::get()->gui_build();
    //cc->tune();
    pros::delay(100);

    //pros::lcd::initialize();
    //pros::lcd::set_text(1, "This means things are working?");
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
void competition_initialize() {
    pros::delay(100); // any use of sensors needs to be prefixed with a delay to avoid
        // sensor garbage (at least it seems to work)
    std::shared_ptr<ConfigManager> config_manager = ConfigManager::get();
    while (!cc && pros::competition::is_disabled())
        pros::delay(10);
    int side = ConfigManager::get()->selected_team;
    auto state = config_manager->get_auton_state(config_manager->selected_auton);
    peripherals->left_mtr.tare_position();
    peripherals->right_mtr.tare_position();
    peripherals->lefttwo_mtr.tare_position();
    peripherals->lefttwo_mtr.tare_position();
    cc->odom->step();
    cc->odom->setState(state,
        okapi::StateMode::CARTESIAN);
    int it = 0;
    while (pros::competition::is_disabled()) {
        GUI::get()->set_line(0, "Odom stepping");
        it++;
        if (cc)
            cc->odom->step();

        if (it >= 50){
          it = 0;
          auto state = cc->odom->getState(okapi::StateMode::CARTESIAN);
          printf("odom: x: %f, y:%f, theta:%f\n", state.x.convert(okapi::inch), state.y.convert(okapi::inch), state.theta.convert(okapi::degree));
        }
        pros::delay(10);
        /* keep track of odom state while initialising
         this means that the robot can be pushed during initialising and keep
         track of where it is, good for setting up auton relative to the field
      */
    }
}
