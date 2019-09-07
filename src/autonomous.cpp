#include "main.h"

extern Chassis chassis;

// The autonomous framework that is a godsend
using namespace okapi;

/*
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

const auto WHEEL_DIAMETER = 4_in;  // Fix this
const auto CHASSIS_WIDTH = 13.5_in;



void autonomous() {

  auto okapi_chassis = ChassisControllerFactory::create(
                  {peripherals.left_port,peripherals.lefttwo_port},
                  {peripherals.right_port,peripherals.righttwo_port},
                  AbstractMotor::gearset::red,
                  {WHEEL_DIAMETER, CHASSIS_WIDTH});

  okapi_chassis.moveDistance(1_m);
  okapi_chassis.turnAngle(90_deg);
}
