#include "main.h"

extern Chassis chassis;

// The autonomous framework that is a godsend
using namespace okapi;

/*
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

const auto WHEEL_DIAMETER = 4.125_in;  // Fix this
const auto CHASSIS_WIDTH = 385_mm;



void autonomous() {

  auto ccont = ChassisControllerFactory::create(
                  {peripherals.left_port,peripherals.lefttwo_port},
                  {peripherals.right_port,peripherals.righttwo_port},
                  AbstractMotor::gearset::red,
                  {WHEEL_DIAMETER, CHASSIS_WIDTH});

  ccont.moveDistance(1_m);
  ccont.waitUntilSettled();
  ccont.turnAngle(180_deg);
  ccont.waitUntilSettled();
  ccont.moveDistance(1_m);
}
