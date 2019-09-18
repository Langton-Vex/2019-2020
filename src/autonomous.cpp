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

// intake is roughly 5

auto ccont = ChassisControllerFactory::create(
                {20,19}, // peripherals.left_port,peripherals.lefttwo_port
                {17,18},// peripherals.right_port,peripherals.righttwo_port
                AbstractMotor::gearset::green,
                {WHEEL_DIAMETER, CHASSIS_WIDTH});

MotorGroup intake({3,4});



void autonomous() {
  auto lift = AsyncControllerFactory::posIntegrated(
    {1,-2});

  pros::lcd::set_text(1,"running autonomous");
  ccont.turnAngle(90_deg);
  ccont.moveDistanceAsync(6_in);
  lift.setTarget(500);

  lift.setTarget(-500);
  ccont.waitUntilSettled();

  ccont.turnAngle(-90_deg);
  ccont.moveDistance(3_in);
  lift.setTarget(0);
  lift.waitUntilSettled();

  intake.moveVelocity(-127);
  pros::delay(1000); // This defo needs changing
  intake.moveVelocity(0);

  ccont.moveDistance(-3_in);
  ccont.turnAngle(-90_deg);
  ccont.moveDistance(18_in);

  intake.moveVelocity(127);
  pros::delay(1000); // This defo needs changing
  intake.moveVelocity(0);



}
