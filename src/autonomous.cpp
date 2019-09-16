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

auto ccont = ChassisControllerFactory::create(
                {peripherals.left_port,peripherals.lefttwo_port},
                {peripherals.right_port,peripherals.righttwo_port},
                AbstractMotor::gearset::green,
                {WHEEL_DIAMETER, CHASSIS_WIDTH});

const double liftkP = 10.0;
const double liftkI = 1;
const double liftkD = 1;

auto liftControl = AsyncControllerFactory::posPID(
  {peripherals.leftarm_port,peripherals.rightarm_port},
  liftkP,liftkI,liftkD);

MotorGroup intake({peripherals.leftintake_port,peripherals.rightintake_port});

void autonomous() {

  ccont.turnAngle(90_deg);
  ccont.moveDistanceAsync(6_in);
  liftControl.setTarget(0.5);
  ccont.waitUntilSettled();

  ccont.turnAngle(-90_deg);
  ccont.moveDistance(3_in);
  liftControl.setTarget(0.1);
  liftControl.waitUntilSettled();

  intake.moveVelocity(127);
  pros::delay(500); // This defo needs changing
  intake.moveVelocity(0);

  ccont.moveDistance(-3_in);
  ccont.turnAngle(-90_deg);
  ccont.moveDistance(18_in);

  intake.moveVelocity(-127);
  pros::delay(500); // This defo needs changing
  intake.moveVelocity(0);



}
