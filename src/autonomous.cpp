#include "main.h"

extern Chassis chassis;

// The autonomous framework that is a godsend
using namespace okapi;

/*
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 extern int left_port, right_port, lefttwo_port, righttwo_port,
 	            leftarm_port, rightarm_port, leftintake_port,rightintake_port;

const auto WHEEL_DIAMETER = 4.125_in;  // Fix this
const auto CHASSIS_WIDTH = 384_mm;

// intake is roughly 5

auto ccont = ChassisControllerFactory::create(
                {left_port,lefttwo_port}, // peripherals.left_port,peripherals.lefttwo_port
                {right_port,righttwo_port},// peripherals.right_port,peripherals.righttwo_port
                AbstractMotor::gearset::green,
                {WHEEL_DIAMETER, CHASSIS_WIDTH});

MotorGroup intake({leftintake_port,rightintake_port});



void autonomous() {
  auto lift = AsyncControllerFactory::posIntegrated(
    {leftarm_port,-rightarm_port});

  pros::lcd::set_text(1,"running autonomous");
  ccont.turnAngle(90_deg);
  ccont.moveDistanceAsync(6_in);

  lift.setTarget(-500);
  ccont.waitUntilSettled();

  ccont.turnAngle(-90_deg);
  ccont.moveDistance(5_in);
  lift.setTarget(0);
  lift.waitUntilSettled();

  intake.moveVelocity(-127);
  pros::delay(1000); // This defo needs changing
  intake.moveVelocity(0);

  ccont.moveDistance(-5_in);
  ccont.turnAngle(-90_deg);
  ccont.moveDistance(18_in);

  lift.setTarget(-500);
  intake.moveVelocity(127);
  lift.waitUntilSettled();
  intake.moveVelocity(0);



}
