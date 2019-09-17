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
                {-20,17},
                {-19,18},
                AbstractMotor::gearset::green,
                {WHEEL_DIAMETER, CHASSIS_WIDTH});

const double liftkP = 10.0;
const double liftkI = 1;
const double liftkD = 1;

auto lift = AsyncControllerFactory::posPID(
  {peripherals.leftarm_port,peripherals.rightarm_port},
  liftkP,liftkI,liftkD);

MotorGroup intake({peripherals.leftintake_port,peripherals.rightintake_port});

void autonomous() {
  pros::lcd::set_text(1,"running autonomous");
  //ccont.moveDistance(128_m);
  ccont.turnAngle(100);
  //ccont.moveDistanceAsync(6_in);
  //lift.setTarget(0.5);
  //ccont.waitUntilSettled();

  //ccont.turnAngle(-90_deg);
  //ccont.moveDistance(3_in);
  //lift.setTarget(0.1);
  //lift.waitUntilSettled();

  intake.moveVelocity(127);
  pros::delay(500); // This defo needs changing
  intake.moveVelocity(0);

  ccont.moveDistance(-3_in);
  //ccont.turnAngle(-90_deg);
  ccont.moveDistance(18_in);

  intake.moveVelocity(-127);
  pros::delay(500); // This defo needs changing
  intake.moveVelocity(0);



}
