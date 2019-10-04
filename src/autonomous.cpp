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

extern ConfigManager configManager;

const auto WHEEL_DIAMETER = 4.07_in;
const auto CHASSIS_WIDTH = 384_mm;

// intake is roughly 5

auto ccont = ChassisControllerFactory::create(
                {left_port,lefttwo_port}, // peripherals.left_port,peripherals.lefttwo_port
                {right_port,righttwo_port},// peripherals.right_port,peripherals.righttwo_port
                AbstractMotor::gearset::green,
                {WHEEL_DIAMETER, CHASSIS_WIDTH});

MotorGroup intake({leftintake_port,rightintake_port});

auto lift = AsyncControllerFactory::posIntegrated(
  {leftarm_port,-rightarm_port});

void near_small(){
    printf("near small running");
    pros::delay(20);

    ccont.turnAngle(-100_deg);
    ccont.moveDistance(18_in);
    intake.moveVelocity(127);
    lift.setTarget(-400);
    lift.waitUntilSettled();
    intake.moveVelocity(0);
    ccont.moveDistance(-16_in);
    lift.setTarget(0);

}

void colour_tile(){
    ccont.setMaxVelocity(150);
    int side = configManager.selected_team;

    lift.setMaxVelocity(100);

    pros::lcd::set_text(1,"running autonomous");
    //ccont.moveDistance(1_in);
    //ccont.turnAngle(90_deg);
    lift.setTarget(-400);
    lift.waitUntilSettled();
    ccont.moveDistanceAsync(8_in);

    ccont.waitUntilSettled();

    //ccont.turnAngle(-110_deg);
    //ccont.moveDistance(5_in);

    intake.moveVelocity(-127);
    lift.setTarget(0);
    lift.waitUntilSettled();
    pros::delay(500); // This defo needs changing
    //intake.moveVelocity(0);
    lift.setTarget(-200);
    lift.waitUntilSettled();

    ccont.moveDistance(-5_in);
    ccont.turnAngle(110_deg * side);
    ccont.moveDistance(34_in);
    intake.moveVelocity(0);
    lift.setTarget(0);
    lift.waitUntilSettled();
    intake.moveVelocity(127);
    pros::delay(250);
    lift.setMaxVelocity(50);
    lift.setTarget(-500);
    lift.waitUntilSettled();
    pros::delay(250);
    lift.setMaxVelocity(200);
    lift.setTarget(-1000);
    lift.waitUntilSettled();
    lift.setTarget(-400);
    lift.waitUntilSettled();
    pros::delay(500);
    intake.moveVelocity(0);
    ccont.setMaxVelocity(100);
    ccont.moveDistance(-16_in);
    lift.setTarget(0);


}

void do_nothing(){};

void init_autonomous(){

  lift.flipDisable(true);

  configManager.register_auton("near small", near_small);
  configManager.register_auton("colour tile", colour_tile);
  configManager.register_auton("do nothing",do_nothing);
}

void autonomous() {
    auton_routine routine = configManager.auton_routines[configManager.selected_auton];
    lift.flipDisable(false);
    routine(); // nullptr could happen :o
    lift.flipDisable(true);

}
