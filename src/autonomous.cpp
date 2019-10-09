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
    leftarm_port, rightarm_port, leftintake_port, rightintake_port;

extern ConfigManager configManager;

const auto WHEEL_DIAMETER = 4.3_in;
const auto CHASSIS_WIDTH = 370_mm;

// intake is roughly 5

auto ccont = ChassisControllerFactory::create(
    { left_port, lefttwo_port }, // peripherals.left_port,peripherals.lefttwo_port
    { right_port, righttwo_port }, // peripherals.right_port,peripherals.righttwo_port
    AbstractMotor::gearset::green,
    { WHEEL_DIAMETER, CHASSIS_WIDTH });

MotorGroup intake({ leftintake_port, rightintake_port });

extern AsyncPosIntegratedController lift;

void lift_stack(int cubes) {
    lift.setTarget(-30);
    lift.waitUntilSettled();
    intake.moveVelocity(-200);
    lift.setMaxVelocity(27);
    pros::delay(630);
    lift.setTarget(-(18.4681 * 5.5 * (cubes - 1))); //TODO: Make this not hard-coded
    lift.waitUntilSettled(); // Perfect stacking speeds from 4 inches up
    intake.moveVelocity(0);
    lift.setMaxVelocity(100);
}

// Starts pointing towards small goal zone
void near_small() {
    ccont.moveDistance(15.6 * inch);

    lift_stack(1);

    lift.setTarget(0);
    lift.waitUntilSettled();
    ccont.moveDistance(-16_in);
}

// Starts pointing towards singular cube one tile left of large goal zone
void colour_tile() {

    ccont.setMaxVelocity(150);
    int side = configManager.selected_team;
    lift.setMaxVelocity(100);

    ccont.moveDistanceAsync(11.5 * inch);
    lift.setTarget(-30);
    lift.waitUntilSettled();
    ccont.waitUntilSettled();

    //ccont.turnAngle(-110_deg);
    //ccont.moveDistance(5_in);

    intake.moveVelocity(200);
    lift.setTarget(0);
    lift.waitUntilSettled();
    //pros::delay(500); // TODO: This defo needs changing
    //intake.moveVelocity(0);
    //lift.setTarget(-200);
    //lift.waitUntilSettled();

    ccont.moveDistance(-5.6 * inch);
    intake.moveVelocity(0);
    ccont.turnAngle(90_deg * side);
    ccont.moveDistance(27_in);

    lift_stack(2);

    lift.setMaxVelocity(100);
    lift.setTarget(0);
    lift.waitUntilSettled();

    ccont.moveDistance(-16 * inch);
}

/* This starts with the left side of the robot in line with the middle of
cube to the right of the medium sized tower */
void four_stack() {
    int side = configManager.selected_team;
    lift.setMaxVelocity(100);

    ccont.moveDistance(20 * inch);
    ccont.moveDistance(-3.5 * inch);
    ccont.turnAngle(-90_deg * side);

    lift.setTarget(24.7 * 18.4681);
    lift.waitUntilSettled();
    ccont.moveDistance(5.94 * inch);
    intake.moveVelocity(-200);
    lift.setTarget(lift.getTarget() + (18.4681 * 1.5));
    pros::delay(630);
    lift.waitUntilSettled();
    intake.moveVelocity(0);
    ccont.moveDistance(-5.94 * inch);
    lift.setTarget(0);
    lift.waitUntilSettled();

    ccont.turnAngle(90_deg * side);
    ccont.moveDistance(13_in);
    ccont.turnAngle(90_deg * side);
    ccont.moveDistance(12.5 * inch);
    lift.setTarget(18 * 18.4681);
    ccont.turnAngle(-90_deg * side);
    ccont.moveDistance(6_in);
    lift.waitUntilSettled();
    intake.moveVelocity(200);
    lift.setTarget(0);
    lift.waitUntilSettled();
    ccont.turnAngle(90_deg * side);
    ccont.moveDistance(32_in);
    ccont.turnAngle(80_deg * side);
    ccont.moveDistance(20_in);
}
/* This autonomous starts with the right side of the robot lined up with the
   middle of the left mid tower cube */
void four_floor_small() {
    ccont.moveDistance(20_in);
    ccont.moveDistance(-3.5 * inch);

    lift.setTarget(24.7 * 18.4681);
    ccont.turnAngle(90_deg);
    lift.waitUntilSettled();
    ccont.moveDistance(7_in);
    intake.moveVelocity(-200);

    ccont.moveDistance(7_in);
    intake.moveVelocity(-200);
    lift.setTarget(lift.getTarget() + (18.4681 * 1.5));
    pros::delay(630);
    lift.waitUntilSettled();
    ccont.moveDistance(-7_in);
    lift.setTarget(0);
    lift.waitUntilSettled();
    ccont.moveDistance(-13_in);
    ccont.turnAngle(90_deg);
    ccont.moveDistance(4_in);
    ccont.turnAngle(90_deg);
    ccont.moveDistance(15_in);
    ccont.turnAngle(90_deg);

    intake.moveVelocity(200);
    for (int i = 0; i < 4; i++) {
        lift.setTarget(-30);
        ccont.moveDistance(5.5 * inch);
        lift.setTarget(0);
        pros::delay(630);
    }

    ccont.moveDistance(-35_in);
    ccont.turnAngle(-100_deg);
    ccont.moveDistance(14_in);
}

void do_nothing() {
    pros::delay(5000);
};

void move_15() {
    lift.setTarget(-800);
    ccont.moveDistance(15_in);
    ccont.moveDistance(-15_in);
}

void auton_safety(void* param) {
    while (true) {
        double power_mult = chassis.power_mult_calc();
        //printf("power mult: %f\n",power_mult);
        int max_speed = 150 * round(power_mult);
        if (max_speed > 150)
            max_speed = 150;
        chassis.modify_profiled_velocity(max_speed);
        ccont.setMaxVelocity(max_speed);

        pros::delay(20);
    }
}

void init_autonomous() {
    lift.flipDisable(true);

    configManager.register_auton("near small", near_small);
    configManager.register_auton("colour tile", colour_tile);
    configManager.register_auton("do nothing", do_nothing);
    configManager.register_auton("four stack grab", four_stack);
    configManager.register_auton("four floor small", four_floor_small);

    configManager.register_auton("Move 15", move_15);
}

void autonomous() {
    if (configManager.auton_routines.size() > configManager.selected_auton) {

        //pros::Task auton_task(auton_safety, nullptr, "autonsafety_task");

        auton_routine routine = configManager.auton_routines[configManager.selected_auton];
        lift.flipDisable(false);
        routine(); // nullptr could happen :o
        lift.flipDisable(true);
        //auton_task.remove();
    } else {
        printf("Selected auton is greater than amount of autons");
    }
}
