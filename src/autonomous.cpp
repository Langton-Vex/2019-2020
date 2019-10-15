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

std::shared_ptr<okapi::ChassisControllerIntegrated> ccont = std::static_pointer_cast<okapi::ChassisControllerIntegrated>(
    ChassisControllerBuilder()
        .withMotors({ static_cast<int8_t>(left_port), static_cast<int8_t>(lefttwo_port) },
            { static_cast<int8_t>(right_port), static_cast<int8_t>(righttwo_port) })
        .withGearset(AbstractMotor::gearset::green)
        .withDimensions({ { WHEEL_DIAMETER, CHASSIS_WIDTH }, imev5GreenTPR })
        .build());

MotorGroup intake({ static_cast<int8_t>(leftintake_port), static_cast<int8_t>(rightintake_port) });

extern std::shared_ptr<okapi::AsyncPosIntegratedController> lift;

void lift_stack(int cubes) {
    lift->setMaxVelocity(22);
    lift->setTarget(-108);
    intake.moveVelocity(-200);
    pros::delay(1500);
    lift->setTarget(lift->getTarget() + -(18.45 * 5.5 * (cubes - 1))); //TODO: Make this not hard-coded
    lift->waitUntilSettled(); // Perfect stacking speeds from 4 inches up
    intake.moveVelocity(0);
    lift->setMaxVelocity(100);
}

// Starts pointing towards small goal zone
void near_small() {
    ccont->moveDistance(17 * inch);
    ccont->moveDistance(-18_in);
}

// Starts pointing towards singular cube one tile left of large goal zone
void colour_tile() {

    ccont->setMaxVelocity(150);
    int side = configManager.selected_team;
    lift->setMaxVelocity(200);

    lift->setTarget(-148);
    ccont->setMaxVelocity(100);
    ccont->moveDistanceAsync(11.2 * inch);
    lift->waitUntilSettled();
    ccont->waitUntilSettled();
    lift->setMaxVelocity(100);
    //ccont->turnAngle(-110_deg);
    //ccont->moveDistance(5_in);

    intake.moveVelocity(200);
    lift->setTarget(0);
    lift->waitUntilSettled();
    lift->setTarget(-100);
    pros::delay(1000);
    //pros::delay(500); // TODO: This defo needs changing
    //intake.moveVelocity(0);
    //lift->setTarget(-200);
    //lift->waitUntilSettled();

    ccont->moveDistance(-7.5 * inch);
    intake.moveVelocity(0);
    ccont->turnAngle(100_deg * side);
    ccont->moveDistance(38_in);
    lift->setTarget(0);
    lift->waitUntilSettled();
    lift_stack(2);

    lift->setMaxVelocity(100);
    ccont->setMaxVelocity(100);
    ccont->moveDistance(-17 * inch);
    lift->setTarget(0);
    lift->waitUntilSettled();
}

void colour_tile_3_point() {

    ccont->setMaxVelocity(150);
    int side = configManager.selected_team;
    lift->setMaxVelocity(200);

    lift->setTarget(-108);
    ccont->setMaxVelocity(100);
    ccont->moveDistanceAsync(11.5 * inch);
    ccont->waitUntilSettled();
    lift->setMaxVelocity(100);
    //ccont->turnAngle(-110_deg);
    //ccont->moveDistance(5_in);

    intake.moveVelocity(200);
    lift->setTarget(0);
    lift->waitUntilSettled();
    lift->setTarget(-100);
    pros::delay(1000);
    //pros::delay(500); // TODO: This defo needs changing
    //intake.moveVelocity(0);
    //lift->setTarget(-200);
    //lift->waitUntilSettled();

    ccont->moveDistance(3.5 * inch);
    ccont->turnAngle(90_deg * side);
    lift->setMaxVelocity(200);
    lift->setTarget(-148);
    ccont->moveDistance(19 * inch);
    lift->setMaxVelocity(100);

    intake.moveVelocity(200);
    lift->setTarget(0);
    lift->waitUntilSettled();
    pros::delay(750);
    //lift->setTarget(-100);

    ccont->moveDistance(3 * inch);
    ccont->turnAngle(55_deg * side);
    ccont->moveDistance(20_in);

    lift->setTarget(0);
    lift->waitUntilSettled();
    lift_stack(3);

    lift->setMaxVelocity(100);
    ccont->setMaxVelocity(75);
    ccont->moveDistance(-17 * inch);
    lift->setTarget(0);
    lift->waitUntilSettled();
}

/* This starts with the left side of the robot in line with the middle of
cube to the right of the medium sized tower */
void four_stack() {
    int side = configManager.selected_team;
    lift->setMaxVelocity(100);

    ccont->moveDistance(20_in);
    ccont->moveDistance(-5.5 * inch);
    ccont->turnAngle(-90_deg * side);

    lift->setTarget(-(24.7 * 18.1123));
    lift->waitUntilSettled();
    ccont->setMaxVelocity(75);
    ccont->moveDistance(7_in);
    intake.moveVelocity(-200);
    lift->setTarget(lift->getTarget() - (18.1123 * 4));
    pros::delay(1200);
    lift->waitUntilSettled();
    intake.moveVelocity(0);
    ccont->setMaxVelocity(25);
    ccont->moveDistance(-6_in);
    lift->setTarget(0);
    lift->waitUntilSettled();
    ccont->setMaxVelocity(150);

    ccont->turnAngle(90_deg * side);
    ccont->moveDistance(13_in);
    ccont->turnAngle(90_deg * side);
    ccont->moveDistance(12.5 * inch);
    lift->setTarget(-(18 * 18.1123));
    ccont->turnAngle(-90_deg * side);
    ccont->moveDistance(6_in);
    lift->waitUntilSettled();
    intake.moveVelocity(200);
    lift->setTarget(0);
    lift->waitUntilSettled();
    ccont->turnAngle(90_deg * side);
    ccont->moveDistance(32_in);
    ccont->turnAngle(80_deg * side);
    ccont->moveDistance(20_in);
}

void four_stack_only() {
    int side = configManager.selected_team;
    lift->setMaxVelocity(100);

    ccont->moveDistance(22 * inch);
    lift->setTarget(-(18 * 18.1123));
    lift->waitUntilSettled();
    ccont->setMaxVelocity(75);
    ccont->moveDistance(5_in);
    intake.moveVelocity(200);
    lift->setTarget(0);
    lift->waitUntilSettled();
    ccont->turnAngle(90_deg * side);
    ccont->moveDistance(32_in);
    ccont->turnAngle(80_deg * side);
    ccont->moveDistance(32_in);
}
/* This autonomous starts with the right side of the robot lined up with the
   middle of the left mid tower cube */
void four_floor_small() {
    ccont->moveDistance(20_in);
    ccont->moveDistance(-5.5 * inch);

    lift->setTarget(-(24.7 * 18.1123));
    ccont->setMaxVelocity(75);
    ccont->turnAngle(90_deg);
    lift->waitUntilSettled();
    ccont->moveDistance(7_in);
    intake.moveVelocity(-200);

    ccont->moveDistance(3_in);
    intake.moveVelocity(-200);
    lift->setTarget(lift->getTarget() - (18.1123 * 1.5));
    pros::delay(1200);
    lift->waitUntilSettled();
    ccont->moveDistance(-7_in);
    lift->setTarget(0);
    lift->waitUntilSettled();
    ccont->setMaxVelocity(150);
    ccont->moveDistance(-13_in);
    ccont->turnAngle(90_deg);
    ccont->moveDistance(4_in);
    ccont->turnAngle(90_deg);
    ccont->moveDistance(15_in);
    ccont->turnAngle(90_deg);

    intake.moveVelocity(200);
    for (int i = 0; i < 4; i++) {
        lift->setTarget(-108);
        lift->waitUntilSettled();
        ccont->moveDistance(5.5 * inch);
        lift->setTarget(0);
        pros::delay(1200);
    }

    ccont->moveDistance(-35_in);
    ccont->turnAngle(-100_deg);
    ccont->moveDistance(14_in);
}

//  starts in line with the 4 floor cubes
void simple_four_floor() {
    lift->setTarget(-(5.5 * 18.1123));
    ccont->moveDistance(17.15_in);
    intake.moveVelocity(200);
    lift->setTarget(0);
    lift->waitUntilSettled();
    lift->setTarget(-(5.5 * 18.1123));
    ccont->moveDistance(5.5_in);
    lift->setTarget(0);
    lift->waitUntilSettled();
    ccont->moveDistance(-3_in);
    ccont->turnAngle(-135_deg);
    lift->setTarget(-(28.5 * 18.1123));
    lift_stack(3);
}
void do_nothing() {
    pros::delay(5000);
};

void move_15() {
    lift->setTarget(-800);
    ccont->moveDistance(15_in);
    ccont->moveDistance(-15_in);
}

void auton_safety(void* param) {
    while (true) {
        double power_mult = chassis.power_mult_calc();
        //printf("power mult: %f\n",power_mult);
        int max_speed = 150 * round(power_mult);
        if (max_speed > 150)
            max_speed = 150;
        chassis.modify_profiled_velocity(max_speed);
        ccont->setMaxVelocity(max_speed);

        pros::delay(20);
    }
}

void init_autonomous() {
    lift->flipDisable(true);

    configManager.register_auton("near small", near_small);
    configManager.register_auton("colour tile", colour_tile);
    configManager.register_auton("colour tile three pnt", colour_tile_3_point);
    configManager.register_auton("do nothing", do_nothing);
    configManager.register_auton("four stack grab", four_stack);
    configManager.register_auton("four floor small", four_floor_small);
    configManager.register_auton("four floor simple", simple_four_floor);
    configManager.register_auton("four stack only", four_stack_only);

    configManager.register_auton("Move 15", move_15);
}

void autonomous() {
    if (configManager.auton_routines.size() > configManager.selected_auton) {

        //pros::Task auton_task(auton_safety, nullptr, "autonsafety_task");

        auton_routine routine = configManager.auton_routines[configManager.selected_auton];
        lift->flipDisable(false);
        routine(); // nullptr could happen :o
        lift->flipDisable(true);
        //auton_task.remove();
    } else {
        printf("Selected auton is greater than amount of autons");
    }
}
