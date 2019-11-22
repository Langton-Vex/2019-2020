#include "main.h"

// The autonomous framework that is a godsend
using namespace okapi;

/*
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

extern int8_t left_port, right_port, lefttwo_port, righttwo_port,
    leftarm_port, rightarm_port, intake_port, strafe_port;

const auto WHEEL_DIAMETER = 4.3_in;
const auto CHASSIS_WIDTH = 370_mm;

std::shared_ptr<okapi::ChassisController> ccont;
std::shared_ptr<Motor> intake;
std::shared_ptr<ChassisControllerHDrive> cc;
pros::Mutex cc_mutex;
// This mutex is not actually used for the controller, rather the shared_ptr
// We  don't want two threads trying to set/reset this pointer!

extern std::shared_ptr<okapi::AsyncPosIntegratedController> lift;

void lift_stack(int cubes) {
    lift->setMaxVelocity(22);
    lift->setTarget(-108);
    intake->moveVelocity(-200);
    pros::delay(1500);
    lift->setTarget(lift->getTarget() + (18.45 * 5.5 * (cubes - 1))); //TODO: Make this not hard-coded
    lift->waitUntilSettled(); // Perfect stacking speeds from 4 inches up
    intake->moveVelocity(0);
    lift->setMaxVelocity(100);
}

// Starts pointing towards small goal zone
void near_small() {
    ccont->moveDistance(17 * inch);
    ccont->moveDistance(-18_in);
}

void do_nothing() {
    pros::delay(5000);
};

void move_15() {
    lift->setTarget(800);
    ccont->moveDistance(15_in);
    ccont->moveDistance(-15_in);
}

void small_four_cubes() {
    auto i = Claw::get();
    int side = ConfigManager::get()->selected_team;

    // Pick up preload
    i->set(-127);
    //intake->moveVelocity(-200);
    pros::delay(500); // make this some kinda distance based.

    lift->setTarget(18.45 * 6.7); // TODO: make pot based!!
    lift->waitUntilSettled();
    cc->driveStraight(36 * okapi::centimeter);
    i->set(127);
    cc->driveStraight(-3 * okapi::centimeter);

    lift->setTarget(0);
    pros::delay(100);
    i->set(127);
    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(500);
    // Drive to scoring zone, place cubes, move out of the way a bit
    lift->setTarget(18.45 * 6.7);
    lift->waitUntilSettled();
    cc->driveStraight(6.7 * okapi::inch);

    lift->setTarget(0);
    pros::delay(100);
    i->set(127);
    lift->waitUntilSettled();
    lift->setTarget(0);
    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(500);
    // Drive to scoring zone, place cubes, move out of the way a bit
    lift->setTarget(18.45 * 5.5);
    cc->turnAngle(-139 * okapi::degree);
    cc->driveStraight(54 * okapi::centimeter);
    lift->setTarget(0);
    lift->waitUntilSettled();
    i->set(127);
    lift->setTarget(18.45 * 5.5);
    lift->waitUntilSettled();
}

void two_cubes() {
    auto i = Claw::get();
    int side = ConfigManager::get()->selected_team;

    // Pick up preload
    i->set(-127);
    //intake->moveVelocity(-200);
    pros::delay(500); // make this some kinda distance based.

    // drive to first cube and pickup
    lift->setTarget(18.45 * 6.7); // TODO: make pot based!!
    lift->waitUntilSettled();
    cc->driveStraight(24 * okapi::centimeter);
    i->set(127);
    cc->driveStraight(-5 * okapi::centimeter);
    lift->setTarget(0);

    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(600);
    lift->setTarget(18.45 * 5.5);
    lift->waitUntilSettled();

    cc->turnAngle(102 * okapi::degree * side);
    cc->driveStraight(84 * okapi::centimeter);
    lift->setTarget(0);
    lift->waitUntilSettled();
    i->set(127);
    lift->setTarget(18.45 * 5.5);
    lift->waitUntilSettled();
}

void three_cubes() {
    auto i = Claw::get();
    int side = ConfigManager::get()->selected_team;

    // Pick up preload
    i->set(-127);
    //intake->moveVelocity(-200);
    pros::delay(500); // make this some kinda distance based.

    // drive to first cube and pickup
    lift->setTarget(18.45 * 6.6); // TODO: make pot based!!
    lift->waitUntilSettled();
    cc->driveStraight(21 * okapi::centimeter);
    i->set(127);
    cc->driveStraight(-5 * okapi::centimeter);
    lift->setTarget(0);

    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(600);

    // move arm up, drive to second cube

    cc->driveStraight(14 * okapi::inch);
    lift->setTarget(18.45 * 8.2);
    lift->waitUntilSettled();
    cc->turnAngle(90 * okapi::degree * side);
    cc->driveStraight(32 * okapi::centimeter);

    // Pickup second cube
    lift->setTarget(0);
    pros::delay(100);
    i->set(127);
    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(500);

    // Drive to scoring zone, place cubes, move out of the way a bit
    lift->setTarget(18.45 * 5.5);
    cc->turnAngle(40 * okapi::degree * side);
    cc->driveStraight(53 * okapi::centimeter);
    lift->setTarget(0);
    lift->waitUntilSettled();
    i->set(127);
    lift->setTarget(18.45 * 5.5);
    lift->waitUntilSettled();
}

void init_autonomous() {

    ccont = ChassisControllerBuilder()
                .withMotors({ left_port, lefttwo_port },
                    { right_port, righttwo_port })
                .withGearset(AbstractMotor::gearset::green)
                .withDimensions({ { WHEEL_DIAMETER, CHASSIS_WIDTH }, imev5GreenTPR })
                .build();

    intake = std::make_unique<okapi::Motor>(intake_port);

    lift->flipDisable(true);
    auto configManager = ConfigManager::get();
    configManager->register_auton("near small", near_small);
    configManager->register_auton("do nothing", do_nothing);

    configManager->register_auton("three cubes", three_cubes);
    configManager->register_auton("two cubes", two_cubes);
    configManager->register_auton("small_four_cubes", small_four_cubes);

    configManager->register_auton("Move 15", move_15);
}

void auton_cleanup() {
    lift->flipDisable(true);
    cc_mutex.take(TIMEOUT_MAX);
    if (cc) {
        cc->stop_task();
        cc->reset();
    }
    cc_mutex.give();
}
void auton_cleanup_task(void* param) {
    pros::c::task_notify_take(true, TIMEOUT_MAX);
    auton_cleanup();
}

void autonomous() {
    auto our_cleanup_task = pros::Task(auton_cleanup_task, NULL, TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT, "Auton cleanup");

    pros::c::task_notify_when_deleting(CURRENT_TASK, our_cleanup_task, 0,
        pros::E_NOTIFY_ACTION_NONE);

    PIDTuning straightTuning = PIDTuning(0.001890, 0.0, 0.000019);
    PIDTuning angleTuning = PIDTuning(0.000764, 0, 0.000007);
    PIDTuning turnTuning = PIDTuning(0.002304, 0, 0.000033);
    PIDTuning strafeTuning = PIDTuning(0, 0, 0);
    PIDTuning hypotTuning = PIDTuning(0, 0, 0);
    okapi::MotorGroup leftSide(
        { left_port, lefttwo_port });
    okapi::MotorGroup rightSide(
        { right_port, righttwo_port });
    okapi::Motor strafeMotor(strafe_port);

    // std::unique_ptr<ChassisControllerHDrive>
    cc = std::make_unique<ChassisControllerHDrive>(
        straightTuning, angleTuning, turnTuning, strafeTuning, hypotTuning, // Tunings
        leftSide, rightSide, strafeMotor, // left mtr, right mtr, strafe mtr
        okapi::AbstractMotor::gearset::green, // swerve steer gearset
        okapi::AbstractMotor::gearset::green, // strafe gearset
        okapi::ChassisScales(
            { { okapi::inch * 4.125, 15.1 * okapi::inch, // wheel diam, wheelbase diam
                  0 * okapi::millimeter, okapi::inch * 4.125 }, // middle wheel distance, middle wheel diam
                okapi::imev5GreenTPR }));

    //cc.tune();

    // Run your standard auton
    std::shared_ptr<ConfigManager> configManager = ConfigManager::get();

    if (configManager->auton_routines.size() > configManager->selected_auton) {
        auton_routine routine = configManager->auton_routines[configManager->selected_auton];
        cc->start_task();
        lift->flipDisable(false);
        routine(); // nullptr could happen, lets hope it doesn't :o
        lift->flipDisable(true);
        cc->stop_task();
        //auton_task.remove();
    } else {
        printf("Selected auton is greater than amount of autons");
    }
    auton_cleanup();
}

/*
blue small side - be lined up with the 4 long tihng of cubes
  grip the pre-load
  forward(distance to cubes...)
  armUp(height of one cube)
  forward(length of cube)
  armUn-Grab
  repeat 4 times:
    armDown(height of one cubes)
    armGrab
    armUp(height of one cube)
    fowrard(length of one cube)
    armUn-Grab
  backwards(one tile)w
  turnLeft(135Deg)
  forward(34 inches)
  armUn-Grab
  */
