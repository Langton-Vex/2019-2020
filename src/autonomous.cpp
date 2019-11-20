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

void three_cubes(){
    auto i = Claw::get();
    int side = ConfigManager::get()->selected_team;

    // Pick up preload
    lift->setTarget(0);
    lift->waitUntilSettled();
    i->set(-127);
    //intake->moveVelocity(-200);
    pros::delay(1000); // make this some kinda distance based.

    // drive to first cube and pickup
    lift->setTarget(18.45 * 5.5); // TODO: make pot based!!
    cc->driveStraight(31.5 * okapi::centimeter);
    lift->setTarget(0);
    pros::delay(200);
    i->set(127);
    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(500);

    // move arm up, drive to second cube
    lift->setTarget(18.45 * 5.5);
    cc->turnAngle(90*okapi::degree);
    cc->driveStraight(22 * okapi::centimeter);

    // Pickup second cube
    lift->setTarget(0);
    pros::delay(200);
    i->set(127);
    lift->waitUntilSettled();
    i->set(-127);
    pros::delay(500);

    // Drive to scoring zone, place cubes, move out of the way a bit
    lift->setTarget(18.45 * 5.5);
    cc->turnAngle(25*okapi::degree);
    cc->driveStraight(53*okapi::centimeter);
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

    configManager->register_auton("Move 15", move_15);
}

void auton_cleanup(void* param){
  pros::c::task_notify_take(true, TIMEOUT_MAX);

  lift->flipDisable(true);
  if (cc){
    cc->stop_task();
    cc->reset();
  }
}

void autonomous() {
    auto our_cleanup_task = pros::Task(auton_cleanup, NULL, TASK_PRIORITY_DEFAULT,
                              TASK_STACK_DEPTH_DEFAULT, "Auton cleanup");

    pros::c::task_notify_when_deleting(CURRENT_TASK, our_cleanup_task,0,
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
          {{ okapi::inch * 4.125, 15.1 * okapi::inch,  // wheel diam, wheelbase diam
          0 * okapi::millimeter, okapi::inch * 4.125 }, // middle wheel distance, middle wheel diam
            okapi::imev5GreenTPR })
        );

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
}
