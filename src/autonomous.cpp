#include "main.h"

// The autonomous framework that is a godsend
using namespace okapi;

/*
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

const auto WHEEL_DIAMETER = 4.3_in;
const auto CHASSIS_WIDTH = 370_mm;
extern okapi::QLength INTAKE_FROM_CENTER;

std::shared_ptr<Motor> intake;
std::shared_ptr<ChassisControllerHDrive> cc;
pros::Mutex cc_mutex;
// This mutex is not actually used for the controller, rather the shared_ptr
// We  don't want two threads trying to set/reset this pointer!

//extern std::shared_ptr<okapi::AsyncPositionController<double, double>> lift;

void open_claw() {
    intake->moveVoltage(-12000);
    while (intake->getActualVelocity() > 5)
        pros::delay(10);
    intake->moveVoltage(0);
}

void close_claw() {
    intake->moveVoltage(12000);
    while (intake->getActualVelocity() > 5)
        pros::delay(10);
}

void create_cc() {
    PIDTuning straightTuning = PIDTuning(0.001890, 0.0, 0.000019);
    PIDTuning angleTuning = PIDTuning(0.000764, 0, 0.000007);
    PIDTuning turnTuning = PIDTuning(0.001500, 0, 0.000053);
    PIDTuning strafeTuning = PIDTuning(0.002, 0, 0.00003);
    okapi::MotorGroup leftSide(
        { left_port, lefttwo_port });
    okapi::MotorGroup rightSide(
        { right_port, righttwo_port });
    okapi::Motor strafeMotor(strafe_port);

    // std::unique_ptr<ChassisControllerHDrive>
    cc_mutex.take(TIMEOUT_MAX);
    // Auton cleanup shouldn't get preempted, but just in case
    cc = std::make_unique<ChassisControllerHDrive>(
        straightTuning, angleTuning, turnTuning, strafeTuning, // Tunings
        leftSide, rightSide, strafeMotor, // left mtr, right mtr, strafe mtr
        okapi::AbstractMotor::gearset::green, // swerve steer gearset
        okapi::AbstractMotor::gearset::green, // strafe gearset
        okapi::ChassisScales(
            { { okapi::inch * 4.125, 15.1 * okapi::inch, // wheel diam, wheelbase diam
                  0 * okapi::millimeter, okapi::inch * 4.125 }, // middle wheel distance, middle wheel diam
                okapi::imev5GreenTPR }));
    cc_mutex.give();
}

void init_autonomous() {

    intake = std::make_unique<okapi::Motor>(intake_port);

    Arm::get()->flipDisable(true);

    if (!cc)
        create_cc();

    auto configManager = ConfigManager::get();
    configManager->register_auton("do nothing", do_nothing);
    configManager->register_auton("four stack", four_stack,
        okapi::OdomState{ 97.1_in, 26.4_in - INTAKE_FROM_CENTER, 0_deg });

    configManager->register_auton("Move 15", move_15);
    configManager->register_auton("arm test", arm_test);
    //configManager->register_auton("potentiomenter test", pot_lookup);
    //configManager->register_auton("vision test", vision_test);
}

void auton_cleanup() {
    Arm::get()->flipDisable(true);
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
    pros::delay(150); // Counter ADI garbage
    auto our_cleanup_task = pros::Task(auton_cleanup_task, NULL, TASK_PRIORITY_DEFAULT,
        TASK_STACK_DEPTH_DEFAULT, "Auton cleanup");

    pros::c::task_notify_when_deleting(CURRENT_TASK, our_cleanup_task, 0,
        pros::E_NOTIFY_ACTION_NONE);

    if (!cc)
        create_cc();

    // Run your standard auton
    std::shared_ptr<ConfigManager> configManager = ConfigManager::get();
    std::shared_ptr<Arm> arm = Arm::get();
    arm->set_height(0_in);
    if (configManager->auton_routines.size() > configManager->selected_auton) {
        auton_func routine = configManager->get_auton_func(configManager->selected_auton);
        cc->start_task();
        arm->flipDisable(false);
        routine(); // nullptr could happen, lets hope it doesn't :o
        arm->flipDisable(true);
        cc->stop_task();
        //auton_task.remove();
    } else {
        printf("Selected auton is greater than amount of autons");
    }
    auton_cleanup();
}
