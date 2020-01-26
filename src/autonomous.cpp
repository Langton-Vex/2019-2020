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
const auto INTAKE_FROM_CENTER = 12.6_in; // NOTE: this is probably wrong, measure this when the arm is at rest!

std::shared_ptr<okapi::ChassisController> ccont;
std::shared_ptr<Motor> intake;
std::shared_ptr<ChassisControllerHDrive> cc;
pros::Mutex cc_mutex;
// This mutex is not actually used for the controller, rather the shared_ptr
// We  don't want two threads trying to set/reset this pointer!

//extern std::shared_ptr<okapi::AsyncPositionController<double, double>> lift;

extern pros::ADIAnalogIn arm_pot;

void open_claw() {
    intake->moveVoltage(-12000);
    while (intake->getActualVelocity() > 5)
        pros::delay(10);
    intake->moveVoltage(-1000); // -1volt
}

void close_claw() {
    intake->moveVoltage(12000);
    while (intake->getActualVelocity() > 5)
        pros::delay(10);
}

void position_intake_to_point(okapi::QLength x, okapi::QLength y) {
    double angle_rad = cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian);
    cc->driveToPoint({ x - (sin(angle_rad) * INTAKE_FROM_CENTER), y - (cos(angle_rad) * INTAKE_FROM_CENTER) });
}

void position_intake_to_point_async(okapi::QLength x, okapi::QLength y) {
    double angle_rad = cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian);
    cc->driveToPointAsync({ x - (sin(angle_rad) * INTAKE_FROM_CENTER), y - (cos(angle_rad) * INTAKE_FROM_CENTER) });
}

void position_intake_to_point_diag(okapi::QLength x, okapi::QLength y) {
    double angle_rad = cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian);
    cc->driveVector(x - (sin(angle_rad) * INTAKE_FROM_CENTER), y - (cos(angle_rad) * INTAKE_FROM_CENTER));
}

void vision_test() {
    cc->tune();
    //std::shared_ptr<Arm> arm = Arm::get();
    //arm->tune();
    pros::delay(100);
    //fprintf(stderr, "waiting for yeet");

    /*
    cc->driveToPoint({ 2_ft, 0_ft });
    cc->driveToPoint({ 0_ft, 0_ft });
    cc->lookToPoint({ 1_ft, 0_ft });
    */
    //cc->generatePath({ { 0_ft, 0_ft, 0_deg }, { 2_ft, 0_ft, 0_deg } }, "A");
    //cc->runPath("A", false, false);
    //return;
    /*
    cc->stop_task();

    auto profileController = AsyncMotionProfileControllerBuilder()
                             .withLimits({0.5, 0.5, 0.5})
                             .withOutput(ccont)
                             .buildMotionProfileController();
    profileController->generatePath({{0_ft, 0_ft, 0_deg}, {2_ft, -2_ft, -90_deg}}, "A");
    profileController->setTarget("A");
    profileController->waitUntilSettled();
    */

    //while (true)
    //    pros::delay(100);
    //return;
}

// Starts pointing towards small goal zone
void near_small() {
    ccont->moveDistance(12 * inch);
    ccont->moveDistance(-18_in);
}

void do_nothing() {
    while (true)
        pros::delay(10);
};

void simpler_four_stack() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();

    // Get to cube
    arm->flipDisable(true);
    auto cubeydelta = (48.2_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y;

    cc->driveStraight(cubeydelta);
    arm->flipDisable(false);
    arm->set_height(1.4_in);

    cc->strafe((97.1_in - cc->odom->getState(okapi::StateMode::CARTESIAN).x));

    arm->waitUntilSettled();

    cc->setHeading(0_deg);
    cc->driveStraight(1.4_in); // Actually goes to 49.6, for a tad bit of tolerance
    intake->moveVoltage(12000);
    pros::delay(500);
    //close_claw();
    arm->set_height(4_in);
    cc->driveStraight(-1.5_ft);

    auto large_side_x = (side > 0) ? 11.5_ft : 56.2_in;
    // ok so this is jank but the zones are mirrored weirdly in this game and I don't want to write two routines so here we go
    position_intake_to_point(large_side_x, 9_in);

    //cc->lookToPoint({ large_side + (1_ft * side), cc->odom->getState(okapi::StateMode::CARTESIAN).y });

    arm->flipDisable(false);
    arm->set_height(0_in);
    arm->waitUntilSettled();
    arm->flipDisable(true);
    arm->set(-200);
    pros::delay(500);
    intake->moveVoltage(-12000);
    pros::delay(500);
    // open_claw();
    arm->set(200);
    intake->moveVoltage(0);
    cc->driveStraight(-2_in);
    arm->set(0);
}
/*
void new_four_stack() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();

    // Get to cube
    arm->flipDisable(true);

    auto nearcubeydelta = 26.4_in - cc->odom->getState(okapi::StateMode::CARTESIAN).y;
    cc->driveStraight(nearcubeydelta);
    arm->flipDisable(false);
    arm->set_height(2.3_in);
    cc->strafe((97.1_in - cc->odom->getState(okapi::StateMode::CARTESIAN).x) + (3_in * side * -1));
    cc->setHeading(0_deg);
    auto cubeydelta = (48.7_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y;
    cc->driveStraight(cubeydelta);

    arm->waitUntilSettled();
    intake->moveVoltage(12000);
    pros::delay(500);
    arm->set_height(3.5_in);

    cc->driveStraight(-2_ft);

    auto large_side = (side > 0) ? 11.5_ft - INTAKE_FROM_CENTER : 58.6_in + INTAKE_FROM_CENTER;
    cc->driveToPoint({ large_side, 5_in });

    arm->flipDisable(false);
    arm->set_height(0_in);
    arm->waitUntilSettled();
    arm->flipDisable(true);
    arm->set(-200);
    pros::delay(500);
    intake->moveVoltage(-12000);
    pros::delay(500);
    arm->set(200);
    intake->moveVoltage(0);
    pros::delay(100);
    arm->set(0);
    cc->driveStraight(-2_in);
    arm->set(0);

    // If arm control works?

    arm->set_height(0_in);
    arm->waitUntilSettled();
    intake->moveVoltage(-12000);
    pros::delay(500);
    arm->set_height(1_in);
    intake->moveVoltage(0);
    cc->driveStraight(-2_in);

}

void four_floor() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();
    okapi::QAngle inward;
    if (side < 0)
        inward = 180_deg;
    else
        inward = 0_deg;

    intake->moveVoltage(12000);
    arm->set_height(5.5_in);
    cc->driveToPoint({ 26.4_in, 33.4_in - INTAKE_FROM_CENTER });
    cc->setHeading(inward);
    for (int i = 0; i < 3; i++) {
        intake->moveVoltage(12000);
        pros::delay(500);
        arm->set_height(5.5_in);
        arm->waitUntilSettled();
        cc->driveStraight(5.5_ft);
        intake->moveVoltage(-12000);
        pros::delay(500);
        arm->set_height(0_in);
    }
    arm->set_height(2.3_in);
    intake->moveVoltage(12000);
    pros::delay(500);
    arm->set_height(3.5_in);

    cc->driveToPoint({ 6_in + (INTAKE_FROM_CENTER * sin(cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian))),
        6_in + (INTAKE_FROM_CENTER * cos(cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian))) });
    arm->set_height(0_in);
    arm->waitUntilSettled();

    intake->moveVoltage(-12000);
    pros::delay(500);
    arm->set_height(5_in);
    intake->moveVoltage(0);
    cc->driveStraight(-2_in);
}

void bob_auton() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();

    auto small_side = 0.5_ft + INTAKE_FROM_CENTER;

    //cube 1
    close_claw();
    arm->set_height(6.5_in);

    //cube 2
    cc->driveStraight((33.4_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y);
    arm->set_height(5.5_in);
    open_claw();
    arm->set_height(0_in);
    arm->waitUntilSettled();
    close_claw();
    arm->set_height(6.5_in);
    arm->waitUntilSettled();

    //cube 3
    cc->driveStraight((38.9_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y);
    arm->set_height(5.5_in);
    open_claw();
    arm->set_height(0_in);
    arm->waitUntilSettled();
    close_claw();
    arm->set_height(6.5_in);
    arm->waitUntilSettled();

    //cube 4
    cc->driveStraight((44.4_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y);
    arm->set_height(5.5_in);
    open_claw();
    arm->set_height(2.3_in);
    arm->waitUntilSettled();
    close_claw();

    arm->set_height(3.5_in);
    arm->waitUntilSettled();

    cc->driveStraight(-1.5_ft);
    cc->driveToPoint({ small_side, 4.5_in });
    //cc->lookToPoint({ small_side + (1_ft * side), cc->odom->getState(okapi::StateMode::CARTESIAN).y });

    arm->set_height(0_in);
    arm->waitUntilSettled();
    open_claw();
    arm->set_height(1_in);
    cc->driveStraight(-4_in);
    arm->waitUntilSettled();
}
*/

void skill_auton() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();

    close_claw();
    arm->set_height(26_in);
    position_intake_to_point_async(77_in, 25.4_in);
    arm->waitUntilSettled();
    position_intake_to_point(70.3_in, 23.2_in);
    open_claw();

    position_intake_to_point_diag(97.1_in, 49.9_in);
    arm->set_height(0.5_in);
    cc->setHeading(0_deg);
    arm->waitUntilSettled();
    close_claw();
    arm->set_height(2_in);

    position_intake_to_point(11.5_ft, 8_in);
    open_claw();
    arm->set_height(3_in);

    position_intake_to_point_diag(112.3_in, 70.3_in);
    arm->set_height(0_in);
    cc->lookToPoint({ 105.7_in, 70.3_in });
    arm->waitUntilSettled();
    close_claw();

    arm->set_height(18.7_in);
    arm->waitUntilSettled();
    position_intake_to_point(105.7_in, 70.3_in);
    open_claw();

    cc->strafe(1.25_ft);
    arm->set_height(0.5_in);
    position_intake_to_point_diag(97.1_in, 90.7_in);
    arm->waitUntilSettled();
    close_claw();
    arm->set_height(3_in);

    position_intake_to_point(11.5_ft, 10.5_ft);
    arm->set_height(0_in);
    arm->waitUntilSettled();
    open_claw();
    arm->set_height(3_in);

    position_intake_to_point_diag(77_in, 117.5_in);
    arm->set_height(0_in);
    arm->waitUntilSettled();
    cc->lookToPoint({ 70.3_in, 117.5_in });
    close_claw();
    arm->set_height(26_in);
    arm->waitUntilSettled();
    position_intake_to_point(70.3_in, 117.5_in);
    open_claw();
}

/* ----------------------------------------------------------------
   AUTON HOUSE KEEPING FUNCTIONS
   ---------------------------------------------------------------- */

void create_cc() {
    PIDTuning straightTuning = PIDTuning(0.001890, 0.0, 0.000019);
    PIDTuning angleTuning = PIDTuning(0.000764, 0, 0.000007);
    PIDTuning turnTuning = PIDTuning(0.001500, 0, 0.000053);
    PIDTuning strafeTuning = PIDTuning(0.003819, 0, 0.000030);

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
}

void init_autonomous() {
    ccont = ChassisControllerBuilder()
                .withMotors({ left_port, lefttwo_port },
                    { right_port, righttwo_port })
                .withDimensions(AbstractMotor::gearset::green, { { WHEEL_DIAMETER, CHASSIS_WIDTH }, imev5GreenTPR })
                .build();

    intake = std::make_unique<okapi::Motor>(intake_port);

    Arm::get()->flipDisable(true);

    if (!cc)
        create_cc();

    auto configManager = ConfigManager::get();
    configManager->register_auton("near small", near_small);
    configManager->register_auton("do nothing", do_nothing);
    configManager->register_auton("four stack", simpler_four_stack,
        okapi::OdomState{ 97.1_in, 26.4_in - INTAKE_FROM_CENTER, 0_deg });
    /*
    configManager->register_auton("strafey new four stack", new_four_stack,
        okapi::OdomState{ 97.1_in, 26.4_in - INTAKE_FROM_CENTER, 0_deg });

    configManager->register_auton("bob auton", bob_auton,
        okapi::OdomState{ 26.4_in, 33.4_in - INTAKE_FROM_CENTER, 0_deg });
    configManager->register_auton("four floor", four_floor,
        okapi::OdomState{ 26.4_in, 33.4_in - INTAKE_FROM_CENTER, 0_deg });
        */

    //configManager->register_auton("potentiomenter test", pot_lookup);
    configManager->register_auton("vision test", vision_test);
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
    } else {
        printf("Selected auton is greater than amount of autons");
    }
    auton_cleanup();
}
