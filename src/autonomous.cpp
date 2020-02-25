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
const auto INTAKE_FROM_CENTER = 13_in; // NOTE: this is probably wrong, measure this when the arm is at rest!

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
    while (intake->getActualVelocity() > 1)
        pros::delay(50);
    intake->moveVoltage(-1000); // -1volt
}

void close_claw() {
    intake->moveVoltage(12000);
    while (intake->getActualVelocity() > 1)
        pros::delay(50);
    pros::delay(200);
}

void position_intake_to_point_async(okapi::QLength x, okapi::QLength y) {
    auto angle = okapi::OdomMath::computeAngleToPoint(
        okapi::Point({ x, y }).inFT(okapi::StateMode::CARTESIAN), cc->odom->getState(okapi::StateMode::FRAME_TRANSFORMATION));
    cc->turnAngle(angle);

    double angle_rad = cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian);
    okapi::Point point{ x - (sin(angle_rad) * INTAKE_FROM_CENTER), y - (cos(angle_rad) * INTAKE_FROM_CENTER) };

    auto length = okapi::OdomMath::computeDistanceToPoint(
        point.inFT(okapi::StateMode::CARTESIAN), cc->odom->getState(okapi::StateMode::FRAME_TRANSFORMATION));

    cc->driveStraightAsync(length);
}

void position_intake_to_point(okapi::QLength x, okapi::QLength y) {
    position_intake_to_point_async(x, y);
    cc->waitUntilSettled();
}

void position_intake_to_point_diag(okapi::QLength x, okapi::QLength y) {
    double angle_rad = cc->odom->getState(okapi::StateMode::CARTESIAN).theta.convert(okapi::radian);
    cc->diagToPoint({ x - (sin(angle_rad) * INTAKE_FROM_CENTER), y - (cos(angle_rad) * INTAKE_FROM_CENTER) });
}

void position_intake_to_point_diag(okapi::QLength x, okapi::QLength y, okapi::QAngle angle) {
    double angle_rad = angle.convert(okapi::radian);
    cc->diagToPoint({ x - (sin(angle_rad) * INTAKE_FROM_CENTER), y - (cos(angle_rad) * INTAKE_FROM_CENTER) });
}

void vision_test() {
    std::shared_ptr<Arm> arm = Arm::get();
    //arm->set_height(3_in);
    //cc->strafe(12_in);
    //cc->tune();
    //cc->driveVector(24_in, 24_in);
    arm->tune();
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
    cc->driveStraight(-12_in);
    cc->driveStraight(18_in);
}

void do_nothing() {
    while (true)
        pros::delay(10);
};

void read_number() {
    while (true) {
        int left = peripherals->leftenc->get();
        int right = peripherals->rightenc->get();
        int mid = peripherals->midenc->get();

        printf("left: %d, right: %d, mid: %d\n\n", left, right, mid);
    }
};

void simpler_four_stack() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();

    // Get to cube
    arm->flipDisable(true);
    auto cubeydelta = (48.2_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y;

    cc->driveStraight(cubeydelta);
    arm->flipDisable(false);
    arm->set_height(2_in); // shuffled height lol was 1.4

    cc->strafe((97.1_in - cc->odom->getState(okapi::StateMode::CARTESIAN).x));

    arm->waitUntilSettled();

    cc->setHeading(0_deg);
    cc->driveStraight(1.6_in); // Actually goes to 49.6, for a tad bit of tolerance
    intake->moveVoltage(12000);
    pros::delay(500);
    //close_claw();
    arm->set_height(4_in);
    cc->driveStraight(-1.7_ft);
    // 11.3_ft : 56.4_in;
    auto large_side_x = (side > 0) ? 11.2_ft : 56.26_in;
    // ok so this is jank but the zones are mirrored weirdly in this game and I don't want to write two routines so here we go
    position_intake_to_point(large_side_x, 5_in);

    //cc->lookToPoint({ large_side + (1_ft * side), cc->odom->getState(okapi::StateMode::CARTESIAN).y });

    //arm->flipDisable(false);
    //arm->set_height(0_in);
    //arm->waitUntilSettled();
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

    // Get first tower large blue
    close_claw();
    arm->set_height(26_in);
    position_intake_to_point(70.3_in, 23.2_in);
    arm->waitUntilSettled();
    open_claw();

    // Now head to four stack
    cc->driveToPoint({ 97.1_in, 49.9_in - (INTAKE_FROM_CENTER + 2_in) });
    arm->set_height(0.5_in);
    position_intake_to_point(97.1_in, 49.9_in);
    arm->waitUntilSettled();
    close_claw();
    arm->set_height(2_in);

    // now stack in large blue side
    position_intake_to_point(11.5_ft, 8_in);
    arm->set_height(1_in);
    open_claw();
    arm->set_height(3_in);

    // Drive to small tower between large zones
    cc->driveToPoint({ 125_in, 70.3_in });
    arm->set_height(0_in);
    position_intake_to_point(112.3_in, 70.3_in);
    arm->waitUntilSettled();
    close_claw();

    // Score cube
    arm->set_height(20_in);
    arm->waitUntilSettled();
    position_intake_to_point(105.7_in, 70.3_in);
    open_claw();

    // get next 4 stack
    cc->driveStraight(-9_in);
    arm->set_height(0.5_in);
    cc->setHeading(-90_deg);
    cc->strafe(90.7_in - cc->odom->getState(okapi::StateMode::CARTESIAN).y);

    position_intake_to_point(97.1_in, 90.7_in);
    arm->waitUntilSettled();
    close_claw();
    arm->set_height(2_in);

    // Score it
    position_intake_to_point(11.5_ft, 10.5_ft);
    arm->set_height(1_in);
    arm->waitUntilSettled();
    open_claw();
    arm->set_height(3_in);

    // Get final tower
    cc->driveToPoint({ 89_in, 117.5_in });
    arm->set_height(1_in);
    position_intake_to_point(77_in, 117.5_in);
    arm->waitUntilSettled();
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
    PIDTuning strafeTuning = PIDTuning(0.0025, 0.0000100, 0.000120);

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
                okapi::imev5GreenTPR }),

        okapi::ChassisScales( // Tracking scales
            { { okapi::inch * 2.75, 8.1 * okapi::inch, // wheel diam, wheelbase diam
                  2 * okapi::millimeter, okapi::inch * 4.125 }, // middle wheel distance, middle wheel diam
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
    configManager->register_auton("skills", skill_auton,
        okapi::OdomState{ 97.1_in, 26.4_in - INTAKE_FROM_CENTER, 0_deg });
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
