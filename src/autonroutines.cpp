#include "main.h"
using namespace okapi;

extern std::shared_ptr<okapi::ChassisController> ccont;
extern std::shared_ptr<Motor> intake;
extern std::shared_ptr<ChassisControllerHDrive> cc;
extern pros::ADIAnalogIn arm_pot;

okapi::QLength INTAKE_FROM_CENTER = 12.5_in;

void vision_test() {
    fprintf(stderr, "waiting for yeet");

    /*
    cc->driveToPoint({ 2_ft, 0_ft });
    cc->driveToPoint({ 0_ft, 0_ft });
    cc->lookToPoint({ 1_ft, 0_ft });
    */
    cc->generatePath({ { 0_ft, 0_ft, 0_deg }, { 2_ft, 0_ft, 0_deg } }, "A");
    cc->runPath("A", false, false);
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
    //vision_signature_s_t sig = pros::Vision::signature_from_utility ( 1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0 );
    //vision::signature SIG_1 (1, 607, 2287, 1446, 6913, 10321, 8618, 3.000, 0);
}

void do_nothing() {
    while (true)
        pros::delay(100);
};

void move_15() {
    cc->driveStraight(18_in);
    cc->driveStraight(-18_in);
}

void arm_test() {
    //cc->setHeading(90_deg);
    cc->strafe(6_in);
    return;
    std::shared_ptr<Arm> arm = Arm::get();
    arm->set_height(10_in);
    arm->waitUntilSettled();
}

void pot_lookup() {
    cc->stop_task();
    Arm::get()->flipDisable(true);
    fprintf(stderr, "starting");
    std::ofstream save_file("/usd/yeet.csv", std::ofstream::out | std::ofstream::trunc);
    save_file.clear();
    peripherals->leftarm_mtr.move_velocity(30);
    peripherals->rightarm_mtr.move_velocity(30);
    for (int i = 0; i < 500; i++) {
        save_file << peripherals->leftarm_mtr.get_position() << ",";
        save_file << arm_pot.get_value() << "\n";
        pros::delay(10);
    }
    save_file.close();
    fprintf(stderr, "stopped");
};

/* ----------------------------------------------------------------
   competition routines
   ----------------------------------------------------------------*/

void four_stack() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();

    // Get to cube
    arm->flipDisable(true);
    auto cubeydelta = (48.2_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y;
    cc->driveStraight(cubeydelta);
    arm->flipDisable(false);
    arm->set_height(2.3_in);
    cc->strafe((97.1_in - cc->odom->getState(okapi::StateMode::CARTESIAN).x) + (3_in * side * -1));
    arm->waitUntilSettled();

    cc->setHeading(0_deg);
    cc->driveStraight(1.2_in);
    intake->moveVoltage(12000);
    pros::delay(500);
    arm->set_height(7_in);
    cc->driveStraight(-1.5_ft);
    auto large_side = (side > 0) ? 11.5_ft - INTAKE_FROM_CENTER : 58.6_in + INTAKE_FROM_CENTER;
    cc->driveToPoint({ large_side, 9_in });
    cc->lookToPoint({ large_side + (1_ft * side), cc->odom->getState(okapi::StateMode::CARTESIAN).y });

    //cc->driveStraight(1_in); // NOTE: this shouldn't be necessary but is
    //cc->strafe(0.2_ft- cc->odom->getState(okapi::StateMode::CARTESIAN).x );
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
    cc->driveStraight(-2_in);
    arm->set(0);
}

void seven_stack() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();
    okapi::QAngle inward;
    if (side < 0)
        inward = 180_deg;
    else
        inward = 0_deg;

    auto cubeydelta = (49.9_in - INTAKE_FROM_CENTER) - cc->odom->getState(okapi::StateMode::CARTESIAN).y;
    cc->driveStraight(cubeydelta);
    arm->set_height(2.5_in);
    cc->strafe((70.3_in + 2.5_in) - cc->odom->getState(okapi::StateMode::CARTESIAN).x);
    arm->waitUntilSettled();
    intake->moveVoltage(12000);

    cc->driveToPoint({ 11.5_ft - INTAKE_FROM_CENTER, 1_ft });
    cc->lookToPoint({ 13_ft, 1_ft });
    arm->flipDisable(false);
    arm->set_height(0_in);
    arm->waitUntilSettled();
    arm->flipDisable(true);
    arm->set(-200);
    pros::delay(500);
    intake->moveVoltage(-12000);
    pros::delay(500);
    arm->set(0);
    intake->moveVoltage(0);
    pros::delay(500);
    cc->driveStraightAsync(-5_in);
    arm->set_height(0.1_in);
    cc->waitUntilSettled();

    cc->driveToPoint({ 105.7_in, 49.9_in - INTAKE_FROM_CENTER });
    arm->set_height(2.5_in);
    cc->setHeading(inward);
    cc->strafe(97.1_in - cc->odom->getState(okapi::StateMode::CARTESIAN).x);
    arm->waitUntilSettled();

    intake->moveVoltage(12000);
    pros::delay(500);
    arm->set_height(7_in);
    cc->driveToPoint({ 1_ft, 11.5_ft - INTAKE_FROM_CENTER });
    arm->set_height(22_in);
    arm->waitUntilSettled();
    cc->lookToPoint({ 13_ft, 1_ft });

    arm->set_height(21_in);
    pros::delay(100);
    intake->moveVoltage(-12000);
    arm->waitUntilSettled();
    cc->driveStraight(-1_in);
}

void four_floor() {
    int side = ConfigManager::get()->selected_team;
    std::shared_ptr<Arm> arm = Arm::get();
    okapi::QAngle inward;
    if (side < 0)
        inward = 180_deg;
    else
        inward = 0_deg;

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
    intake->moveVoltage(12000);

    cc->driveToPoint({ 0.75_ft, 0.75_ft - INTAKE_FROM_CENTER });
    arm->set_height(0_in);
    arm->waitUntilSettled();
    intake->moveVoltage(-12000);
}
