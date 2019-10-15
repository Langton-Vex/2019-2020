#include "main.h"

ChassisController::ChassisController(
    PIDTuning straightTuning, PIDTuning angleTuning,
    PIDTuning turnTuning, PIDTuning strafeTuning,
    PIDTuning hypotTuning, okapi::MotorGroup ileftSide,
    okapi::MotorGroup irightSide, okapi::Motor istrafe,
    okapi::AbstractMotor::GearsetRatioPair istraightGearset,
    okapi::AbstractMotor::GearsetRatioPair istrafeGearset,
    okapi::ChassisScales iscales) {

    // Here be a wall of constructors
    straightPID = std::make_unique<okapi::IterativePosPIDController>(
        okapi::IterativeControllerFactory::posPID(
            straightTuning.kP, straightTuning.kI, straightTuning.kD));
    anglePID = std::make_unique<okapi::IterativePosPIDController>(
        okapi::IterativeControllerFactory::posPID(
            angleTuning.kP, angleTuning.kI, angleTuning.kD));
    turnPID = std::make_unique<okapi::IterativePosPIDController>(
        okapi::IterativeControllerFactory::posPID(
            turnTuning.kP, turnTuning.kI, turnTuning.kD));
    strafePID = std::make_unique<okapi::IterativePosPIDController>(
        okapi::IterativeControllerFactory::posPID(
            strafeTuning.kP, strafeTuning.kI, strafeTuning.kD));
    hypotPID = std::make_unique<okapi::IterativePosPIDController>(
        okapi::IterativeControllerFactory::posPID(
            hypotTuning.kP, hypotTuning.kI, hypotTuning.kD));

    straightGearset = std::make_unique<okapi::AbstractMotor::GearsetRatioPair>(
        istraightGearset);
    strafeGearset = std::make_unique<okapi::AbstractMotor::GearsetRatioPair>(
        istrafeGearset);
    scales = std::make_unique<okapi::ChassisScales>(
        iscales);

    leftSide = std::make_unique<okapi::MotorGroup>(ileftSide);
    rightSide = std::make_unique<okapi::MotorGroup>(irightSide);
    strafeMotor = std::make_unique<okapi::Motor>(istrafe);

    timeUtil = std::make_unique<okapi::TimeUtil>(okapi::TimeUtilFactory::create());
    settledUtil = timeUtil->getSettledUtil();

    reset();

    disable_controllers();
};

/*
ChassisController::~ChassisController() {
    disable_controllers();
};
*/

void ChassisController::reset() {
    leftSideStart = leftSide->getPosition();
    rightSideStart = rightSide->getPosition();
    strafeStart = strafeMotor->getPosition();
}

void ChassisController::driveStraight(okapi::QLength distance) {
    driveStraightAsync(distance);
    waitUntilSettled();
};
void ChassisController::driveStraightAsync(okapi::QLength distance) {
    disable_controllers();
    straightPID->flipDisable(false);
    anglePID->flipDisable(false);
    mode = ControllerMode::straight;
    reset();

    anglePID->setTarget(0);
    straightPID->setTarget(
        distance.convert(okapi::meter) * scales->straight * straightGearset->ratio);
};

void ChassisController::turnAngle(okapi::QAngle angle) {
    turnAngleAsync(angle);
    waitUntilSettled();
};
void ChassisController::turnAngleAsync(okapi::QAngle angle) {
    disable_controllers();
    turnPID->flipDisable(false);
    reset();
    mode = ControllerMode::turn;

    turnPID->setTarget(angle.convert(okapi::degree) * scales->turn * straightGearset->ratio);
};

void ChassisController::strafe(okapi::QLength distance){};
void ChassisController::strafeAsync(okapi::QLength distance){};

void ChassisController::waitUntilSettled() {
    auto rate = timeUtil->getRate();
    // finally:
    disable_controllers();
};

void ChassisController::waitUntilDistanceSettled(){};
void ChassisController::waitUntilTurnSettled(){};
void ChassisController::waitUntilStrafeSettled(){};

void ChassisController::disable_controllers() {
    straightPID->flipDisable(true);
    anglePID->flipDisable(true);
    turnPID->flipDisable(true);
    strafePID->flipDisable(true);
    hypotPID->flipDisable(true);
};

void ChassisController::step() {
    double distance_forward = ((leftSide->getPosition() - leftSideStart) + (rightSide->getPosition() - rightSideStart)) / 2.0;
    double angleChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart));
    double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;

    if (mode == ControllerMode::straight) {
        double straightOut = straightPID->step(distance_forward);
        double angleOut = anglePID->step(angleChange);

        double leftVelocity = straightGearset->gearset * (straightOut - angleOut);
        double rightVelocity = straightGearset->gearset * (straightOut + angleOut);

        printf("straightOut: %f, leftVelocity:%f\n", straightOut, leftVelocity);

        leftSide->moveVelocity(leftVelocity);
        rightSide->moveVelocity(rightVelocity);
    }

    if (mode == ControllerMode::turn) {
        double turnOut = turnPID->step(turnChange);

        double leftVelocity = straightGearset->ratio * turnOut;
        double rightVelocity = straightGearset->ratio * turnOut * -1.0;

        leftSide->moveVelocity(leftVelocity);
        rightSide->moveVelocity(rightVelocity);
    }
};
void start_task(){};
void stop_task(){};
