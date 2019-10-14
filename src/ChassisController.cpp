#include "main.h"

ChassisController::ChassisController(
    PIDTuning straightTuning, PIDTuning angleTuning,
    PIDTuning turnTuning, PIDTuning strafeTuning,
    PIDTuning hypotTuning, okapi::MotorGroup ileftSide,
    okapi::MotorGroup irightSide, okapi::Motor istrafe) {

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

    leftSide = std::make_unique<okapi::MotorGroup>(ileftSide);
    rightSide = std::make_unique<okapi::MotorGroup>(irightSide);
    strafeMotor = std::make_unique<okapi::Motor>(istrafe);

    disable_controllers();
};

ChassisController::~ChassisController() {
    disable_controllers();
};

void ChassisController::driveStraight(okapi::QLength distance) {
    driveStraightAsync(distance);
    waitUntilSettled();
};
void ChassisController::driveStraightAsync(okapi::QLength distance) {
    disable_controllers();
    straightPID->flipDisable(false);
    anglePID->flipDisable(false);

    anglePID->setTarget(0);
    straightPID->setTarget(
        distance.convert(okapi::meter) * scales->straight * straightGearset->ratio);
};

void ChassisController::turnAngle(okapi::QAngle angle){};
void ChassisController::turnAngleAsync(okapi::QAngle angle){};

void ChassisController::strafe(okapi::QLength distance){};
void ChassisController::strafeAsync(okapi::QLength distance){};

void ChassisController::waitUntilSettled() {
    // finally:
    disable_controllers();
};

void ChassisController::disable_controllers() {
    straightPID->flipDisable(true);
    anglePID->flipDisable(true);
    turnPID->flipDisable(true);
    strafePID->flipDisable(true);
    hypotPID->flipDisable(true);
};

void step(){};
void start_task(){};
void stop_task(){};
