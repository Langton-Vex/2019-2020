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
    // Thanks okapi
    bool completelySettled = false;
    while (!completelySettled) {
        switch (mode) {
        case ControllerMode::straight:
            completelySettled = waitUntilDistanceSettled();
            break;
        case ControllerMode::turn:
            completelySettled = waitUntilTurnSettled();
            break;
        case ControllerMode::strafe:
            completelySettled = waitUntilStrafeSettled();
            break;
        case ControllerMode::none:
            completelySettled = true;
            break;
        }
        pros::delay(2); // Just in case
    }
    // finally:
    disable_controllers();
};

bool ChassisController::waitUntilDistanceSettled() {
    while (!straightPID->isSettled() || !turnPID->isSettled()) { // Boolean logic is weird
        if (mode != ControllerMode::straight)
            return false;
        pros::delay(2);
    }
    return true;
};
bool ChassisController::waitUntilTurnSettled() {
    while (!turnPID->isSettled()) { // Boolean logic is weird
        if (mode != ControllerMode::turn)
            return false;
        pros::delay(2);
    }
    return true;
};
bool ChassisController::waitUntilStrafeSettled() {
    while (!strafePID->isSettled()) { // Boolean logic is weird
        if (mode != ControllerMode::strafe)
            return false;
        pros::delay(2);
    }
    return true;
};

void ChassisController::disable_controllers() {
    straightPID->flipDisable(true);
    anglePID->flipDisable(true);
    turnPID->flipDisable(true);
    strafePID->flipDisable(true);
    hypotPID->flipDisable(true);
};

void ChassisController::setMaxVelocity(int speed) {
    maxVelocity = speed;
};

void ChassisController::step() {
    double distance_forward = ((leftSide->getPosition() - leftSideStart) + (rightSide->getPosition() - rightSideStart)) / 2.0;
    double angleChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart));
    double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;

    double leftVelocity = 0;
    double rightVelocity = 0;

    if (mode == ControllerMode::straight) {
        double straightOut = straightPID->step(distance_forward);
        double angleOut = anglePID->step(angleChange);

        leftVelocity = (double)straightGearset->internalGearset * (straightOut - angleOut);
        rightVelocity = (double)straightGearset->internalGearset * (straightOut + angleOut);

        printf("straightOut: %f, leftVelocity:%f\n", straightOut, leftVelocity);
    }

    if (mode == ControllerMode::turn) {
        double turnOut = turnPID->step(turnChange);

        leftVelocity = (double)straightGearset->internalGearset * turnOut;
        rightVelocity = (double)straightGearset->internalGearset * turnOut * -1.0;
    }
    // Speed limits that are updated every loop, awesome!

    leftSide->moveVelocity(std::min((int)round(leftVelocity),maxVelocity));
    rightSide->moveVelocity(std::min((int)round(rightVelocity), maxVelocity));
};

void ChassisController::trampoline(void* param) {
    ChassisController* cc;
    if (param) {
        cc = static_cast<ChassisController*>(param);
        // This throws if something has gone wrong
    } else {
        printf("Invalid pointer for trampoline to CC!");
        return;
    } // What do you do here?
    cc->asyncThread();
}

void ChassisController::asyncThread() {
    while (true) {
        this->step();
        pros::delay(this->asyncUpdateDelay);
    }
}

void ChassisController::start_task() {
    if (!task) {
        task = std::make_unique<pros::Task>(pros::Task(trampoline, this,
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
            "Chassis Controller Async Task"));
    }
};
void ChassisController::stop_task() {
    task->remove();
    while (task->get_state() != pros::E_TASK_STATE_DELETED)
        pros::delay(20);
    task.reset();
};
