#include "main.h"

ChassisControllerHDrive::ChassisControllerHDrive(
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
ChassisControllerHDrive::~ChassisControllerHDrive() {
    disable_controllers();
};
*/

void ChassisControllerHDrive::reset() {
    leftSideStart = leftSide->getPosition();
    rightSideStart = rightSide->getPosition();
    strafeStart = strafeMotor->getPosition();
}

void ChassisControllerHDrive::driveStraight(okapi::QLength distance) {
    driveStraightAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::driveStraightAsync(okapi::QLength distance) {
    disable_controllers();
    straightPID->flipDisable(false);
    anglePID->flipDisable(false);
    mode = ControllerMode::straight;
    reset();

    anglePID->setTarget(0);
    straightPID->setTarget(
        distance.convert(okapi::meter) * scales->straight * straightGearset->ratio);
};

void ChassisControllerHDrive::turnAngle(okapi::QAngle angle) {
    turnAngleAsync(angle);
    waitUntilSettled();
};
void ChassisControllerHDrive::turnAngleAsync(okapi::QAngle angle) {
    disable_controllers();
    turnPID->flipDisable(false);
    reset();
    mode = ControllerMode::turn;

    turnPID->setTarget(angle.convert(okapi::degree) * scales->turn * straightGearset->ratio);
};

void ChassisControllerHDrive::strafe(okapi::QLength distance){};
void ChassisControllerHDrive::strafeAsync(okapi::QLength distance){};

void ChassisControllerHDrive::tune() {
    /*
  std::shared_ptr<okapi::MotorGroup> leftShared = std::move(leftSide);
  std::shared_ptr<okapi::MotorGroup> rightShared = std::move(rightSide);

  std::shared_ptr<okapi::SkidSteerModel>
    model = std::make_unique<okapi::SkidSteerModel>(
      okapi::SkidSteerModel(leftShared,rightShared,
      leftSide->getEncoder(),rightSide->getEncoder(),200.0,12.0));

  //std::shared_ptr<okapi::Motor> strafeShared = std::move(strafeMotor);

  auto straightTuner = okapi::PIDTunerFactory::createPtr(
    leftShared->getEncoder(),
  model, 1*okapi::minute, 720,
  0,0,0, 0.001, 0.001, 0.001);


  //okapi::PIDTuner::Output straightTune = straightTuner->autotune();
  //model.reset();
  */
}

void ChassisControllerHDrive::waitUntilSettled() {
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

bool ChassisControllerHDrive::waitUntilDistanceSettled() {
    while (!straightPID->isSettled() || !turnPID->isSettled()) { // Boolean logic is weird
        if (mode != ControllerMode::straight)
            return false;
        pros::delay(2);
    }
    return true;
};
bool ChassisControllerHDrive::waitUntilTurnSettled() {
    while (!turnPID->isSettled()) { // Boolean logic is weird
        if (mode != ControllerMode::turn)
            return false;
        pros::delay(2);
    }
    return true;
};
bool ChassisControllerHDrive::waitUntilStrafeSettled() {
    while (!strafePID->isSettled()) { // Boolean logic is weird
        if (mode != ControllerMode::strafe)
            return false;
        pros::delay(2);
    }
    return true;
};

void ChassisControllerHDrive::disable_controllers() {
    straightPID->flipDisable(true);
    anglePID->flipDisable(true);
    turnPID->flipDisable(true);
    strafePID->flipDisable(true);
    hypotPID->flipDisable(true);
};

void ChassisControllerHDrive::setMaxVelocity(int speed) {
    maxVelocity = speed;
};
// this code makes my pp very hard

void ChassisControllerHDrive::step() {
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

        //printf("straightOut: %f, leftVelocity:%f\n", straightOut, leftVelocity);
    }

    if (mode == ControllerMode::turn) {
        double turnOut = turnPID->step(turnChange);

        leftVelocity = (double)straightGearset->internalGearset * turnOut;
        rightVelocity = (double)straightGearset->internalGearset * turnOut * -1.0;
    }
    // Speed limits that are updated every loop, awesome!

    printf("distance: %f, angle: %f, turn: %f\n", distance_forward, angleChange, turnChange);
    leftSide->moveVelocity(std::min((int)round(leftVelocity), maxVelocity));
    rightSide->moveVelocity(std::min((int)round(rightVelocity), maxVelocity));
};

void ChassisControllerHDrive::trampoline(void* param) {
    ChassisControllerHDrive* cc;
    if (param) {
        cc = static_cast<ChassisControllerHDrive*>(param);
        // This throws if something has gone wrong
    } else {
        printf("Invalid pointer for trampoline to CC!");
        return;
    } // What do you do here?
    cc->asyncThread();
}

void ChassisControllerHDrive::asyncThread() {
    while (true) {
        this->step();
        pros::delay(this->asyncUpdateDelay);
    }
}

void ChassisControllerHDrive::start_task() {
    if (!task) {
        task = std::make_unique<pros::Task>(pros::Task(trampoline, this,
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
            "Chassis Controller Async Task"));
    }
};
void ChassisControllerHDrive::stop_task() {
    task->remove();
    while (task->get_state() != pros::E_TASK_STATE_DELETED)
        pros::delay(20);
    task.reset();
};
