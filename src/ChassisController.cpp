#include "main.h"
#include <vector>

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

    timeUtil = std::make_unique<okapi::TimeUtil>(okapi::TimeUtilFactory::createDefault());
    settledUtil = timeUtil->getSettledUtil();

    model = std::make_unique<okapi::SkidSteerModel>(leftSide, rightSide,
        leftSide->getEncoder(), rightSide->getEncoder(), maxVelocity, maxVoltage);

    odom = std::make_unique<okapi::Odometry>(okapi::TimeUtilFactory::createDefault(),
        std::static_pointer_cast<okapi::ReadOnlyChassisModel>(model), *scales);

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
    mode.clear();

    straightPID->reset();
    anglePID->reset();
    turnPID->reset();
    strafePID->reset();
    hypotPID->reset();
}

void ChassisControllerHDrive::driveStraight(okapi::QLength distance) {
    driveStraightAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::driveStraightAsync(okapi::QLength distance) {
    //disable_controllers();
    straightPID->flipDisable(false);
    anglePID->flipDisable(false);
    mode.insert(ControllerMode::straight);
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
    //disable_controllers();
    turnPID->flipDisable(false);
    reset();
    mode.insert(ControllerMode::turn);

    turnPID->setTarget(angle.convert(okapi::degree) * scales->turn * straightGearset->ratio);
};

void ChassisControllerHDrive::strafe(okapi::QLength distance) {
    strafeAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::strafeAsync(okapi::QLength distance) {
    //disable_controllers();
    strafePID->flipDisable(false);
    reset();
    mode.insert(ControllerMode::strafe);

    strafePID->setTarget(
        distance.convert(okapi::meter) * scales->middle * strafeGearset->ratio);
};

void ChassisControllerHDrive::driveVector(okapi::QLength straight, okapi::QLength strafe) {
    driveVectorAsync(straight, strafe);
    waitUntilSettled();
};
void ChassisControllerHDrive::driveVectorAsync(okapi::QLength straight, okapi::QLength strafe) {
    driveStraightAsync(straight);
    strafeAsync(strafe);
};

void ChassisControllerHDrive::diagToPointAsync(okapi::Point point) {
    okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
    driveVectorAsync(point.y - state.y, point.x - state.x);
};
void ChassisControllerHDrive::diagToPoint(okapi::Point point) {
    diagToPointAsync(point);
    waitUntilSettled();
};

void ChassisControllerHDrive::diagToPointAndTurn(okapi::Point point, okapi::QAngle angle){};
void ChassisControllerHDrive::diagToPointAndTurnAsync(okapi::Point point, okapi::QAngle angle){};

std::string TuningToString(okapi::PIDTuner::Output tuning) {
    std::string ret = "Kp: ";
    ret.append(std::to_string(tuning.kP));
    ret.append("\n");

    ret.append("Ki :");
    ret.append(std::to_string(tuning.kI));
    ret.append("\n");

    ret.append("KD :");
    ret.append(std::to_string(tuning.kD));
    ret.append("\n");
    return ret;
}

void ChassisControllerHDrive::tune() {

    //std::shared_ptr<okapi::Motor> strafeShared = std::move(strafeMotor);

    std::shared_ptr<ChassisControllerHDrive> ct(this);

    auto StraightTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 1 * okapi::minute, 1,
        0, 0, 0, 0.001, 0.001, 0.001);
    auto AngleTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 1 * okapi::minute, 0,
        0, 0, 0, 0.001, 0.001, 0.001);
    auto TurnTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 1 * okapi::minute, 1,
        0, 0, 0, 0.001, 0.001, 0.001);

    tuningMode = TuningMode::TuneStraight;
    okapi::PIDTuner::Output straightTune = StraightTuner->autotune();
    tuningMode = TuningMode::TuneAngle;
    okapi::PIDTuner::Output angleTune = AngleTuner->autotune();
    tuningMode = TuningMode::TuneTurn;
    okapi::PIDTuner::Output turnTune = TurnTuner->autotune();

    std::string straightValue = TuningToString(straightTune);
    std::string angleValue = TuningToString(angleTune);
    std::string turnValue = TuningToString(turnTune);

    std::shared_ptr<GUI> gui = GUI::get();
    gui->add_line(straightValue);
    gui->add_line(angleValue);
    gui->add_line(turnValue);

    //model.reset();
}

double ChassisControllerHDrive::controllerGet() {
    switch (tuningMode) {
    case TuningMode::TuneStraight:
        return ((leftSide->getPosition() - leftSideStart) + (rightSide->getPosition() - rightSideStart)) / 2.0;
    case TuningMode::TuneAngle:
        return ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart));
    case TuningMode::TuneTurn:
        return ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;
    default:
        return 0.0;
    }
}

void ChassisControllerHDrive::controllerSet(double ivalue) {
    double leftVelocity = 0;
    double rightVelocity = 0;

    double distance_forward = ((leftSide->getPosition() - leftSideStart) + (rightSide->getPosition() - rightSideStart)) / 2.0;
    double angleChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart));
    double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;

    if (tuningMode == TuningMode::TuneStraight) {
        //double angleOut = anglePID->step(angleChange);

        leftVelocity = (double)straightGearset->internalGearset * ivalue;
        rightVelocity = (double)straightGearset->internalGearset * ivalue;
    }
    if (tuningMode == TuningMode::TuneTurn) {
        //double turnOut = turnPID->step(turnChange);

        leftVelocity = (double)straightGearset->internalGearset * ivalue;
        rightVelocity = (double)straightGearset->internalGearset * ivalue * -1.0;
    }
    if (tuningMode == TuningMode::TuneAngle) {
        leftVelocity = (double)straightGearset->internalGearset * (-ivalue);
        rightVelocity = (double)straightGearset->internalGearset * (ivalue);
    }

    leftSide->moveVelocity(-1 * std::min((int)round(leftVelocity), maxVelocity));
    rightSide->moveVelocity(-1 * std::min((int)round(rightVelocity), maxVelocity));
};

void ChassisControllerHDrive::waitUntilSettled() {
    // Thanks okapi
    bool completelySettled = false;
    while (mode.size() > 0) {
        if (mode.find(ControllerMode::straight) != mode.end())
            completelySettled = waitUntilDistanceSettled();
        if (mode.find(ControllerMode::turn) != mode.end())
            completelySettled = waitUntilTurnSettled();
        if (mode.find(ControllerMode::strafe) != mode.end())
            completelySettled = waitUntilStrafeSettled();

        pros::delay(2); // Just in case
    }

    // finally:
    disable_controllers();
};

bool ChassisControllerHDrive::waitUntilDistanceSettled() {
    while (!straightPID->isSettled() || !turnPID->isSettled()) { // Boolean logic is weird
        if (!(mode.find(ControllerMode::straight) != mode.end()))
            return false;
        pros::delay(2);
    }
    mode.erase(ControllerMode::straight);
    return true;
};
bool ChassisControllerHDrive::waitUntilTurnSettled() {
    while (!turnPID->isSettled()) { // Boolean logic is weird
        if (!(mode.find(ControllerMode::turn) != mode.end()))
            return false;
        pros::delay(2);
    }
    mode.erase(ControllerMode::turn);
    return true;
};
bool ChassisControllerHDrive::waitUntilStrafeSettled() {
    while (!strafePID->isSettled()) { // Boolean logic is weird
        if (!(mode.find(ControllerMode::strafe) != mode.end()))
            return false;
        pros::delay(2);
    }
    mode.erase(ControllerMode::strafe);
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

void ChassisControllerHDrive::step() {
    odom->step();

    double distance_forward = ((leftSide->getPosition() - leftSideStart) + (rightSide->getPosition() - rightSideStart)) / 2.0;
    double angleChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart));
    double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;
    double strafeChange = strafeMotor->getPosition() - strafeStart;

    double leftVelocity = 0;
    double rightVelocity = 0;
    double strafeVelocity = 0;

    if (mode.find(ControllerMode::straight) != mode.end()) {
        double straightOut = straightPID->step(distance_forward);
        double angleOut = anglePID->step(angleChange);

        leftVelocity += (double)straightGearset->internalGearset * (straightOut - angleOut);
        rightVelocity += (double)straightGearset->internalGearset * (straightOut + angleOut);

        //printf("straightOut: %f, leftVelocity:%f\n", straightOut, leftVelocity);
    }

    if (mode.find(ControllerMode::turn) != mode.end()) {
        double turnOut = turnPID->step(turnChange);

        leftVelocity += (double)straightGearset->internalGearset * turnOut;
        rightVelocity += (double)straightGearset->internalGearset * turnOut * -1.0;
    }

    if (mode.find(ControllerMode::strafe) != mode.end()) {
        double strafeOut = strafePID->step(turnChange);

        strafeVelocity += (double)strafeGearset->internalGearset * strafeOut;
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
