#include "main.h"
#include <vector>

/* TODO:
   * Probably need to split this class into multiple files, because it's getting huge
   * Get the PID control working obviously.
   * Get odom working, and well, probably with tracking wheels if we can.
   * Add motion profiling? (oh god)
   * clean up some of the code.
   * Double check logic so that it all works as expected.
*/

/* Remember with this controller that if you are doing two movements
 * Consecutively, you must WaitUntilSettled between them or it will try to move
 * To an absolute position!
 * This can't be fixed between now and the next competition lol, not sure how
 * I would even fix this
 * Just make sure that you don't do anything to complicated
 */
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

ChassisControllerHDrive::~ChassisControllerHDrive() {
    disable_controllers();
    stop_task();
    // smart pointers should be automatically destroyed
};

void ChassisControllerHDrive::reset() {
    leftSideStart = leftSide->getPosition();
    rightSideStart = rightSide->getPosition();
    strafeStart = strafeMotor->getPosition();
    mode.clear();

    leftSide->moveVelocity(0);
    rightSide->moveVelocity(0);
    strafeMotor->moveVelocity(0);

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
    mode.push_back(ControllerMode::straight);

    reset();
    mode.push_back(ControllerMode::straight);
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
    mode.push_back(ControllerMode::turn);

    turnPID->setTarget(angle.convert(okapi::degree) * scales->turn * straightGearset->ratio);
};

void ChassisControllerHDrive::strafe(okapi::QLength distance) {
    strafeAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::strafeAsync(okapi::QLength distance) {
    //disable_controllers();
    strafePID->flipDisable(false);
    mode.push_back(ControllerMode::strafe);

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

    okapi::Logger::setDefaultLogger(
    std::make_shared<okapi::Logger>(
        okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
        "/ser/sout", // Output to the PROS terminal
        okapi::Logger::LogLevel::info // Show errors and warnings
    )
);

    std::shared_ptr<ChassisControllerHDrive> ct(this);
    //P,P, I, I, D, D
    auto StraightTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 4 * okapi::second, 3500,
        0.0005, 0.002, 0, 0, 0, 0.0001);

    auto AngleTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 4 * okapi::second, 0,
        0.0005, 0.001, 0, 0, 0, 0.0001);
    auto TurnTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 4 * okapi::second, 3500,
        0.0005, 0.003, 0, 0, 0, 0.0001);
    auto StrafeTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 7 * okapi::second, 3000,
        0.002, 0.004, 0, 0, 0, 0.0001);

    /*
    tuningMode = TuningMode::TuneStraight;
    printf("straight tuning\n");
    okapi::PIDTuner::Output straightTune = StraightTuner->autotune();
    std::string straightValue = TuningToString(straightTune);
    printf("straight value: %s\n", straightValue.c_str());


    printf("angle tuning\n");
    tuningMode = TuningMode::TuneAngle;
    okapi::PIDTuner::Output angleTune = AngleTuner->autotune();
    std::string angleValue = TuningToString(angleTune);
    printf("angle value: %s\n", angleValue.c_str());

    printf("turn tuning\n");
    tuningMode = TuningMode::TuneTurn;
    okapi::PIDTuner::Output turnTune = TurnTuner->autotune();
    std::string turnValue = TuningToString(turnTune);
    printf("turn value: %s\n", turnValue.c_str());

    */
    printf("strafe tuning\n");
    tuningMode = TuningMode::TuneStrafe;
    okapi::PIDTuner::Output strafeTune = StrafeTuner->autotune();
    std::string strafeValue = TuningToString(strafeTune);
    printf("strafe value: %s\n", strafeValue.c_str());

    std::shared_ptr<GUI> gui = GUI::get();
    /*
    gui->add_line(straightValue);
    gui->add_line(angleValue);
    gui->add_line(turnValue);
    */
    gui->add_line(strafeValue);


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
    case TuningMode::TuneStrafe:
        return strafeMotor->getPosition() - strafeStart;
    default:
        return 0.0;
    }
}

void ChassisControllerHDrive::controllerSet(double ivalue) {
    double leftVelocity = 0;
    double rightVelocity = 0;
    double strafeVelocity = 0;

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
        leftVelocity = (double)straightGearset->internalGearset * (ivalue);
        rightVelocity = (double)straightGearset->internalGearset * (-ivalue);
        leftVelocity += 50;
        rightVelocity += 50;
    }
    if (tuningMode == TuningMode::TuneStrafe) {
        strafeVelocity = (double)strafeGearset->internalGearset * ivalue;

    }
    //fprintf(stderr, "out: %d", std::min((int)round(leftVelocity), maxVelocity));

    leftSide->moveVelocity(std::min((int)round(leftVelocity), maxVelocity));
    rightSide->moveVelocity(std::min((int)round(rightVelocity), maxVelocity));
    strafeMotor->moveVelocity(std::min((int)round(strafeVelocity), maxVelocity));
};

void ChassisControllerHDrive::waitUntilSettled() {
    if (!mode.empty()) {
        std::remove_if(mode.begin(), mode.end(), [this](ControllerMode a) {
            if (a == ControllerMode::straight) {
                fprintf(stderr, "waiting for straight");
                return waitUntilDistanceSettled();
            }
            if (a == ControllerMode::turn) {
                fprintf(stderr, "waiting for turn");
                return waitUntilTurnSettled();
            }
            if (a == ControllerMode::strafe) {
                fprintf(stderr, "waiting for strafe");
                return waitUntilStrafeSettled();
            }
            return false; // What to do here?
        });
    }
    // finally:
    disable_controllers();
    reset();
};

bool ChassisControllerHDrive::waitUntilDistanceSettled() {
    while (!straightPID->isSettled() || !anglePID->isSettled()) { // Boolean logic is weird
        pros::delay(2);
    }
    straightPID->flipDisable(true);
    anglePID->flipDisable(true);
    return true;
};
bool ChassisControllerHDrive::waitUntilTurnSettled() {
    while (!turnPID->isSettled()) {
        pros::delay(2);
    }
    turnPID->flipDisable(true);
    return true;
};
bool ChassisControllerHDrive::waitUntilStrafeSettled() {
    while (!strafePID->isSettled()) {
        pros::delay(2);
    }
    strafePID->flipDisable(true);
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
    printf("\n%d\n", mode[0]);
    if (std::find(mode.begin(), mode.end(), ControllerMode::straight) != mode.end()) {
        double straightOut = straightPID->step(distance_forward);
        double angleOut = anglePID->step(angleChange);
        printf("%f straight error, %f angle error\n",straightPID->getError(), anglePID->getError());
        leftVelocity += (double)straightGearset->internalGearset * (straightOut + angleOut);
        rightVelocity += (double)straightGearset->internalGearset * (straightOut - angleOut);

        //printf("straightOut: %f, leftVelocity:%f\n", straightOut, leftVelocity);
    }

    if (std::find(mode.begin(), mode.end(), ControllerMode::turn) != mode.end()) {
        double turnOut = turnPID->step(turnChange);
        printf("%f turn error\n",turnPID->getError());
        leftVelocity += (double)straightGearset->internalGearset * turnOut;
        rightVelocity += (double)straightGearset->internalGearset * turnOut * -1.0;
    }

    if (std::find(mode.begin(), mode.end(), ControllerMode::strafe) != mode.end()) {
        double strafeOut = strafePID->step(turnChange);

        strafeVelocity += (double)strafeGearset->internalGearset * strafeOut;
    }
    // Speed limits that are updated every loop, awesome!

    //printf("distance: %f, angle: %f, turn: %f\n", distance_forward, angleChange, turnChange);
    leftSide->moveVelocity(std::min((int)round(leftVelocity), maxVelocity));
    rightSide->moveVelocity(std::min((int)round(rightVelocity), maxVelocity));
    strafeMotor->moveVelocity(std::min((int)round(strafeVelocity), maxVelocity));
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
    while (task_running) {
        this->step();
        pros::delay(this->asyncUpdateDelay);
    }
}

void ChassisControllerHDrive::start_task() {
    if (!task) {
        task_running = true;
        task = std::make_unique<pros::Task>(pros::Task(trampoline, this,
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
            "Chassis Controller Async Task"));
    }
};
void ChassisControllerHDrive::stop_task() {
    if (task) {
        task_running = false;
        task.reset();
    }
};
