#include "main.h"
#include <vector>

// Now achieving triple level commenting, this whole controller needs works, please
// remember to take the robot home over Christmas and work on this garbage fire of
// A chassis controller and turn it into something beautiful with odom and motion profiling

/* TODO:
   * Probably need to split this class into multiple files, because it's getting huge
   * Get the PID control working obviously.
   * Get odom working, and well, probably with tracking wheels if we can.
   * Add motion profiling? (oh god)
   * clean up some of the code.
   * Double check logic so that it all works as expected.
   * Take the tuner function and make it a lot more flexible, i.e if it were in
     A cold package / library, make each element toggleable, ranges tweakable, etc.
     Current solution of updating tuning function is a little hacky.
   * Ok so we can solve the DR4B shifts weight problem in two ways:
      * Motion profiling, or a velocity controller (?) will eventually do anyway.
      * limit acceleration by looping and solving (final velocity)2 - (initial velocity)2 = 2 × acceleration × distance
        for distance, are we above / below / about to cross the boundary? We can start slowing then.
   * Very large amounts of this class use hard coded values, or make lots of assumptions about the programmer / external state
     for example, gear ratios, motor speeds, motor encoder units, etc etc.  For a workable API that we can use in the future
     This needs to be updated.
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
    okapi::ChassisScales iscales, okapi::ChassisScales itrackingscales) {

    pros::delay(100); // Just a good idea

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
    chassisScales = std::make_unique<okapi::ChassisScales>(
        iscales);
    trackingScales = std::make_unique<okapi::ChassisScales>(
        itrackingscales);

    leftSide = std::make_unique<okapi::MotorGroup>(ileftSide);
    rightSide = std::make_unique<okapi::MotorGroup>(irightSide);
    strafeMotor = std::make_unique<okapi::Motor>(istrafe);

    timeUtil = std::make_unique<okapi::TimeUtil>(okapi::TimeUtilFactory::createDefault());
    settledUtil = timeUtil->getSettledUtil();

    model = std::make_unique<okapi::HDriveModel>(leftSide, rightSide, strafeMotor,
        peripherals->leftenc, peripherals->rightenc, peripherals->midenc,
        maxVelocity, maxVoltage);
    odom = std::make_unique<okapi::ThreeEncoderOdometry>(okapi::TimeUtilFactory::createDefault(),
        std::static_pointer_cast<okapi::ReadOnlyChassisModel>(model), trackingScales);

    reset();

    disable_controllers();
};

ChassisControllerHDrive::~ChassisControllerHDrive() {
    disable_controllers();
    stop_task();
    // smart pointers should be automatically destroyed
};

void ChassisControllerHDrive::reset() {
    leftSideStart = peripherals->leftenc.get();
    rightSideStart = peripherals->rightenc.get();
    strafeStart = peripherals->leftenc.get();
    mode.clear();

    leftSide->moveVelocity(0);
    rightSide->moveVelocity(0);
    strafeMotor->moveVelocity(0);

    currentMaxVelocity = 0.0;

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
    reset();
    mode.push_back(ControllerMode::straight);
    mode.push_back(ControllerMode::angle);
    anglePID->setTarget(0);
    straightPID->setTarget(
        distance.convert(okapi::meter) * chassisScales->straight * straightGearset->ratio);
};

void ChassisControllerHDrive::turnAngle(okapi::QAngle angle) {
    turnAngleAsync(angle);
    waitUntilSettled();
};
void ChassisControllerHDrive::turnAngleAsync(okapi::QAngle angle) {
    //disable_controllers();
    turnPID->flipDisable(false);
    mode.push_back(ControllerMode::turn);

    turnPID->setTarget(angle.convert(okapi::degree) * chassisScales->turn * straightGearset->ratio);
};

void ChassisControllerHDrive::enableTurn(okapi::QAngle angle) {
    turnPID->flipDisable(false);
    mode.push_back(ControllerMode::turn);

    turnPID->setTarget(angle.convert(okapi::degree) * chassisScales->turn * straightGearset->ratio);
};

void ChassisControllerHDrive::changeTurn(okapi::QAngle angle) {
    turnPID->setTarget(angle.convert(okapi::degree) * chassisScales->turn * straightGearset->ratio);
};

void ChassisControllerHDrive::strafe(okapi::QLength distance) {
    strafeAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::strafeAsync(okapi::QLength distance) {
    //disable_controllers();
    strafePID->flipDisable(false);
    anglePID->flipDisable(false);

    mode.push_back(ControllerMode::strafe);
    mode.push_back(ControllerMode::angle);
    anglePID->setTarget(0);
    strafePID->setTarget(
        distance.convert(okapi::meter) * chassisScales->middle * strafeGearset->ratio);
};

void ChassisControllerHDrive::driveToPoint(okapi::Point point, bool turnreversed) {
    driveToPointAsync(point, turnreversed);
    waitUntilSettled();
};

void ChassisControllerHDrive::driveToPointAsync(okapi::Point point, bool turnreversed) {
    auto [length, angle] = okapi::OdomMath::computeDistanceAndAngleToPoint(
        point.inFT(okapi::StateMode::CARTESIAN), odom->getState(okapi::StateMode::FRAME_TRANSFORMATION));
    turnAngle(angle);
    driveStraightAsync(length);
};

void ChassisControllerHDrive::driveVector(okapi::QLength straight, okapi::QLength strafe) {
    driveVectorAsync(straight, strafe);
    waitUntilSettled();
};

void ChassisControllerHDrive::lookToPoint(okapi::Point point, bool turnreversed) {
    lookToPointAsync(point, turnreversed);
    waitUntilSettled();
};

void ChassisControllerHDrive::lookToPointAsync(okapi::Point point, bool turnreversed) {
    auto angle = okapi::OdomMath::computeAngleToPoint(
        point.inFT(okapi::StateMode::CARTESIAN), odom->getState(okapi::StateMode::FRAME_TRANSFORMATION));

    if (turnreversed)
        turnAngle(-angle);
    else
        turnAngle(angle);
};

void ChassisControllerHDrive::setHeading(okapi::QAngle angle) {
    setHeadingAsync(angle);
    waitUntilSettled();
};

void ChassisControllerHDrive::setHeadingAsync(okapi::QAngle angle) {

    okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
    fprintf(stderr, "state angle: %f\n", state.theta.convert(okapi::degree));
    auto delta = state.theta - angle;
    delta *= -1;
    fprintf(stderr, "delta angle: %f\n", delta.convert(okapi::degree));
    turnAngle(delta);
};

void ChassisControllerHDrive::driveVectorAsync(okapi::QLength straight, okapi::QLength strafe) {
    driveStraightAsync(straight);
    strafeAsync(strafe);
};

void ChassisControllerHDrive::diagToPointAsync(okapi::Point point) {
    okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
    double angle_rad = state.theta.convert(okapi::radian);
    auto strafeDistance = (cos(angle_rad) * (point.x - state.x)) + (sin(angle_rad) * (point.y - state.y));
    auto forwardDistance = (cos(angle_rad) * (point.y - state.y)) + (sin(angle_rad) * (point.x - state.x));
    driveVectorAsync(forwardDistance, strafeDistance);
};
void ChassisControllerHDrive::diagToPoint(okapi::Point point) {
    diagToPointAsync(point);
    waitUntilSettled();
};
// Partially yeeted from okapi ty okapi <3
void ChassisControllerHDrive::generatePath(std::initializer_list<okapi::PathfinderPoint> iwaypoints, const std::string& ipathId) {
    if (iwaypoints.size() == 0)
        return;

    std::vector<Waypoint> points;
    points.reserve(iwaypoints.size());
    for (auto& point : iwaypoints) {
        points.push_back(
            Waypoint{ point.x.convert(okapi::meter), point.y.convert(okapi::meter), point.theta.convert(okapi::radian) });
    }
    TrajectoryCandidate candidate;
    // Arguments:
    // Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
    // Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
    //                      PATHFINDER_SAMPLES_LOW  (10 000)
    //                      PATHFINDER_SAMPLES_FAST (1 000)
    pathfinder_prepare(points.data(), points.size(), FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.010, plimits.maxVel, plimits.maxAccel, plimits.maxJerk, &candidate);

    int length = candidate.length;

    // Array of Segments (the trajectory points) to store the trajectory in
    SegmentPtr trajectory(static_cast<Segment*>(malloc(length * sizeof(Segment))), free);
    SegmentPtr leftTrajectory((Segment*)malloc(sizeof(Segment) * length), free);
    SegmentPtr rightTrajectory((Segment*)malloc(sizeof(Segment) * length), free);

    // Generate the trajectory
    pathfinder_generate(&candidate, trajectory.get());
    pathfinder_modify_tank(trajectory.get(), length, leftTrajectory.get(), rightTrajectory.get(), chassisScales->wheelTrack.convert(okapi::meter));

    // In case
    removePath(ipathId);

    paths.emplace(ipathId,
        TrajectoryPair{ std::move(leftTrajectory), std::move(rightTrajectory), length });
};
void ChassisControllerHDrive::removePath(const std::string& ipathId) {
    if (currentPath == ipathId)
        return;
    auto oldPath = paths.find(ipathId);
    if (oldPath != paths.end()) {
        paths.erase(ipathId);
    }
};
// This function is blocking!
void ChassisControllerHDrive::runPath(const std::string& ipathId, bool reversed, bool mirrored) {
    turnPID->flipDisable(false);
    reset();

    // Probably not needed
    mode.push_back(ControllerMode::turn);
    //mode.push_back(ControllerMode::pathfinderProfile);

    auto rate = timeUtil->getRate();

    TrajectoryPair& path = paths.find(ipathId)->second;
    const int pathLength = path.length;

    EncoderFollower left_follower;
    left_follower.last_error = 0;
    left_follower.segment = 0;
    left_follower.finished = 0;
    EncoderFollower right_follower;
    right_follower.last_error = 0;
    right_follower.segment = 0;
    right_follower.finished = 0;
    // TODO: This assumes everything is in degrees, please consider changing this!!
    EncoderConfig left_config = { (int)leftSide->getPosition(), 360, chassisScales->straight, // Position, Ticks per Rev, Wheel Circumference
        0.8, 0.0, 0.0, 1.0 / plimits.maxVel, 0.0 }; // Kp, Ki, Kd and Kv, Ka

    EncoderConfig right_config = { (int)rightSide->getPosition(), 360, chassisScales->straight, // Position, Ticks per Rev, Wheel Circumference
        0.8, 0.0, 0.0, 1.0 / plimits.maxVel, 0.0 }; // Kp, Ki, Kd and Kv, Ka

    /* This is jank, should be in step, this entire project needs refactoring into
       mutiple files and step states need to be in different functions */
    stop_task();
    while (!left_follower.finished || !right_follower.finished) {
        const auto segDT = path.left.get()[0].dt * okapi::second;
        //const auto linear = path.segments.get()[i].velocity * okapi::mps;
        // NOTE: We probably want this?? I don't know until I try
        double l = (double)straightGearset->internalGearset * pathfinder_follow_encoder(left_config, &left_follower, path.left.get(), pathLength, (int)leftSide->getPosition());
        double r = (double)straightGearset->internalGearset * pathfinder_follow_encoder(right_config, &right_follower, path.right.get(), pathLength, (int)rightSide->getPosition());

        //const auto LinearToRot = (360 * okapi::degree / (scales->wheelDiameter * 1 * okapi::pi)) * straightGearset->ratio;
        //const auto chassisRPM = (linear * LinearToRot).convert(okapi::rpm);
        //const double speed = chassisRPM / toUnderlyingType(straightGearset->internalGearset) * reversed;

        // In degrees
        double heading = (left_follower.heading * okapi::radian).convert(okapi::degree);
        if (heading > 180) {
            heading = (360.0 - heading);
        }

        turnPID->setTarget(heading * chassisScales->turn * straightGearset->ratio);
        double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;
        double turnOut = turnPID->step(turnChange);
        double turnVelocity = (double)straightGearset->internalGearset * turnOut;

        int leftVelocity;
        int rightVelocity;
        if (mirrored) {
            leftVelocity = (int)(l - turnVelocity);
            rightVelocity = (int)(r + turnVelocity);
        } else {
            leftVelocity = (int)(l + turnVelocity);
            rightVelocity = (int)(r - turnVelocity);
        }
        leftSide->moveVelocity(leftVelocity);
        rightSide->moveVelocity(rightVelocity);

        rate->delayUntil(segDT);
    }
    start_task();
    waitUntilSettled();
};

void ChassisControllerHDrive::diagToPointAndTurn(okapi::Point point, okapi::QAngle angle){};
void ChassisControllerHDrive::diagToPointAndTurnAsync(okapi::Point point, okapi::QAngle angle){};

/* ----------------------------------------------------------------
   TUNING SECTION
   ---------------------------------------------------------------- */

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
    stop_task();
    //std::shared_ptr<okapi::Motor> strafeShared = std::move(strafeMotor);

    okapi::Logger::setDefaultLogger(
        std::make_shared<okapi::Logger>(
            okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
            "/ser/sout", // Output to the PROS terminal
            okapi::Logger::LogLevel::info // Show errors and warnings
            ));

    std::shared_ptr<ChassisControllerHDrive> ct(this);
    //P,P, I, I, D, D
    auto StraightTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 4 * okapi::second, 3500,
        0.0005, 0.002, 0, 0, 0, 0.0001);

    auto AngleTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 4 * okapi::second, 0,
        0.0005, 0.001, 0, 0, 0, 0.0001);
    auto TurnTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 8 * okapi::second, 1035,
        0.0005, 0.004, 0, 0, 0, 0.00005);
    auto StrafeTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 7 * okapi::second, 2000,
        0.0045, 0.0065, 0, 0, 0.00007, 0.00014,
        5, 8); // Num iterations and num particles, default 5, 16

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

    TurnTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 8 * okapi::second, 1035,
        0.0005, 0.004, 0, 0, 0, 0.0001);
    TurnTuner->autotune();
    */

    printf("strafe tuning\n");
    tuningMode = TuningMode::TuneStrafe;
    okapi::PIDTuner::Output strafeTune = StrafeTuner->autotune();
    std::string strafeValue = TuningToString(strafeTune);
    fprintf(stderr, "strafe value: %s\n", strafeValue.c_str());
    pros::delay(100);
    while (true) {
        fprintf(stderr, "strafe value: %s\n", strafeValue.c_str());
        pros::delay(2000);
    }

    pros::delay(10000);
    auto s = okapi::PIDTunerFactory::createPtr(
        ct, ct, 7 * okapi::second, 3000,
        0.002, 0.004, 0, 0, 0, 0.0001);
    okapi::PIDTuner::Output t = StrafeTuner->autotune();

    std::shared_ptr<GUI> gui = GUI::get();
    gui->add_line(strafeValue);
    /*
    gui->add_line(straightValue);
    gui->add_line(angleValue);
    gui->add_line(turnValue);
    gui->add_line(strafeValue);
    */

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
    pros::delay(5);
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
    while (!straightPID->isSettled()) {
        pros::delay(2);
    }
    straightPID->flipDisable(true);
    return true;
};

bool ChassisControllerHDrive::waitUntilAngleSettled() {
    while (!anglePID->isSettled()) {
        pros::delay(2);
    }
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
    okapi::OdomState state = odom->getState();
    //fprintf(stderr, "odom: %f, %f, %f\n", state.x.convert(okapi::meter), state.y.convert(okapi::meter), state.theta.convert(okapi::degree));

    double distance_forward = ((peripherals->leftenc.get() - leftSideStart) + (peripherals->rightenc.get() - rightSideStart)) / 2.0;
    double angleChange = ((peripherals->leftenc.get() - leftSideStart) - (peripherals->rightenc.get() - rightSideStart));
    double turnChange = ((peripherals->leftenc.get() - leftSideStart) - (peripherals->rightenc.get() - rightSideStart)) / 2.0;
    double strafeChange = peripherals->midenc.get() - strafeStart;

    distance_forward *= trackingScales->straight / chassisScales->straight;
    angleChange *= trackingScales->turn / chassisScales->turn;
    turnChange *= trackingScales->turn / chassisScales->turn;
    strafeChange *= trackingScales->middle / chassisScales->middle;

    double leftVelocity = 0;
    double rightVelocity = 0;
    double strafeVelocity = 0;
    std::shared_ptr<Chassis> chassis = Chassis::get();
    if (mode.size() > 0) {
        currentMaxVelocity += (double)maxAccel * (double)asyncUpdateDelay * 0.001;
        currentMaxVelocity = (double)std::min(currentMaxVelocity, (double)maxVelocity);
        //currentMaxVelocity = currentMaxVelocity * chassis->power_mult_calc();
    }

    if (std::find(mode.begin(), mode.end(), ControllerMode::straight) != mode.end()) {
        double straightOut = straightPID->step(distance_forward);

        //printf("%f straight error, %f angle error\n", straightPID->getError(), anglePID->getError());
        leftVelocity += (double)straightGearset->internalGearset * straightOut;
        rightVelocity += (double)straightGearset->internalGearset * straightOut;

        //printf("straightOut: %f, leftVelocity:%f\n", straightOut, leftVelocity);
    }

    if (std::find(mode.begin(), mode.end(), ControllerMode::turn) != mode.end()) {
        double turnOut = turnPID->step(turnChange);
        printf("%f turn error\n", turnPID->getError());
        leftVelocity += (double)straightGearset->internalGearset * turnOut;
        rightVelocity -= (double)straightGearset->internalGearset * turnOut;
    }

    if (std::find(mode.begin(), mode.end(), ControllerMode::strafe) != mode.end()) {
        double strafeOut = strafePID->step(strafeChange);

        strafeVelocity += (double)strafeGearset->internalGearset * strafeOut;
    }

    if (std::find(mode.begin(), mode.end(), ControllerMode::angle) != mode.end()) {
        double angleOut = anglePID->step(angleChange);

        leftVelocity += (double)straightGearset->internalGearset * angleOut;
        leftVelocity -= (double)straightGearset->internalGearset * angleOut;
    }

    // Speed limits that are updated every loop, awesome!

    //printf("distance: %f, angle: %f, turn: %f\n", distance_forward, angleChange, turnChange);
    if (leftVelocity >= 0)
        leftVelocity = std::min(leftVelocity, currentMaxVelocity);
    else
        leftVelocity = std::max(leftVelocity, -currentMaxVelocity);
    if (rightVelocity >= 0)
        rightVelocity = std::min(rightVelocity, currentMaxVelocity);
    else
        rightVelocity = std::max(rightVelocity, -currentMaxVelocity);

    if (strafeVelocity >= 0)
        strafeVelocity = std::min(strafeVelocity, currentMaxVelocity);
    else
        strafeVelocity = std::max(strafeVelocity, -currentMaxVelocity);

    leftSide->moveVelocity((int)leftVelocity);
    rightSide->moveVelocity((int)rightVelocity);
    strafeMotor->moveVelocity((int)strafeVelocity);
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
        pros::delay(asyncUpdateDelay);
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
