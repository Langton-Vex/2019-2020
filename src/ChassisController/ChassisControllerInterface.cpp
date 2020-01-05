#include "main.h"
#include <math.h>

void ChassisControllerHDrive::setMaxVelocity(int speed) {
    maxVelocity = speed;
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::driveStraight(okapi::QLength distance) {
    driveStraightAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::driveStraightAsync(okapi::QLength distance) {
    //disable_controllers();
    straightPID.flipDisable(false);
    anglePID.flipDisable(false);
    reset();
    mode.push_back(ControllerMode::straight);
    mode.push_back(ControllerMode::angle);
    anglePID.setTarget(0);
    straightPID.setTarget(
        distance.convert(okapi::meter) * scales.straight * straightGearset.ratio);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::turnAngle(okapi::QAngle angle) {
    turnAngleAsync(angle);
    waitUntilSettled();
};
void ChassisControllerHDrive::turnAngleAsync(okapi::QAngle angle) {
    //disable_controllers();
    turnPID.flipDisable(false);
    mode.push_back(ControllerMode::turn);

    turnPID.setTarget(angle.convert(okapi::degree) * scales.turn * straightGearset.ratio);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::enableTurn(okapi::QAngle angle) {
    turnPID.flipDisable(false);
    mode.push_back(ControllerMode::turn);

    turnPID.setTarget(angle.convert(okapi::degree) * scales.turn * straightGearset.ratio);
};

void ChassisControllerHDrive::changeTurn(okapi::QAngle angle) {
    turnPID.setTarget(angle.convert(okapi::degree) * scales.turn * straightGearset.ratio);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::strafe(okapi::QLength distance) {
    strafeAsync(distance);
    waitUntilSettled();
};
void ChassisControllerHDrive::strafeAsync(okapi::QLength distance) {
    //disable_controllers();
    strafePID.flipDisable(false);
    anglePID.flipDisable(false);

    mode.push_back(ControllerMode::strafe);
    mode.push_back(ControllerMode::angle);
    anglePID.setTarget(0);
    strafePID.setTarget(
        distance.convert(okapi::meter) * scales.middle * strafeGearset.ratio);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::driveToPoint(okapi::Point point, bool turnreversed) {
    driveToPointAsync(point, turnreversed);
    waitUntilSettled();
};

void ChassisControllerHDrive::driveToPointAsync(okapi::Point point, bool turnreversed) {
    auto [length, angle] = okapi::OdomMath::computeDistanceAndAngleToPoint(
        point.inFT(okapi::StateMode::CARTESIAN), odom->getState(okapi::StateMode::FRAME_TRANSFORMATION));
    if (turnreversed)
        turnAngle(-angle);
    else
        turnAngle(angle);
    driveStraight(length);
};

// ----------------------------------------------------------------

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

// ----------------------------------------------------------------

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

// ----------------------------------------------------------------

void ChassisControllerHDrive::driveVector(okapi::QLength straight, okapi::QLength strafe) {
    driveVectorAsync(straight, strafe);
    waitUntilSettled();
};

void ChassisControllerHDrive::driveVectorAsync(okapi::QLength straight, okapi::QLength strafe) {
    driveStraightAsync(straight);
    strafeAsync(strafe);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::straightXDistance(okapi::QLength XCoord){
  straightXDistanceAsync(XCoord);
  waitUntilSettled();
};
void ChassisControllerHDrive::straightXDistanceAsync(okapi::QLength XCoord){
  okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
  okapi::QLength distance = (XCoord - state.x) / sin(state.theta.convert(okapi::radian));
  driveStraightAsync(distance);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::straightYDistance(okapi::QLength YCoord){
  straightYDistanceAsync(YCoord);
  waitUntilSettled();
};
void ChassisControllerHDrive::straightYDistanceAsync(okapi::QLength YCoord){
    okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
    okapi::QLength distance = (YCoord - state.y) / cos(state.theta.convert(okapi::radian));
    driveStraightAsync(distance);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::strafeXDistance(okapi::QLength XCoord){
  strafeXDistanceAsync(XCoord);
  waitUntilSettled();
};
void ChassisControllerHDrive::strafeXDistanceAsync(okapi::QLength XCoord){
  okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
  okapi::QLength distance = (XCoord - state.x) / cos(state.theta.convert(okapi::radian));
  strafeAsync(distance);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::strafeYDistance(okapi::QLength YCoord){
  strafeYDistanceAsync(YCoord);
  waitUntilSettled();
};
void ChassisControllerHDrive::strafeYDistanceAsync(okapi::QLength YCoord){
  okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
  okapi::QLength distance = (YCoord - state.y) / sin(state.theta.convert(okapi::radian));
  strafeAsync(distance);
};

// ----------------------------------------------------------------

void ChassisControllerHDrive::diagToPointAsync(okapi::Point point) {
    okapi::OdomState state = odom->getState(okapi::StateMode::CARTESIAN);
    driveVectorAsync(point.y - state.y, point.x - state.x);
};
void ChassisControllerHDrive::diagToPoint(okapi::Point point) {
    diagToPointAsync(point);
    waitUntilSettled();
};

// ----------------------------------------------------------------

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
    pathfinder_modify_tank(trajectory.get(), length, leftTrajectory.get(), rightTrajectory.get(), scales.wheelTrack.convert(okapi::meter));

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
    turnPID.flipDisable(false);
    reset();

    // Probably not needed
    mode.push_back(ControllerMode::turn);
    //mode.push_back(ControllerMode::pathfinderProfile);

    auto rate = timeUtil.getRate();

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
    EncoderConfig left_config = { (int)leftSide->getPosition(), 360, scales.straight, // Position, Ticks per Rev, Wheel Circumference
        0.8, 0.0, 0.0, 1.0 / plimits.maxVel, 0.0 }; // Kp, Ki, Kd and Kv, Ka

    EncoderConfig right_config = { (int)rightSide->getPosition(), 360, scales.straight, // Position, Ticks per Rev, Wheel Circumference
        0.8, 0.0, 0.0, 1.0 / plimits.maxVel, 0.0 }; // Kp, Ki, Kd and Kv, Ka

    /* This is jank, should be in step, this entire project needs refactoring into
       mutiple files and step states need to be in different functions */
    stop_task();
    while (!left_follower.finished || !right_follower.finished) {
        const auto segDT = path.left.get()[0].dt * okapi::second;
        //const auto linear = path.segments.get()[i].velocity * okapi::mps;
        // NOTE: We probably want this?? I don't know until I try
        double l = (double)straightGearset.internalGearset * pathfinder_follow_encoder(left_config, &left_follower, path.left.get(), pathLength, (int)leftSide->getPosition());
        double r = (double)straightGearset.internalGearset * pathfinder_follow_encoder(right_config, &right_follower, path.right.get(), pathLength, (int)rightSide->getPosition());

        //const auto LinearToRot = (360 * okapi::degree / (scales->wheelDiameter * 1 * okapi::pi)) * straightGearset->ratio;
        //const auto chassisRPM = (linear * LinearToRot).convert(okapi::rpm);
        //const double speed = chassisRPM / toUnderlyingType(straightGearset->internalGearset) * reversed;

        // In degrees
        double heading = (left_follower.heading * okapi::radian).convert(okapi::degree);
        if (heading > 180) {
            heading = (360.0 - heading);
        }

        turnPID.setTarget(heading * scales.turn * straightGearset.ratio);
        double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;
        double turnOut = turnPID.step(turnChange);
        double turnVelocity = (double)straightGearset.internalGearset * turnOut;

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

// ----------------------------------------------------------------

void ChassisControllerHDrive::diagToPointAndTurn(okapi::Point point, okapi::QAngle angle){};
void ChassisControllerHDrive::diagToPointAndTurnAsync(okapi::Point point, okapi::QAngle angle){};
