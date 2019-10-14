#ifndef _CHASSIS_CONTROLLER_H_
#define _CHASSIS_CONTROLLER_H_

#include "main.h"

/* Yes everything uses okapi it's a great library dont judge
   There is nothing wrong with not wanting to reinvent the wheel
   like who even wants to create their own PID controller when a good one
   already exists
*/

class ChassisController {
public:
    ChassisController(
        PIDTuning straightTuning, PIDTuning angleTuning,
        PIDTuning turnTuning, PIDTuning strafeTuning,
        PIDTuning hypotTuning, okapi::MotorGroup leftSide,
        okapi::MotorGroup rightSide, okapi::Motor strafe);
    ~ChassisController(); /* TODO: Make this controller use okapi's models
                           For cross robot compatability*/

    okapi::QAcceleration max_acceleration; // TODO: implement

    // I don't like weird constructors OK

    std::unique_ptr<okapi::IterativePosPIDController> straightPID;
    std::unique_ptr<okapi::IterativePosPIDController> anglePID;
    std::unique_ptr<okapi::IterativePosPIDController> turnPID;
    std::unique_ptr<okapi::IterativePosPIDController> strafePID;
    std::unique_ptr<okapi::IterativePosPIDController> hypotPID;

    std::unique_ptr<okapi::AbstractMotor::GearsetRatioPair> straightGearset;
    std::unique_ptr<okapi::AbstractMotor::GearsetRatioPair> strafeGearset;
    std::unique_ptr<okapi::ChassisScales> scales;

    std::unique_ptr<okapi::MotorGroup> leftSide;
    std::unique_ptr<okapi::MotorGroup> rightSide;
    std::unique_ptr<okapi::Motor> strafeMotor;

    void driveStraight(okapi::QLength distance);
    void driveStraightAsync(okapi::QLength distance);

    void turnAngle(okapi::QAngle angle);
    void turnAngleAsync(okapi::QAngle angle);

    void strafe(okapi::QLength distance);
    void strafeAsync(okapi::QLength distance);

    void driveVector(okapi::QLength straight, okapi::QLength strafe);
    void driveVectorAsync(okapi::QLength straight, okapi::QLength strafe);

    void driveVectorAndTurn(okapi::QLength straight, okapi::QLength strafe,
        okapi::QAngle angle);
    void driveVectorAndTurnAsync(okapi::QLength straight, okapi::QLength strafe,
        okapi::QAngle angle);

    void waitUntilSettled();
    void waitUntilTravelled(okapi::QLength distance);

    void disable_controllers();

    void step();
    void start_task();
    void stop_task();

private:
    std::unique_ptr<okapi::TimeUtil> timeUtil;
};

#endif
