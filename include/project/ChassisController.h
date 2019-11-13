#ifndef _CHASSIS_CONTROLLER_H_
#define _CHASSIS_CONTROLLER_H_

#include "main.h"

/* Yes everything uses okapi it's a great library dont judge
   There is nothing wrong with not wanting to reinvent the wheel
   like who even wants to create their own PID controller when a good one
   already exists
*/

class ChassisControllerHDrive : public okapi::ControllerOutput<double>,
                                public okapi::ControllerInput<double> {
public:
    ChassisControllerHDrive(
        PIDTuning straightTuning, PIDTuning angleTuning,
        PIDTuning turnTuning, PIDTuning strafeTuning,
        PIDTuning hypotTuning, okapi::MotorGroup leftSide,
        okapi::MotorGroup rightSide, okapi::Motor strafe,
        okapi::AbstractMotor::GearsetRatioPair istraightGearset,
        okapi::AbstractMotor::GearsetRatioPair istrafeGearset,
        okapi::ChassisScales iscales);
    ~ChassisControllerHDrive();
    /* TODO: Make this controller use okapi's models
                           For cross robot compatability*/

    okapi::QAcceleration max_acceleration; // TODO: implement
    std::vector<ControllerMode> mode;
    TuningMode tuningMode;
    std::unique_ptr<pros::Task> task;
    int asyncUpdateDelay = 20;

    std::unique_ptr<okapi::Odometry> odom;
    std::shared_ptr<okapi::SkidSteerModel> model;

    int maxVelocity = 200;
    double maxVoltage = 12.0;

    static void trampoline(void* param);

    void asyncThread();
    bool task_running = false;

    // I don't like weird constructors OK

    std::unique_ptr<okapi::IterativePosPIDController> straightPID;
    std::unique_ptr<okapi::IterativePosPIDController> anglePID;
    std::unique_ptr<okapi::IterativePosPIDController> turnPID;
    std::unique_ptr<okapi::IterativePosPIDController> strafePID;
    std::unique_ptr<okapi::IterativePosPIDController> hypotPID;

    std::unique_ptr<okapi::AbstractMotor::GearsetRatioPair> straightGearset;
    std::unique_ptr<okapi::AbstractMotor::GearsetRatioPair> strafeGearset;
    std::unique_ptr<okapi::ChassisScales> scales;

    std::shared_ptr<okapi::AbstractMotor> leftSide;
    std::shared_ptr<okapi::AbstractMotor> rightSide;
    std::shared_ptr<okapi::Motor> strafeMotor;

    double leftSideStart, rightSideStart, strafeStart;

    void driveStraight(okapi::QLength distance);
    void driveStraightAsync(okapi::QLength distance);

    void turnAngle(okapi::QAngle angle);
    void turnAngleAsync(okapi::QAngle angle);

    void strafe(okapi::QLength distance);
    void strafeAsync(okapi::QLength distance);

    void driveVector(okapi::QLength straight, okapi::QLength strafe);
    void driveVectorAsync(okapi::QLength straight, okapi::QLength strafe);

    void diagToPointAsync(okapi::Point point);
    void diagToPoint(okapi::Point point);

    void diagToPointAndTurn(okapi::Point point, okapi::QAngle angle);
    void diagToPointAndTurnAsync(okapi::Point point, okapi::QAngle angle);

    void waitUntilSettled();
    bool waitUntilDistanceSettled();
    bool waitUntilTurnSettled();
    bool waitUntilStrafeSettled();

    void waitUntilTravelled(okapi::QLength distance);

    void disable_controllers();
    void reset();
    void setMaxVelocity(int speed);
    void tune();

    void controllerSet(double ivalue);
    double controllerGet();

    void step();
    void start_task();
    void stop_task();

private:
    std::unique_ptr<okapi::TimeUtil> timeUtil;
    std::unique_ptr<okapi::SettledUtil> settledUtil;
};

#endif
