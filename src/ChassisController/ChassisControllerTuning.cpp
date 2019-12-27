#include "main.h"


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

    TurnTuner = okapi::PIDTunerFactory::createPtr(
        ct, ct, 8 * okapi::second, 1035,
        0.0005, 0.004, 0, 0, 0, 0.0001);
    TurnTuner->autotune();
    */

    printf("strafe tuning\n");
    tuningMode = TuningMode::TuneStrafe;
    okapi::PIDTuner::Output strafeTune = StrafeTuner->autotune();
    std::string strafeValue = TuningToString(strafeTune);
    printf("strafe value: %s\n", strafeValue.c_str());

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
};
