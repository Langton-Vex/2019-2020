#include "main.h"

// Now achieving triple level commenting, this whole controller needs works, please
// remember to take the robot home over Christmas and work on this garbage fire of
// A chassis controller and turn it into something beautiful with odom and motion profiling

/* TODO:
   * Get tracking wheels if we can.
   * Work on motion profiling (current iteration needs testing before that though)
   * clean up ALL of the code.
     * To add onto this, a lot of this code is repetitive (See the waitUntilSetted methods)
       and some of the code in the step() function is just shocking. All that code can be
       cleaned up and whatever
     * Take inspiration from Vulcan, and initialise this class with a struct containing
       the values you want to initialise with.
     * Smart pointer creation is ugly, probably should just have an initializer list.

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
     okapi::ChassisScales iscales) {

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
     scales = std::make_unique<okapi::ChassisScales>(
         iscales);

     leftSide = std::make_unique<okapi::MotorGroup>(ileftSide);
     rightSide = std::make_unique<okapi::MotorGroup>(irightSide);
     strafeMotor = std::make_unique<okapi::Motor>(istrafe);

     timeUtil = std::make_unique<okapi::TimeUtil>(okapi::TimeUtilFactory::createDefault());
     settledUtil = timeUtil->getSettledUtil();

     model = std::make_unique<okapi::HDriveModel>(leftSide, rightSide, strafeMotor,
         leftSide->getEncoder(), rightSide->getEncoder(), strafeMotor->getEncoder(),
         maxVelocity, maxVoltage);
     odom = std::make_unique<okapi::ThreeEncoderOdometry>(okapi::TimeUtilFactory::createDefault(),
         std::static_pointer_cast<okapi::ReadOnlyChassisModel>(model), iscales);

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

     currentMaxVelocity = 0.0;

     straightPID->reset();
     anglePID->reset();
     turnPID->reset();
     strafePID->reset();
     hypotPID->reset();
 }


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

 void ChassisControllerHDrive::disable_controllers() {
     straightPID->flipDisable(true);
     anglePID->flipDisable(true);
     turnPID->flipDisable(true);
     strafePID->flipDisable(true);
     hypotPID->flipDisable(true);
 };


 void ChassisControllerHDrive::step() {
     odom->step();
     okapi::OdomState state = odom->getState();
     //fprintf(stderr, "odom: %f, %f, %f\n", state.x.convert(okapi::meter), state.y.convert(okapi::meter), state.theta.convert(okapi::degree));

     double distance_forward = ((leftSide->getPosition() - leftSideStart) + (rightSide->getPosition() - rightSideStart)) / 2.0;
     double angleChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart));
     double turnChange = ((leftSide->getPosition() - leftSideStart) - (rightSide->getPosition() - rightSideStart)) / 2.0;
     double strafeChange = strafeMotor->getPosition() - strafeStart;

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
   uint32_t millis = pros::millis();
     while (task_running) {
         this->step();
         pros::Task::delay_until(&millis, asyncUpdateDelay);
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
