#ifndef _USERDEFS_H_
#define _USERDEFS_H_
#include "api.h"
// User definitions

float sgn(int x);
bool compare_vision_objects(pros::vision_object const lhs, pros::vision_object const rhs);

#include "project/peripherals_t.h"

#include "project/macros.h"

#include <memory>

extern std::unique_ptr<Peripherals_t> peripherals;


#include "project/ControllerMode.h"
#include "project/TuningMode.h"
#include "project/PIDTuning.h"
#include "project/ChassisController.h"
#include "project/gui.h"
#include "project/chassis.h"
#include "project/arm.h"
#include "project/claw.h"
#include "project/config_manager.h"


// can use smart pointers later, but meh

//extern Chassis chassis;

#endif
