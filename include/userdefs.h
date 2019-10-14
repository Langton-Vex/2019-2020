#ifndef _USERDEFS_H_
#define _USERDEFS_H_
#include "api.h"
// User definitions

float sgn(int x);

#include "project/peripherals_t.h"

#include "project/macros.h"

extern Peripherals_t peripherals;

class Chassis;
//class Arm;
//class Claw;
class GUI;

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
