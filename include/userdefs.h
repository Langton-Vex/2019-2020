
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
#include "project/chassis.h"
#include "project/arm.h"
#include "project/claw.h"

// can use smart pointers later, but meh

//extern Chassis chassis;

#endif  // _USERDEFS_H_
