
#ifndef _CLAW_H_
#define _CLAW_H_

struct Peripherals_t;
#include "main.h"

class Claw {
public:
    Claw();
    void user_control();
    void set(int power);

    int power = 0;

protected:
    //Peripherals_t peripherals;
};
#endif
