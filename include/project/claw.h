
#ifndef _CLAW_H_
#define _CLAW_H_

struct Peripherals_t;
#include "main.h"

class Claw {
public:
    static std::shared_ptr<Claw> get();
    void user_control();
    void set(int power);

    int power = 0;

protected:
    Claw();
    //Peripherals_t peripherals;
};
#endif
