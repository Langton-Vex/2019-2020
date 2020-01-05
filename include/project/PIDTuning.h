#ifndef _PID_TUNING_H_
#define _PID_TUNING_H_
#include <initializer_list>

// So I don't have to pass 15 integers to the ChassisController lol
struct PIDTuning {
    PIDTuning(double ikP, double ikI, double ikD);
    PIDTuning(std::initializer_list<double>);
    double kP, kI, kD;
};

#endif
