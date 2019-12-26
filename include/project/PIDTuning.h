#ifndef _PID_TUNING_H_
#define _PID_TUNING_H_

// So I don't have to pass 15 integers to the ChassisController lol
struct PIDTuning {
    PIDTuning(double ikP, double ikI, double ikD);
    double kP, kI, kD;
};

#endif
