#ifndef _TUNING_MODE_H_
#define _TUNING_MODE_H_

// So I don't have to pass 15 integers to the ChassisController lol
enum class TuningMode {

    TuneStraight,
    TuneAngle,
    TuneTurn,
    TuneStrafe,
    TuneHypot,
};

#endif
