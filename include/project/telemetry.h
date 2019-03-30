#ifndef _TELMETRY_H_
#define _TELMETRY_H_

#include "api.h"
#include <vector>

std::vector<double> get_motor_telemetry(pros::Motor motor);
std::vector<double> get_telemetry();
std::string vector_to_csv(std::vector<double> vector);
FILE* telemetry_init(std::string filename);
void telemetry_update(FILE* telem_file);
#endif
