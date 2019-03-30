#include "main.h"

std::vector<double> get_motor_telemetry(pros::Motor motor){
    std::vector<double> telemetry;
    telemetry.push_back((double) motor.get_actual_velocity());
    telemetry.push_back((double) motor.get_current_draw());
    telemetry.push_back((double) motor.get_direction());
    telemetry.push_back((double) motor.get_efficiency());
    telemetry.push_back((double) motor.get_faults());
    telemetry.push_back((double) motor.get_position());
    telemetry.push_back((double) motor.get_power());
    telemetry.push_back((double) motor.get_temperature());
    telemetry.push_back((double) motor.get_torque());
    telemetry.push_back((double) motor.get_voltage());
    return telemetry;
};

std::vector<double> get_telemetry(){
    std::vector<double> telemetry;
    for (int i=0;i<peripherals.motor_list.size();i++){
        std::vector<double> motor_telemetry = get_motor_telemetry(*peripherals.motor_list[i]);
        telemetry.reserve(motor_telemetry.size() + distance(motor_telemetry.begin(),motor_telemetry.end()));
        telemetry.insert(telemetry.end(),motor_telemetry.begin(),motor_telemetry.end());
    }

    telemetry.push_back(pros::battery::get_capacity());
    telemetry.push_back(pros::battery::get_current());
    telemetry.push_back(pros::battery::get_temperature());
    telemetry.push_back(pros::battery::get_voltage());

    telemetry.push_back(pros::competition::get_status());

    telemetry.push_back(peripherals.master_controller.get_battery_capacity());
    telemetry.push_back(peripherals.master_controller.get_battery_level());

    return telemetry;
};

std::string vector_to_csv(std::vector<double> vector){
    std::string csv_string;
    for (auto i = vector.begin();i!=vector.end();i++){
        csv_string.append(std::to_string(*i));
        csv_string.append(",");
    }
    csv_string.append("\n");

    return csv_string;
}

FILE* telemetry_init(std::string filename){
  FILE* telem_file = fopen(filename.c_str(), "a");
  fputs("\n\n", telem_file);
  fflush(telem_file);
  return telem_file;
}

void telemetry_update(FILE* telem_file){
  std::string telem_string = vector_to_csv(get_telemetry());
  fputs(telem_string.c_str(),telem_file);
  fflush(telem_file);
}
