#include "main.h"

std::shared_ptr<ConfigManager> ConfigManager::get() {
    static std::shared_ptr<ConfigManager> instance(new ConfigManager);
    return instance;
}

ConfigManager::ConfigManager(){};

void ConfigManager::save_config() {
    printf("Saving config");
    std::ofstream save_file(save_filepath, std::ofstream::out | std::ofstream::trunc);
    save_file.clear();
    save_file << selected_auton << "\n";
    save_file << selected_team << "\n";
    save_file.close();
}

void ConfigManager::load_config() {
    FILE* file = fopen(save_filepath.c_str(), "r");
    bool file_exists = file != NULL;
    if (file_exists) {
        fclose(file);
        std::ifstream input_file(save_filepath);
        char temp_string[256];

        input_file.getline(temp_string, 256);
        selected_auton = std::stoi(temp_string);

        input_file.getline(temp_string, 256);
        selected_team = std::stoi(temp_string);

        input_file.close();
    }
}

void ConfigManager::register_auton(std::string name, auton_func func) {
    auto routine = std::make_tuple(name, func, okapi::OdomState());
    auton_routines.push_back(routine);
};
void ConfigManager::register_auton(std::string name, auton_func func, okapi::OdomState state) {
    auto routine = std::make_tuple(name, func, state);
    auton_routines.push_back(routine);
};

auton_func ConfigManager::get_auton_func(int id) {
    return std::get<1>(auton_routines[id]);
}
std::string ConfigManager::get_auton_name(int id) {
    return std::get<0>(auton_routines[id]);
}
okapi::OdomState ConfigManager::get_auton_state(int id) {
    return std::get<2>(auton_routines[id]);
} /* TODO: Little bit of a flaw here, but the pitch isn't symmetrical, so we need
           to check which side we are on, and flip some coordinates, but also
           do this in the autons as well.
*/

void ConfigManager::select_auton(int id) {
    selected_auton = id;
    this->save_config();
}
void ConfigManager::select_team(int team) {
    selected_team = team;

    this->save_config();
}
