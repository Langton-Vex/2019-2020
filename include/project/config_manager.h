#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include "main.h"

typedef void (*auton_func)();
typedef std::tuple<std::string, auton_func, okapi::OdomState> auton_routine;

class ConfigManager {
public:
    static std::shared_ptr<ConfigManager> get();

    std::vector<auton_routine> auton_routines;

    int selected_auton = 0; // Keep it safe from nullptr hopefully
    int selected_team = 1; // Turns negative on red side

    void register_auton(std::string name, auton_func func);
    void register_auton(std::string name, auton_func func, okapi::OdomState state);
    void select_auton(int id);
    void select_team(int team);

    auton_func get_auton_func(int id);
    std::string get_auton_name(int id);
    okapi::OdomState get_auton_state(int id);

    void save_config();
    void load_config();

protected:
    ConfigManager();

    std::string save_filepath = "/usd/comp_config.cfg";
};

#endif
