#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include "main.h"

typedef void (*auton_routine)();

class ConfigManager{
public:
  std::vector<std::string> autonomous_names;
  std::vector<auton_routine> auton_routines;

  int selected_auton = 0; // Keep it safe from nullptr hopefully
  int selected_team = 1; // Turns negative on red side

  ConfigManager();
  void register_auton(std::string name, auton_routine routine);
  void select_auton(int id);
  void select_team(int team);

  void save_config();
  void load_config();

protected:
  std::string save_filepath = "/usd/comp_config.cfg";
};

#endif
