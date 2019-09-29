#include "main.h"
#include <iostream>
#include <fstream>
#include <iterator>

ConfigManager::ConfigManager(){
  load_config();
};

void ConfigManager::save_config(){
  printf("Saving config");
  std::ofstream save_file(save_filepath,std::ofstream::out | std::ofstream::trunc);
  save_file.clear();
  save_file << selected_auton << "\n";
  save_file << selected_team << "\n";
  save_file.close();
}

void ConfigManager::load_config(){
  bool file_exists;
  if (FILE *file = fopen(save_filepath.c_str(), "r")) {
        fclose(file);
        file_exists = true;
    } else {
        file_exists = false;
    }

  if (file_exists){
    std::ifstream input_file(save_filepath);
    char temp_string[256];

    input_file.getline(temp_string,256);
    selected_auton =  std::stoi(temp_string);

    input_file.getline(temp_string,256);
    selected_team =  std::stoi(temp_string);
    input_file.close();
  }
}

void ConfigManager::register_auton(std::string name, auton_routine routine){
    autonomous_names.push_back(name);
    auton_routines.push_back(routine);

};

void ConfigManager::select_auton(int id){
    selected_auton = id;
    this->save_config();
}
void ConfigManager::select_team(int team){
    selected_team = team;
    this->save_config();
}
