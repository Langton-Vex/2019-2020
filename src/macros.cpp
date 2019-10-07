#include "main.h"

void Macro::run() {
    if (mutex.take(TIMEOUT_MAX)) {
        func();
        mutex.give();
    }
}
Macro::Macro(MacroFunc f) {
    func = f;
    empty = false;
}

Macro::Macro() {}

pros::controller_digital_e_t button_list[12] = { DIGITAL_L1, DIGITAL_L2, DIGITAL_R1, DIGITAL_R2,
    DIGITAL_UP, DIGITAL_DOWN, DIGITAL_LEFT, DIGITAL_RIGHT,
    DIGITAL_X, DIGITAL_Y, DIGITAL_A, DIGITAL_B };

//L1     ,L2     ,R1     ,R2
Macro macros[12] = { Macro(), Macro(), Macro(), Macro(),
    //UP     ,DOWN   ,LEFT   ,RIGHT
    Macro(), Macro(), Macro(), Macro(),
    //X      ,Y      ,A      ,B
    Macro(), Macro(), Macro(), Macro() };

void macros_update(pros::Controller controller) {
    /*
      for (int i=0;i<12;i++){
      if(controller.get_digital_new_press(button_list[i]) && !macros[i].empty){
        macros[i].run();
      };
    }
    */
}
