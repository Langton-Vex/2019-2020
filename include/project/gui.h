#ifndef _GUI_H_
#define _GUI_H_

#include "main.h"

class GUI {
public:
    static std::shared_ptr<GUI> get();

    void gui_build();
    static void build_main(lv_obj_t* parent);
    void build_diagnostics(lv_obj_t* parent);
    static void build_control(lv_obj_t* parent);
    void build_console(lv_obj_t* parent);

    lv_obj_t* console_box;
    lv_obj_t* arm_temp_guage;
    lv_obj_t* chassis_temp_guage;
    lv_obj_t* claw_temp_guage;

protected:
    GUI();
    int state;
    lv_theme_t* th = lv_theme_alien_init(210, &lv_font_dejavu_20);

    static lv_res_t cb_auton_select(lv_obj_t* auton_select);
    static lv_res_t cb_side(lv_obj_t* side);
};

#endif
