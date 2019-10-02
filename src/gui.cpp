#include "main.h"

extern ConfigManager configManager;

const auto HOR_RES = 480;
const auto VER_RES = 240;

void GUI::gui_build(){
    lv_theme_set_current(th);
    th = lv_theme_get_current();    /*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/
    lv_obj_t * scr = lv_scr_act();
    lv_obj_t * tv = lv_tabview_create(scr, NULL);
    lv_tabview_set_sliding(tv, false); // Sliding is laggy RIP

    lv_obj_set_size(tv, HOR_RES, VER_RES);

    lv_obj_t * main = lv_tabview_add_tab(tv, "Main");
    lv_obj_t * diagnostics = lv_tabview_add_tab(tv, "Diagnostics");
    lv_obj_t * control = lv_tabview_add_tab(tv, "Control");
    lv_obj_t * console = lv_tabview_add_tab(tv, "Console");


    build_main(main);
    build_console(console);
    build_diagnostics(diagnostics);
};

void GUI::build_main(lv_obj_t * parent)
{
    lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY);

    lv_theme_t * th = lv_theme_get_current();

    static lv_style_t h_style;
    lv_style_copy(&h_style, &lv_style_transp);
    h_style.body.padding.inner = LV_DPI / 20;
    h_style.body.padding.hor = LV_DPI / 20;
    h_style.body.padding.ver = LV_DPI / 20;

    lv_obj_t * h = lv_cont_create(parent, NULL);
    lv_obj_set_style(h, &h_style);
    lv_obj_set_click(h, false);
    lv_cont_set_fit(h, true,true);
    //lv_cont_set_layout(h, LV_LAYOUT_PRETTY);

    // Begin main element building

    lv_obj_t * auton_select = lv_roller_create(h, NULL);
    lv_roller_set_action(auton_select, cb_auton_select);

    lv_obj_set_size(auton_select, HOR_RES/4, VER_RES / 4);
    //lv_obj_set_protect(auton_select, LV_PROTECT_POS);
    lv_obj_align(auton_select, NULL, LV_ALIGN_OUT_RIGHT_MID,HOR_RES/2,0);

    std::string routines_str;
    for(int i = 0; i < configManager.autonomous_names.size();i++) {
        if(i>0)
          routines_str.append("\n"+configManager.autonomous_names[i]);
        else
          routines_str.append(configManager.autonomous_names[i]);
    }
    lv_roller_set_options(auton_select,routines_str.c_str());

    lv_roller_set_selected(auton_select, configManager.selected_auton, false);
    lv_roller_set_visible_row_count(auton_select, 3);

    lv_obj_t * auton_select_label = lv_label_create(h, NULL);
    lv_label_set_text(auton_select_label, "Autonomous");
    lv_obj_align(auton_select_label, auton_select, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    // side switch

    static lv_style_t side_sw_on_style;
    static lv_style_t side_sw_off_style;
    static lv_style_t side_sw_bg_style;
    //static lv_style_t side_sw_indic_style;

    lv_style_copy(&side_sw_on_style, &lv_style_pretty_color);
    side_sw_on_style.body.radius = LV_RADIUS_CIRCLE;

    lv_style_copy(&side_sw_off_style, &lv_style_pretty);
    side_sw_off_style.body.radius = LV_RADIUS_CIRCLE;

    //lv_style_copy(&side_sw_indic_style, &lv_style_pretty_color);
    lv_style_copy(&side_sw_bg_style, &lv_style_pretty);

    side_sw_on_style.body.main_color = LV_COLOR_HEX(0xff0000);
    side_sw_on_style.body.grad_color = LV_COLOR_HEX(0xff0000);
    side_sw_off_style.body.main_color = LV_COLOR_HEX(0x0000ff);
    side_sw_off_style.body.grad_color = LV_COLOR_HEX(0x0000ff);


    //side_sw_bg_style.body.main_color= LV_COLOR_HEX(0x424247);

    //side_sw_indic_style.body.radius = LV_RADIUS_CIRCLE;
    side_sw_bg_style.body.radius = LV_RADIUS_CIRCLE;

    //side_sw_indic_style.body.main_color = LV_COLOR_HEX(0xff0000);
    //side_sw_bg_style.body.main_color = LV_COLOR_HEX(0xff0000);
    //side_sw_indic_style.body.grad_color = LV_COLOR_HEX(0xff0000);
    /*
    side_sw_indic_style.body.main_color = LV_COLOR_HEX(0xffffff);
    side_sw_indic_style.body.grad_color = LV_COLOR_HEX(0xffffff);
    side_sw_indic_style.body.padding.hor = LV_DPI/5;
    side_sw_indic_style.body.padding.ver = 0;
    */
    lv_obj_t *side = lv_sw_create(h, NULL);
    lv_sw_set_action(side, cb_side);

    if(configManager.selected_team == 1) lv_sw_off(side);
    else lv_sw_on(side);
    lv_obj_align(side, auton_select, LV_ALIGN_OUT_LEFT_MID, -50, 0);

    lv_sw_set_style(side, LV_SW_STYLE_KNOB_ON, &side_sw_on_style);
    lv_sw_set_style(side, LV_SW_STYLE_KNOB_OFF, &side_sw_off_style);
    lv_sw_set_style(side, LV_SW_STYLE_BG, &side_sw_bg_style);
    lv_sw_set_style(side, LV_SW_STYLE_INDIC, &lv_style_transp);

    lv_obj_t * side_label = lv_label_create(h, NULL);
    lv_label_set_long_mode(side_label, LV_LABEL_LONG_EXPAND);
    lv_label_set_text(side_label, "Team\ncolour");
    lv_obj_align(side_label, side, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

}

void GUI::build_console(lv_obj_t * parent){
  lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY);

  lv_theme_t * th = lv_theme_get_current();

  static lv_style_t h_style;
  lv_style_copy(&h_style, &lv_style_transp);
  h_style.body.padding.inner = LV_DPI / 20;
  h_style.body.padding.hor = LV_DPI / 20;
  h_style.body.padding.ver = LV_DPI / 20;

  lv_obj_t * h = lv_cont_create(parent, NULL);
  lv_obj_set_style(h, &h_style);
  lv_obj_set_click(h, false);
  lv_cont_set_fit(h, true,true);

  static lv_style_t style_console;
  lv_style_copy(&style_console, &lv_style_plain);
  /*
  style_console.body.main_color = LV_COLOR_BLACK;
  style_console.body.grad_color = LV_COLOR_BLACK;
  style_console.body.border.color = LV_COLOR_WHITE;
  style_console.body.border.width = 1;
  style_console.body.border.opa = LV_OPA_70;
  style_console.body.radius = LV_RADIUS_CIRCLE;
  style_console.body.opa = LV_OPA_60;
  */
  console_box = lv_ta_create(h, NULL);
  lv_obj_set_size(console_box, 400,125);
  lv_obj_align(console_box, NULL, LV_ALIGN_CENTER, 0, - LV_DPI / 2);
  lv_ta_set_style(console_box,LV_TA_STYLE_SB, &style_console);                     /*Apply the scroll bar style*/
  lv_ta_set_cursor_type(console_box, LV_CURSOR_NONE);
  lv_ta_set_text(console_box, "Initializing hackerman console:\n");

}

void GUI::build_diagnostics(lv_obj_t * parent){
  lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY);

  lv_theme_t * th = lv_theme_get_current();

  static lv_style_t h_style;
  lv_style_copy(&h_style, &lv_style_transp);
  h_style.body.padding.inner = LV_DPI / 20;
  h_style.body.padding.hor = LV_DPI / 20;
  h_style.body.padding.ver = LV_DPI / 20;

  lv_obj_t * h = lv_cont_create(parent, NULL);
  lv_obj_set_style(h, &h_style);
  lv_obj_set_click(h, false);
  lv_cont_set_fit(h, true,true);

  static lv_style_t style_table;
  lv_style_copy(&style_table, &lv_style_plain);

  static lv_style_t guage_style;
  lv_style_copy(&guage_style, &lv_style_pretty_color);
  guage_style.body.main_color = LV_COLOR_HEX3(0xFFF);     /*Line color at the beginning*/
  guage_style.body.grad_color =  LV_COLOR_HEX3(0xFFF);    /*Line color at the end*/
  guage_style.body.padding.hor = 10;                      /*Scale line length*/
  guage_style.body.padding.inner = 8 ;                    /*Scale label padding*/
  guage_style.body.border.color = LV_COLOR_HEX3(0x333);   /*Needle middle circle color*/
  guage_style.line.width = 3;
  guage_style.text.color = LV_COLOR_HEX3(0xFFF);
  guage_style.line.color = LV_COLOR_RED;                  /*Line color after the critical value*/
  guage_style.text.font = &lv_font_dejavu_10;

  arm_temp_guage = lv_gauge_create(h,NULL);
  lv_obj_set_size(arm_temp_guage, 125,125);
  lv_obj_align(arm_temp_guage, NULL, LV_ALIGN_OUT_LEFT_MID, 0,0);
  lv_gauge_set_style(arm_temp_guage, &style_table);
  lv_gauge_set_range(arm_temp_guage, 0, 70);
  lv_gauge_set_critical_value(arm_temp_guage, 50);
  lv_gauge_set_style(arm_temp_guage, &guage_style);

  chassis_temp_guage = lv_gauge_create(h,NULL);
  lv_obj_set_size(chassis_temp_guage, 125,125);
  lv_obj_align(chassis_temp_guage, arm_temp_guage, LV_ALIGN_OUT_RIGHT_MID, 20,0);
  lv_gauge_set_style(chassis_temp_guage, &style_table);
  lv_gauge_set_range(chassis_temp_guage, 0, 70);
  lv_gauge_set_critical_value(chassis_temp_guage, 50);
  lv_gauge_set_style(chassis_temp_guage, &guage_style);

  claw_temp_guage = lv_gauge_create(h,NULL);
  lv_obj_set_size(claw_temp_guage, 125,125);
  lv_obj_align(claw_temp_guage, chassis_temp_guage, LV_ALIGN_OUT_RIGHT_MID, 20,0);
  lv_gauge_set_style(claw_temp_guage, &style_table);
  lv_gauge_set_range(claw_temp_guage, 0, 70);
  lv_gauge_set_critical_value(claw_temp_guage, 50);
  lv_gauge_set_style(claw_temp_guage, &guage_style);

}

lv_res_t GUI::cb_auton_select(lv_obj_t * auton_select){
  configManager.select_auton(lv_roller_get_selected(auton_select));

  return LV_RES_OK; /*Return OK if the drop down list is not deleted*/
}

lv_res_t GUI::cb_side(lv_obj_t * side){
  bool switch_state = lv_sw_get_state(side); // false is blue, true is red
  if (switch_state)
    configManager.select_team(-1);
  else
    configManager.select_team(1);

  return LV_RES_OK; /*Return OK if the drop down list is not deleted*/
}
