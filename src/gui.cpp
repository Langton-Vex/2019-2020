#include "main.h"

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
};

void GUI::build_main(lv_obj_t * parent)
{
    lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY);

    lv_theme_t * th = lv_theme_get_current();

    static lv_style_t h_style;
    lv_style_copy(&h_style, &lv_style_transp);
    h_style.body.padding.inner = LV_DPI / 20;
    h_style.body.padding.hor = LV_DPI / 20;
    h_style.body.padding.ver = LV_DPI / 10;

    lv_obj_t * h = lv_cont_create(parent, NULL);
    lv_obj_set_style(h, &h_style);
    lv_obj_set_click(h, false);
    lv_cont_set_fit(h, true,true);
    //lv_cont_set_layout(h, LV_LAYOUT_PRETTY);

    // Begin main element building

    lv_obj_t * auton_select = lv_roller_create(h, NULL);
    lv_obj_set_size(auton_select, HOR_RES/4, VER_RES / 2);
    //lv_obj_set_protect(auton_select, LV_PROTECT_POS);
    lv_obj_align(auton_select, NULL, LV_ALIGN_OUT_RIGHT_MID,HOR_RES/2,0);
    lv_roller_set_options(auton_select,
      "Example Auton\n Example 2\n Example 3");
    lv_roller_set_selected(auton_select, 1, false);
    lv_roller_set_visible_row_count(auton_select, 3);

    lv_obj_t * auton_select_label = lv_label_create(h, NULL);
    lv_label_set_text(auton_select_label, "Autonomous");
    lv_obj_align(auton_select_label, auton_select, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    // side switch

    static lv_style_t side_sw_on_style;
    static lv_style_t side_sw_off_style;
    static lv_style_t side_sw_bg_style;
    static lv_style_t side_sw_indic_style;

    lv_style_copy(&side_sw_on_style, &lv_style_pretty_color);
    side_sw_on_style.body.radius = LV_RADIUS_CIRCLE;

    lv_style_copy(&side_sw_off_style, &lv_style_pretty);
    side_sw_off_style.body.radius = LV_RADIUS_CIRCLE;

    lv_style_copy(&side_sw_indic_style, &lv_style_pretty_color);
    lv_style_copy(&side_sw_bg_style, &lv_style_pretty);

    side_sw_on_style.body.main_color = LV_COLOR_HEX(0xff0000);
    side_sw_on_style.body.grad_color = LV_COLOR_HEX(0xff0000);
    side_sw_off_style.body.main_color = LV_COLOR_HEX(0x0000ff);
    side_sw_off_style.body.grad_color = LV_COLOR_HEX(0x0000ff);


    //side_sw_bg_style.body.main_color= LV_COLOR_HEX(0x424247);

    side_sw_indic_style.body.radius = LV_RADIUS_CIRCLE;
    side_sw_bg_style.body.radius = LV_RADIUS_CIRCLE;

    //side_sw_indic_style.body.main_color = LV_COLOR_HEX(0xff0000);
    //side_sw_bg_style.body.main_color = LV_COLOR_HEX(0xff0000);
    //side_sw_indic_style.body.grad_color = LV_COLOR_HEX(0xff0000);
    side_sw_indic_style.body.main_color = LV_COLOR_HEX(0x9fc8ef);
    side_sw_indic_style.body.grad_color = LV_COLOR_HEX(0x9fc8ef);
    side_sw_indic_style.body.padding.hor = LV_DPI/5;
    side_sw_indic_style.body.padding.ver = 0;

    lv_obj_t *side = lv_sw_create(h, NULL);
    lv_obj_align(side, auton_select, LV_ALIGN_OUT_LEFT_MID, -50, 0);

    lv_sw_set_style(side, LV_SW_STYLE_KNOB_ON, &side_sw_on_style);
    lv_sw_set_style(side, LV_SW_STYLE_KNOB_OFF, &side_sw_off_style);
    lv_sw_set_style(side, LV_SW_STYLE_BG, &side_sw_bg_style);
    lv_sw_set_style(side, LV_SW_STYLE_INDIC, &side_sw_indic_style);

    lv_obj_t * side_label = lv_label_create(h, NULL);
    lv_label_set_long_mode(side_label, LV_LABEL_LONG_EXPAND);
    lv_label_set_text(side_label, "Team\ncolour");
    lv_obj_align(side_label, side, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);



}
