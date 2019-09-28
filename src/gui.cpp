#include "main.h"

const auto HOR_RES = 480;
const auto VER_RES = 240;

void GUI::init_draw(){
    lv_theme_set_current(th);
    th = lv_theme_get_current();    /*If `LV_THEME_LIVE_UPDATE  1` `th` is not used directly so get the real theme after set*/
    lv_obj_t * scr = lv_scr_act();
    lv_obj_t * tv = lv_tabview_create(scr, NULL);

    lv_obj_set_size(tv, HOR_RES, VER_RES);

    lv_obj_t * tab1 = lv_tabview_add_tab(tv, "Tab 1");
    lv_obj_t * tab2 = lv_tabview_add_tab(tv, "Tab 2");
    lv_obj_t * tab3 = lv_tabview_add_tab(tv, "Tab 3");

    //create_tab1(tab1);
};

void GUI::create_tab1(lv_obj_t * parent)
{
    lv_page_set_scrl_layout(parent, LV_LAYOUT_PRETTY);

    lv_theme_t * th = lv_theme_get_current();

    static lv_style_t h_style;
    lv_style_copy(&h_style, &lv_style_transp);
    h_style.body.padding.inner = LV_DPI / 10;
    h_style.body.padding.hor = LV_DPI / 4;
    h_style.body.padding.ver = LV_DPI / 10;

    lv_obj_t * h = lv_cont_create(parent, NULL);
    lv_obj_set_style(h, &h_style);
    lv_obj_set_click(h, false);
    lv_cont_set_fit(h, true,true);
    lv_cont_set_layout(h, LV_LAYOUT_COL_M);

    lv_obj_t * btn = lv_btn_create(h, NULL);
    lv_btn_set_fit(btn, true,true);
    //lv_btn_set_toggle(btn, true);
    lv_obj_t * btn_label = lv_label_create(btn, NULL);
    lv_label_set_text(btn_label, "Button");

    btn = lv_btn_create(h, btn);
    lv_btn_toggle(btn);
    btn_label = lv_label_create(btn, NULL);
    lv_label_set_text(btn_label, "Toggled");

    btn = lv_btn_create(h, btn);
    lv_btn_set_state(btn, LV_BTN_STATE_INA);
    btn_label = lv_label_create(btn, NULL);
    lv_label_set_text(btn_label, "Inactive");

    lv_obj_t * label = lv_label_create(h, NULL);
    lv_label_set_text(label, "Primary");
    //lv_obj_set_style(label, th->h_style.label.prim);

    label = lv_label_create(h, NULL);
    lv_label_set_text(label, "Secondary");
    //lv_obj_set_style(label, th->style.label.sec);

    label = lv_label_create(h, NULL);
    lv_label_set_text(label, "Hint");
    //lv_obj_set_style(label, th->style.label.hint);

    //static const char * btnm_str[] = {"1", "2", "3", LV_SYMBOL_OK, LV_SYMBOL_CLOSE, ""};
    lv_obj_t * btnm = lv_btnm_create(h, NULL);
    lv_obj_set_size(btnm, 480 / 4, 2 * LV_DPI / 3);
    //lv_btnm_set_map(btnm, btnm_str);
    //lv_btnm_set_btn_ctrl_all(btnm, LV_BTNM_CTRL_TGL_ENABLE);
    //lv_btn_set_toggle(btnm, true);

    lv_obj_t * table = lv_table_create(h, NULL);
    lv_table_set_col_cnt(table, 3);
    lv_table_set_row_cnt(table, 4);
    lv_table_set_col_width(table, 0, LV_DPI / 3);
    lv_table_set_col_width(table, 1, LV_DPI / 2);
    lv_table_set_col_width(table, 2, LV_DPI / 2);
    lv_table_set_cell_merge_right(table, 0, 0, true);
    lv_table_set_cell_merge_right(table, 0, 1, true);

    lv_table_set_cell_value(table, 0, 0, "Table");
    lv_table_set_cell_align(table, 0, 0, LV_LABEL_ALIGN_CENTER);

    lv_table_set_cell_value(table, 1, 0, "1");
    lv_table_set_cell_value(table, 1, 1, "13");
    lv_table_set_cell_align(table, 1, 1, LV_LABEL_ALIGN_RIGHT);
    lv_table_set_cell_value(table, 1, 2, "ms");

    lv_table_set_cell_value(table, 2, 0, "2");
    lv_table_set_cell_value(table, 2, 1, "46");
    lv_table_set_cell_align(table, 2, 1, LV_LABEL_ALIGN_RIGHT);
    lv_table_set_cell_value(table, 2, 2, "ms");

    lv_table_set_cell_value(table, 3, 0, "3");
    lv_table_set_cell_value(table, 3, 1, "61");
    lv_table_set_cell_align(table, 3, 1, LV_LABEL_ALIGN_RIGHT);
    lv_table_set_cell_value(table, 3, 2, "ms");

    h = lv_cont_create(parent, h);

    lv_obj_t * sw_h = lv_cont_create(h, NULL);
    //lv_cont_set_style(sw_h, NULL, &lv_style_transp);
    lv_cont_set_fit(sw_h, false, true);
    lv_obj_set_width(sw_h, LV_HOR_RES / 4);
    lv_cont_set_layout(sw_h, LV_LAYOUT_PRETTY);

    lv_obj_t * sw = lv_sw_create(sw_h, NULL);
    lv_sw_set_anim_time(sw, 250);

    sw = lv_sw_create(sw_h, sw);
    lv_sw_on(sw);


    lv_obj_t * bar = lv_bar_create(h, NULL);
    lv_bar_set_value(bar, 70);

    lv_obj_t * slider = lv_slider_create(h, NULL);
    lv_bar_set_value(slider, 70);

    lv_obj_t * line = lv_line_create(h, NULL);
    static lv_point_t line_p[2];
    line_p[0].x = 0;
    line_p[0].y = 0;
    line_p[1].x = 480 / 5;
    line_p[1].y = 0;

    lv_line_set_points(line, line_p, 2);
    //lv_line_set_style(line, LV_LINE_STYLE_MAIN, th->style.line.decor);

    lv_obj_t * cb = lv_cb_create(h, NULL);

    cb = lv_cb_create(h, cb);
    lv_btn_set_state(cb, LV_BTN_STATE_TGL_REL);

    lv_obj_t * ddlist = lv_ddlist_create(h, NULL);

    //lv_ddlist_set_fix_width(ddlist, lv_obj_get_width(ddlist) + LV_DPI / 2);   //Make space for the arrow
    lv_ddlist_set_draw_arrow(ddlist, true);

    h = lv_cont_create(parent, h);

    lv_obj_t * list = lv_list_create(h, NULL);
    lv_obj_set_size(list, HOR_RES / 4, VER_RES / 2);
    lv_obj_t * list_btn;
    //list_btn = lv_list_add_btn(list, LV_SYMBOL_GPS,  "GPS");
    //lv_btn_set_toggle(list_btn, true);
    /*
    lv_list_add_btn(list, LV_SYMBOL_WIFI, "WiFi");
    lv_list_add_btn(list, LV_SYMBOL_GPS, "GPS");
    lv_list_add_btn(list, LV_SYMBOL_AUDIO, "Audio");
    lv_list_add_btn(list, LV_SYMBOL_VIDEO, "Video");
    lv_list_add_btn(list, LV_SYMBOL_CALL, "Call");
    lv_list_add_btn(list, LV_SYMBOL_BELL, "Bell");
    lv_list_add_btn(list, LV_SYMBOL_FILE, "File");
    lv_list_add_btn(list, LV_SYMBOL_EDIT, "Edit");
    lv_list_add_btn(list, LV_SYMBOL_CUT,  "Cut");
    lv_list_add_btn(list, LV_SYMBOL_COPY, "Copy");
    */
    //lv_obj_t * roller = lv_roller_create(h, NULL);
    //lv_roller_set_options(roller, "Monday\nTuesday\nWednesday\nThursday\nFriday\nSaturday\nSunday", true);
    //lv_roller_set_selected(roller, 1, false);
    //lv_roller_set_visible_row_count(roller, 3);



}
