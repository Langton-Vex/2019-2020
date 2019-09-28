#ifndef _GUI_H_
#define _GUI_H_

#include "main.h"

class GUI{
public:
  void init_draw();
  static void create_tab1(lv_obj_t * parent);
protected:
  int state;
  lv_theme_t * th = lv_theme_night_init(210, NULL);
};

#endif
