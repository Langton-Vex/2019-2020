#ifndef _MACROS_H_
#define _MACROS_H_

#include "api.h"

typedef void (*MacroFunc) (void);

void macros_update(pros::Controller controller);

struct Macro{
  Macro();
  Macro(MacroFunc f);
  MacroFunc func;
  pros::Mutex mutex;
  bool empty = true;
  void run();
};
#endif
