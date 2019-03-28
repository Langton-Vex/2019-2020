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
