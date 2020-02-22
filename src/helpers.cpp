#include "main.h"

float sgn(int x) {
    if (x > 0)
        return 1.0;
    if (x < 0)
        return -1.0;
    return 0.0;
}

bool compare_vision_objects(pros::vision_object const lhs, pros::vision_object const rhs){
  if (lhs.signature == rhs.signature &&
    lhs.type == rhs.type &&
    lhs.left_coord == rhs.left_coord &&
    lhs.top_coord == rhs.top_coord &&
    lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.angle == rhs.angle){
      return true;
    }
  return false;
}
