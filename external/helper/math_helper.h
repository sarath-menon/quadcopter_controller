#pragma once
#include <math.h>

namespace math_helper {

// Constrains input value to a range
constexpr static float limit(float val, const float max, const float min) {
  if (val > max)
    val = max;
  else if (val < min)
    val = min;
  return val;
}

} // namespace math_helper