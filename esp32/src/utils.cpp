#include "utils.h"

int Utils::clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int Utils::map100To255(int v) {
  v = clampInt(v, -100, 100);
  // Scale -100..100 to roughly -255..255
  return (v * 255) / 100;
}