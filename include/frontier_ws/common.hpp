#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace fe {

static constexpr int UNKNOWN = -1;

struct GridPose { int x{0}, y{0}; };
struct WorldPose { double x{0}, y{0}, yaw{0}; };

inline int IDX(int x, int y, int w) { return y * w + x; }

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

inline double normAngle(double a) {
  while (a > M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

static constexpr int dx8[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
static constexpr int dy8[8] = { 0, 1, 1, 1, 0,-1,-1,-1};
static constexpr int dx4[4] = { 1,-1, 0, 0};
static constexpr int dy4[4] = { 0, 0, 1,-1};

} // namespace fe
