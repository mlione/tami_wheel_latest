#pragma once

#include <cmath>
#include <limits>

namespace wheel_perception::core::road_geometry {

// Road directions point toward +X; the left unit normal is (-sin(yaw), cos(yaw)).
inline double lateralInterceptFromSignedNormalDistance(double distance, double yaw) {
  const double normal_y = std::cos(yaw);
  if (!std::isfinite(distance) || !std::isfinite(yaw) || normal_y <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return distance / normal_y;
}

inline double lateralInterceptFromPointDirection(
    double point_x, double point_y, double direction_x, double direction_y) {
  if (!std::isfinite(point_x) || !std::isfinite(point_y) ||
      !std::isfinite(direction_x) || !std::isfinite(direction_y) ||
      std::abs(direction_x) <= 1e-6) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return point_y - point_x * direction_y / direction_x;
}

// Signed perpendicular distance: positive is on the left side of the line.
inline double signedDistanceToLine(double x, double y, double yaw, double intercept) {
  return -std::sin(yaw) * x + std::cos(yaw) * (y - intercept);
}

}  // namespace wheel_perception::core::road_geometry
