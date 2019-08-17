#include "modules/planning/potential_field.h"

#include <vector>
#include "modules/common/global_gflags.h"
#include "modules/planning/proto/planning.proto"

namespace apollo {
namespace planning {

const double GRID_SIZE = 0.05;  // meters
using apollo::planning::Point;

PotentialField::PotentialField() {}

void PotentialField::Plan(Point start, Point goal,
                          std::vector<Point> obstacles) {
  if (start.empty() || goal.empty()) {
    AWARN << "START OR GOAL Point is empty";
    return;
  }
}

void PotentialField::calcPotentialField() {}

float PotentialField::calcAttractivePotential() { return 0.5 * KP; }

float PotentialField::calcRepulsivePotential() {}

}  // namespace planning
}  // namespace apollo
