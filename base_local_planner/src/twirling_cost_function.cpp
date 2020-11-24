#include <base_local_planner/twirling_cost_function.h>

#include <math.h>

namespace base_local_planner {

double TwirlingCostFunction::scoreTrajectory(Trajectory &traj) {
  return fabs(traj.thetav_);  // add cost for making the robot spin
}

} /* namespace base_local_planner */
