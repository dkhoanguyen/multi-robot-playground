#include "mrp_hungarian/mrp_hungarian.hpp"

namespace mrp_hungarian
{

  MRPHungarian::MRPHungarian()
  {
  }

  MRPHungarian::~MRPHungarian()
  {
  }

  void MRPHungarian::initialise()
  {
  }

  void MRPHungarian::planTasks()
  {
  }

  void MRPHungarian::assignTasks()
  {
  }

} // namespace mrp_hungarian

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mrp_hungarian::MRPHungarian, global_centralised_server_core::GlobalTaskPlannerCore)