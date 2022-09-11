#ifndef MRP_HUNGARIAN__MRP_HUNGARIAN_HPP_
#define MRP_HUNGARIAN__MRP_HUNGARIAN_HPP_

// Plugin
#include "mrp_centralised_server_core/global_task_planner.h"

#include "mrp_hungarian/visibility_control.h"
#include <iostream>

namespace mrp_hungarian
{
  class MRPHungarian : public global_centralised_server_core::GlobalTaskPlannerCore
  {
  public:
    MRPHungarian();
    virtual ~MRPHungarian();

    void initialise();
    void planTasks();
    void assignTasks();
  };

} // namespace mrp_hungarian

#endif // MRP_HUNGARIAN__MRP_HUNGARIAN_HPP_
