#ifndef _MR_GLOBAL_TASK_PLANNER_H_
#define _MR_GLOBAL_TASK_PLANNER_H_

// Multi robot Playground
namespace global_centralised_server_core
{
  class GlobalTaskPlannerCore
  {
  public:
    virtual ~GlobalTaskPlannerCore(){};
    virtual void initialise() = 0;
    virtual void planTasks() = 0;
    virtual void assignTasks() = 0;
  };
}
#endif