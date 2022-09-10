#ifndef _MR_GLOBAL_TASK_PLANNER_H_
#define _MR_GLOBAL_TASK_PLANNER_H_

// Multi robot Playground
namespace global_task_planner_core
{
  class GlobalTaskPlanner
  {
  public:
    virtual ~GlobalTaskPlanner(){};
    virtual void initialise() = 0;
    virtual void planTasks() = 0;
    virtual void assignTasks() = 0;
  };
};
#endif