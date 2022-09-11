#include <pluginlib/class_loader.hpp>
#include "mrp_centralised_server_core/global_task_planner.h"
#include "mrp_hungarian/hungarian_algorithm/hungarian.h"
#include <iostream>

int main(int argc, char **argv)
{
  // To avoid unused parameter warnings
  (void)argc;
  (void)argv;

  pluginlib::ClassLoader<global_centralised_server_core::GlobalTaskPlannerCore> poly_loader("mrp_centralised_server_core", "global_centralised_server_core::GlobalTaskPlannerCore");
  std::vector<std::string> all_class = poly_loader.getDeclaredClasses();
  std::cout << all_class.size() << std::endl;
  std::cout << poly_loader.getBaseClassType() << std::endl;

  try
  {
    std::shared_ptr<global_centralised_server_core::GlobalTaskPlannerCore> triangle = poly_loader.createSharedInstance("mrp_hungarian::MRPHungarian");
    triangle->initialise();
  }
  catch (pluginlib::PluginlibException &ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}