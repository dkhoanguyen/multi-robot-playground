#include <pluginlib/class_loader.hpp>
#include "mrp_local_server_core/local_controller.hpp"
#include "spotturn_controller/spotturn_controller.hpp"
#include <iostream>

int main(int argc, char **argv)
{
  // To avoid unused parameter warnings
  (void)argc;
  (void)argv;

  pluginlib::ClassLoader<local_server_core::LocalController> poly_loader("mrp_local_server_core", "local_server_core::LocalController");
  std::vector<std::string> all_class = poly_loader.getDeclaredClasses();
  std::cout << all_class.size() << std::endl;
  std::cout << poly_loader.getBaseClassType() << std::endl;

  try
  {
    std::shared_ptr<local_server_core::LocalController> triangle = poly_loader.createSharedInstance("spotturn_controller::SpotturnController");
    triangle->initialise();
  }
  catch (pluginlib::PluginlibException &ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}