#include "mrp_rvo/rvo.hpp"

int main(int argc, char **argv)
{
  mrp_motion_planner::RVO rvo;
  rvo.start();
  return 0;
}