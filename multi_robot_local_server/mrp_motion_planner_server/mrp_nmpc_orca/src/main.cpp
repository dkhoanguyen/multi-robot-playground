#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <iostream>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <typeinfo>

int main()
{
  Eigen::Vector2d test({1,1});
  Eigen::VectorXd x = test.transpose() * test;
  std::cout << (double)x(0)<< std::endl;
}