#ifndef MRP_RVO__GEOMETRY_HPP_
#define MRP_RVO__GEOMETRY_HPP_

#include <Eigen/Dense>
#include <iostream>

namespace mrp_orca
{
  namespace common
  {
    class Line
    {
    public:
      Line(Eigen::Vector2d normal, Eigen::Vector2d point)
          : normal_(normal),
            point_(point)
      {
        // Construct line equation from point d and normal
        a_ = normal_(0);
        b_ = normal_(1);

        // Line equation: a(x - x0) + b(y - y0) = 0
        c_ = a_ * point_(0) + b_ * point_(1);
      };
      ~Line(){};

      Eigen::Vector2d point() const
      {
        return point_;
      }

      Eigen::Vector2d normal() const
      {
        return normal_;
      }

      double a() const
      {
        return a_;
      }

      double b() const
      {
        return b_;
      }

      double c() const
      {
        return c_;
      }

    protected:
      Eigen::Vector2d normal_;
      Eigen::Vector2d point_;

      // Constants of line equation ax + by = c
      double a_;
      double b_;
      double c_;
    };

    class HalfPlane
    {
    public:
      HalfPlane(Line line, Eigen::Vector2d direction)
          : line_(line), direction_(direction){};
      ~HalfPlane(){};

      Line line() const
      {
        return line_;
      }

      Eigen::Vector2d direction() const
      {
        return direction_;
      }

    protected:
      Line line_;
      Eigen::Vector2d direction_;
    };
  }

}

#endif