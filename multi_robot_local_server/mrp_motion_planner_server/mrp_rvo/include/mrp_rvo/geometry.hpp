#ifndef MRP_RVO__GEOMETRY_HPP_
#define MRP_RVO__GEOMETRY_HPP_

#include <Eigen/Dense>

namespace mrp_rvo
{
  class Line
  {
  public:
    Line(Eigen::Vector2d normal, Eigen::Vector2d point)
        : normal_(normal),
          point_(point){};
    ~Line()
    {
      // Construct line equation from point and normal
      a_ = normal_(0);
      b_ = normal_(1);

      c_ = a_ * point_(0) + b_ * point_(1);
    };

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
    
    // Coefficients of line equation ax + by = c
    double a_;
    double b_;
    double c_;
  };

  class HalfPlane
  {
    public:

    protected:
      bool upper_half_;

  };
}

#endif