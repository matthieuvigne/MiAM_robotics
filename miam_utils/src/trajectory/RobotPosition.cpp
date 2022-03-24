/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/trajectory/RobotPosition.h"

#include <cmath>

namespace miam{

    RobotPosition::RobotPosition():
        x(0.0),
        y(0.0),
        theta(0.0)
    {

    }


    RobotPosition::RobotPosition(double const& xIn, double const& yIn, double const& thetaIn):
        x(xIn),
        y(yIn),
        theta(thetaIn)
    {

    }


    RobotPosition operator+(RobotPosition const& p1, RobotPosition const& p2)
    {
        return RobotPosition(p1.x + p2.x, p1.y + p2.y, p1.theta + p2.theta);
    }


    RobotPosition operator-(RobotPosition const& p1, RobotPosition const& p2)
    {
        return RobotPosition(p1.x - p2.x, p1.y - p2.y, p1.theta - p2.theta);
    }


    RobotPosition operator*(double const& scalar, RobotPosition const& p1)
    {
        return RobotPosition(scalar * p1.x, scalar * p1.y, scalar * p1.theta);
    }


    RobotPosition operator*(RobotPosition const& p1, double const& scalar)
    {
        return RobotPosition(scalar * p1.x, scalar * p1.y, scalar * p1.theta);
    }


    RobotPosition operator/(RobotPosition const& p1, double const& scalar)
    {
        if(std::abs(scalar) < 1e-6)
            return p1;
        return RobotPosition(p1.x / scalar, p1.y / scalar, p1.theta / scalar);
    }


    double RobotPosition::norm() const
    {
        return std::sqrt(x *x + y * y);
    }


    void RobotPosition::normalize()
    {
        double n = norm();
        if(std::abs(n) > 1e-6)
        {
            x /= n;
            y /= n;
        }
    }


    double RobotPosition::dot(RobotPosition const& secondVector) const
    {
        return x * secondVector.x + y * secondVector.y;
    }


    double RobotPosition::cross(RobotPosition const& secondVector) const
    {
        return  x * secondVector.y - y * secondVector.x;
    }


    void RobotPosition::projectOnto(RobotPosition const& projectionTarget, RobotPosition & colinear, RobotPosition & normal)
    {
        colinear = this->dot(projectionTarget) * projectionTarget;
        colinear.theta = 0;
        normal = *this - colinear;
        normal.theta = 0;
    }


    RobotPosition RobotPosition::rotate(double const& thetaIn)
    {
        RobotPosition p;
        p.theta = this-> theta + thetaIn;
        p.x = std::cos(thetaIn) * this->x - std::sin(thetaIn) * this->y;
        p.y = std::sin(thetaIn) * this->x + std::cos(thetaIn) * this->y;
        return p;
    }


    std::ostream& operator<<(std::ostream& os, const RobotPosition& p)
    {
        os << "x: " << p.x;
        os << " y: " << p.y;
        os << " theta: " << p.theta;
        return os;
    }
}
