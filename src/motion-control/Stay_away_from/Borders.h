#ifndef __BORDERS_H__
#define __BORDERS_H__

#include "Obstacle.h"

namespace Project {

    class Borders : public Obstacle {
    public:
        Eigen::Vector2d gradient(Eigen::Vector2d point);
        Borders();
        Borders(Eigen::Vector2d position, double diameter, double height);
        Borders(Eigen::Vector2d position, double diameter, double height, double maxDist);

    protected:

    private:
        double maxDist;

    };

} // Robotics

#endif //__BORDERS_H__
