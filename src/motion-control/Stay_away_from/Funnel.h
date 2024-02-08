#ifndef CPP_FUNNEL_H
#define CPP_FUNNEL_H

#include "Obstacle.h"

namespace Project {

    class Funnel : public Obstacle {
    public:
        Eigen::Vector2d gradient(Eigen::Vector2d point);
        Funnel();
        Funnel(Eigen::Vector2d position, double diameter, double height);
        Funnel(Eigen::Vector2d position, double diameter, double height, double maxDist);

    protected:

    private:
        double maxDist;

    };

} // Robotics

#endif //CPP_FUNNEL_H
