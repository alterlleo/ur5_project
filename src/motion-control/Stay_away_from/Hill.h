#ifndef CPP_HILL_H
#define CPP_HILL_H

#include "Obstacle.h"

namespace Project {

    class Hill : public Obstacle {
    public:
        Hill();
        Hill(Eigen::Vector2d position, double diameter, double height);
        Eigen::Vector2d gradient(Eigen::Vector2d point);

    protected:

    private:

    };

} // Robotics

#endif //CPP_HILL_H
