#ifndef __SINGULARITIES_H__
#define __SINGULARITIES_H__

#include "Obstacle.h"

namespace Project {

    class Singularities : public Obstacle {
    public:
        Singularities();
        Singularities(Eigen::Vector2d position, double diameter, double height);
        Eigen::Vector2d gradient(Eigen::Vector2d point);

    protected:

    private:

    };

} // Robotics

#endif //__SINGULARITIES_H__
