#ifndef __SINGULARITIES_H__
#define __SINGULARITIES_H__

#include "Obstacle.h"

namespace Project {

    class Singularities : public Obstacle {
    public:
        /**
         * @brief Singularities class default constructor
        */
        Singularities();

        /**
         * @brief Singularities specific contructor
         * @param position: 2D position of the obstacle
         * @param diameter: ipothetic radial diameter of the obstacle
         * @param height: ipothetic height of the obstacle
        */
        Singularities(Eigen::Vector2d position, double diameter, double height);

        /**
         * @brief compute the gradient of a given position with respect to the singularities, returning a vector indicating the direction and intensity of the gradient at that point
         * @param point: 2D position of the obstacle
        */
        Eigen::Vector2d gradient(Eigen::Vector2d point);

    protected:

    private:

    };

} // Robotics

#endif //__SINGULARITIES_H__
