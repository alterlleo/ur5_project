#ifndef __BORDERS_H__
#define __BORDERS_H__

#include "Obstacle.h"

namespace Project {
    /**
     * @class Borders class
    */
    class Borders : public Obstacle {
    public:
        /**
         * @brief computes the gradient of a given position with respect to the borders of a geometric shape, returning a vector indicating the direction and intensity of the gradient at that point
         * @param points: 2D position of the obstacle
        */
        Eigen::Vector2d gradient(Eigen::Vector2d point);
        
        /**
         * @brief Borders class default constructor
        */
        Borders();

        /**
         * @brief Borders specific contructor
         * @param position: 2D position of the obstacle
         * @param diameter: ipothetic radial diameter of the obstacle
         * @param height: ipothetic height of the obstacle
        */
        Borders(Eigen::Vector2d position, double diameter, double height);

        /**
         * @brief Borders specific contructor
         * @param position: 2D position of the obstacle
         * @param diameter: ipothetic radial diameter of the obstacle
         * @param height: ipothetic height of the obstacle
         * @param maxDist: distance from borders
        */
        Borders(Eigen::Vector2d position, double diameter, double height, double maxDist);

    protected:

    private:
        double maxDist;

    };

} // Robotics

#endif //__BORDERS_H__
