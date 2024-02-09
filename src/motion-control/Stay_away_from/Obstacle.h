#ifndef CPP_OBSTACLE_H
#define CPP_OBSTACLE_H

#include <Eigen/Dense>

namespace Project {

    /**
     * @class Obstacle base class
    */
    class Obstacle {
    public:

        /**
         * @brief Obstacle class default constructor
        */
        Obstacle();

        /**
         * @brief Obstacle specific contructor
         * @param position: 2D position of the obstacle
         * @param diameter: ipothetic radial diameter of the obstacle
         * @param height: ipothetic height of the obstacle
        */
        Obstacle(Eigen::Vector2d position, double diameter, double height);

        /**
         * @brief Compute the gradient of a position
         * @param position: position
        */
        Eigen::Vector2d gradient(Eigen::Vector2d position);

        /***
         * @brief Return the obstacle position
        */
        Eigen::Vector2d getPosition();

    protected:
        Eigen::Vector2d position;
        double diameter;
        double height;

    private:

    };

} // Robotics

#endif //CPP_OBSTACLE_H
