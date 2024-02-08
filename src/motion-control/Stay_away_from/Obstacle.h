#ifndef CPP_OBSTACLE_H
#define CPP_OBSTACLE_H

#include <Eigen/Dense>

namespace Project {

    class Obstacle {
    public:
        Obstacle();
        Obstacle(Eigen::Vector2d position, double diameter, double height);
        Eigen::Vector2d gradient(Eigen::Vector2d position);
        Eigen::Vector2d getPosition();

    protected:
        Eigen::Vector2d position;
        double diameter;
        double height;

    private:

    };

} // Robotics

#endif //CPP_OBSTACLE_H
