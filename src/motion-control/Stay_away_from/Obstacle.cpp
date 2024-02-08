#include "Obstacle.h"

namespace Project {
    Obstacle::Obstacle() {
        Obstacle({0.0, 0.0}, 1.0, 1.0);
    }

    Obstacle::Obstacle(Eigen::Vector2d position, double diameter, double height) {
        this->position = position;
        this->diameter = diameter;
        this->height = height;
    }

    Eigen::Vector2d Obstacle::gradient(Eigen::Vector2d point) {
        return {0.0, 0.0};
    }

    Eigen::Vector2d Obstacle::getPosition() {
        return position;
    }
} // Robotics
