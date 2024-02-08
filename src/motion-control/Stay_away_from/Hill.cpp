#include "Hill.h"

namespace Project {
    Hill::Hill() : Hill({0.0, 0.0}, 1.0, 1.0) {
    }

    Hill::Hill(Eigen::Vector2d position, double diameter, double height)
            : Obstacle(position, diameter, height) {
    }

    Eigen::Vector2d Hill::gradient(Eigen::Vector2d point) {
        if ((point - position).isMuchSmallerThan(0.001))
            return {0.0, 0.0};
        double r = (point - position).norm();
        double module = 0.0;
        Eigen::Vector2d direction = (point - position) / r;
        if (r <= diameter / 2) {
            module = -4 * height / (diameter * diameter) * r;
        } else if (r <= diameter) {
            module = 4 * height / (diameter * diameter) * r - 4 * height / diameter;
        } else {
            module = 0.0;
        }
        return direction * module;
    }
} // Robotics
