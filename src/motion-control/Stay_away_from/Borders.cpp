#include "Borders.h"

namespace Project {
    Borders::Borders() : Borders({0.0, 0.0}, 1.0, 1.0) {
    }

    Borders::Borders(Eigen::Vector2d position, double diameter, double height, double maxDist)
            : Obstacle(position, diameter, height) {
        this->maxDist = maxDist;
    }

    Borders::Borders(Eigen::Vector2d position, double diameter, double height)
            : Borders(position, diameter, height, 10.0) {
    }

    Eigen::Vector2d Borders::gradient(Eigen::Vector2d point) {
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
            module = 1 / (maxDist - diameter);
        }
        return direction * module;
    }
}
