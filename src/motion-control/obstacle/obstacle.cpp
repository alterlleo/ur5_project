#include "obstacle.h"
#include "hill.h"

namespace Project {

    Obstacle::Obstacle(){}

    Obstacle::Obstacle(Eigen::Vector2d dims) {
        this->dims = dims;
    }

    void Obstacle::set_start(Eigen::Vector2d pos, double d, double height) {
        start = Hill(pos, d, height);
    }
    
    void Obstacle::set_target(Eigen::Vector2d pos, double d, double height) {
        target = Hill(pos, d, height);
    }
    
    void Obstacle::add_obstacle(Eigen::Vector2d pos, double d, double height) {
        obstacles.emplace_back(Hill(pos, d, height));
    }

    Eigen::Vector2d Obstacle::gradient(Eigen::Vector2d pos) {
        Eigen::Vector2d start_gradient = start.gradient(pos);
        Eigen::Vector2d target_gradient = target.gradient(pos);
        Eigen::Vector2d obstacles_gradient(0.0, 0.0);

        for (auto obs : obstacles) { // Corrected: Use auto& to access by reference
            obstacles_gradient += obs.gradient(pos); // Corrected: Use obs.gradient(pos) instead of obstacles.gradient(pos)
        }

        return start_gradient + target_gradient + obstacles_gradient;
    }
    
    std::vector<Eigen::Vector2d> Obstacle::path(double step, double precision, int max) {
        std::vector<Eigen::Vector2d> path;
        Eigen::Vector2d start_pos = start.get_pos();
        Eigen::Vector2d target_pos = target.get_pos();
        Eigen::Vector2d pos = start_pos;

        path.push_back(start_pos);
        
        int n = 0;
        while (n++ < max && (pos - target_pos).norm() > precision) {
            pos -= gradient(pos).normalized() * step; // Corrected: Use normalized() instead of normilized()
            path.push_back(pos);
        }

        return path;
    }

}
