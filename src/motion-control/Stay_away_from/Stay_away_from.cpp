#include "Stay_away_from.h"

namespace Project {

    Stay_away_from::Stay_away_from(){}

    Stay_away_from::Stay_away_from(Eigen::Vector2d dims) {
        this->dims = dims;
    }

    void Stay_away_from::set_start(Eigen::Vector2d pos, double d, double height) {
        start = Hill(pos, d, height);
    }
    
    void Stay_away_from::set_target(Eigen::Vector2d pos, double d, double height) {
        target = Funnel(pos, d, height, 2.0);
    }
    
    void Stay_away_from::add_obstacle(Eigen::Vector2d pos, double d, double height) {
        obstacles.emplace_back(Hill(pos, d, height));
    }

    Eigen::Vector2d Stay_away_from::gradient(Eigen::Vector2d pos) {
        Eigen::Vector2d start_gradient = start.gradient(pos);
        Eigen::Vector2d target_gradient = target.gradient(pos);
        Eigen::Vector2d obstacles_gradient(0.0, 0.0);

        for (auto obs : obstacles) { // Corrected: Use auto& to access by reference
            obstacles_gradient += obs.gradient(pos); // Corrected: Use obs.gradient(pos) instead of obstacles.gradient(pos)
        }

        return start_gradient + target_gradient + obstacles_gradient;
    }
    
    std::vector<Eigen::Vector2d> Stay_away_from::path(double step, double precision, int max) {
        std::vector<Eigen::Vector2d> path;
        Eigen::Vector2d start_pos = start.getPosition();
        Eigen::Vector2d target_pos = target.getPosition();
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
