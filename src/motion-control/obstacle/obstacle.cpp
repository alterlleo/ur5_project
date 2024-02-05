#include "obstacle.h"

namespace Project{

    Obstacle::Obstacle(Eigen::Vector2d dims){
        this -> dims = dims;
    }

    void set_start(Eigen::Vector2d pos, double d, double height = 1.0){

        start = Hill(pos, d, height);

    }
    
    void set_target(Eigen::Vector2d pos, double d, double height = -1.0){
        target = Hill(pos, d, height);
    }
    
    void add_obstacle(Eigen::Vector2d pos, double d, double height = 1.0){
        obstacles.emplace_back(Hill(pos, d, height));
    }


    Eigen::Vector2d gradient(Eigen::Vector2d pos){
        Eigen::Vector2d start_gradient = start.gradient(pos);
        Eigen::Vector2d target_gradient = target.gradient(pos);
        Eigen::Vector2d obstacles_gradient(0.0, 0.0);

        for(auto obs : obstacles){
            obstacles_gradient = obstacles_gradient + obstacles.gradient(pos);
        }

        return start_gradient + target_gradient + obstacles_gradient;
    }
    
    
    std::vector<Eigen::Vector2d> path(double step, double precision, int max){
        std::vector<Eigen::Vector2d> path;
        Eigen::Vector2d start_pos = start.get_pos();
        Eigen::Vector2d target_pos = target.get_pos();
        Eigen::Vector2d pos = start_pos;

        path.push_back(start_pos);
        
        int n = 0;
        while(n++ < max && (pos - target_pos).norm() > precision){
            pos -= gradient(pos).normilized() * step;
            path.push_back(pos);
        }

        return path;
    }


}