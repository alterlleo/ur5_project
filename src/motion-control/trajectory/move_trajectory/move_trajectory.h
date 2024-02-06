#ifndef __MOVE_TRAJECTORY_H__
#define __MOVE_TRAJECTORY_H__

#include <vector>
#include <Eigen/Dense>
#include "../../utilities/utilities.h"
#include "../../obstacle/obstacle/obstacle.h"

namespace Project{

    class Move_trajectory{
        public:

            Move_trajectory();
            Move_trajectory(Eigen::Vector3d start_point, Eigen::Vector3d traget_point, double starting_yaw, double target_yaw, Obstacle obstacle, std::vector<Eigen::Vector2d> obstacle_poses, double time, double time_step,  double height = 0.25, double distance = 0.05);
            
            double get_time();
            double time_step;
            std::vector<Eigen::VectorXd> points;

        private:

            std::vector<double> move_vertical(double starting_height, double height, double distance);
            std::vector<Eigen::Vector2d> move_horizontal(Project::Obstacle obstacle, double distance, double step = 0.001, double precision = 0.0015, int max = 5000);
            
    };

}

#endif
