#ifndef __MOVE_TRAJECTORY_H__
#define __MOVE_TRAJECTORY_H__

#include <vector>
#include <Eigen/Dense>
#include "../../utilities/utilities.h"
//#include obstacle.h>
#include "../../Stay_away_from/Stay_away_from.h"
#include "../../checkpoint/checkpoint.h"

namespace Project{

    class Move_trajectory{
        public:

            Move_trajectory();
            Move_trajectory(Eigen::Vector3d start_point, Eigen::Vector3d traget_point, double starting_yaw, double target_yaw, Stay_away_from obstacle, std::vector<Eigen::Vector2d> obstacle_poses, double time, double time_step,  double height = 0.25, double distance = 0.05);
            
            double get_time();
            double time_step;
            std::vector<Eigen::VectorXd> points;

        private:

            std::vector<double> move_vertical(double starting_height, double height, double distance);
            std::vector<Eigen::Vector2d> move_horizontal(Project::Stay_away_from obstacle, double distance, double step = 0.001, double precision = 0.0015, int max = 5000);
            
    };

}

#endif
