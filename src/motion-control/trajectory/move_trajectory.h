#ifndef __MOVE_TRAJECTORY_H__
#define __MOVE_TRAJECTORY_H__

#include <libraries.h>

namespace Project{

    class Move_trajectory : public Trajectory{
        public:

            Move_trajectory();
            Move_trajectory(Eigen::Vector3d start_point, Eigen::Vector3d traget_point, double starting_yaw, double target_yaw, Project::Obstacle obstacle, std::vector<Eigen::Vector2d> obstacle_poses, double time, double time_step, double distance = 0.05, double height = 0.25);

        private:

            std::vector<double> move_vertical(double starting_height, double height, double distance);
            std::vector<Eigen::Vector2d> move_horizontal(Project::Obstacle obstacle, double distance, double step = 0.001, double precision = 0.0015, int max = 5000);
            
    };

}

#endif