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
            /**
             * @brief Move_trajectory default constructr. It does nothing, but it is necessary   
            */
            Move_trajectory();

            /**
             * @brief trajectory with cubic interpolation ensuring smooth movement 
             * @param start_point: starting 3D vector
             * @param target_point: target 3D vector
             * @param starting_yaw: starting yaw
             * @param target_yaw: target yaw
             * @param obstacle: obtsacles to avoid
             * @param obstacle_poses: X and Y poses of obstacles
             * @param time: time
             * @param time_step: time step 
             * @param height: final distance between end-effector and table = 0.25
             * @param distance: offset of 0.05
            */
            Move_trajectory(Eigen::Vector3d start_point, Eigen::Vector3d traget_point, double starting_yaw, double target_yaw, Stay_away_from obstacle, std::vector<Eigen::Vector2d> obstacle_poses, double time, double time_step,  double height = 0.25, double distance = 0.05);
            
            double get_time();
            double time_step;
            std::vector<Eigen::VectorXd> points;

        private:

            /**
             * @brief vertical trajectory ensuring smooth movement
             * @param start_height: starting height
             * @param height: final distance between end-effector and table
             * @param distance: offset
            */
            std::vector<double> move_vertical(double starting_height, double height, double distance);

            /**
             * @brief horizontal trajectory it creates checkpoints avoiding obstacles
             * @param obstacle: obtsacles to avoid
             * @param distance: offset
              *@param step: time step between a position and another
             * @param precision: how much distant we want to be from the table limits
             * @param max: maximum time
            */
            std::vector<Eigen::Vector2d> move_horizontal(Project::Stay_away_from obstacle, double distance, double step = 0.001, double precision = 0.0015, int max = 5000);
            
    };

}

#endif
