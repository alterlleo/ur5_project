#include "move_linear.h"

namespace Project{
    Move_linear::Move_linear(){ }

    Move_linear::Move_linear(Eigen::VectorXd starting_pos, Eigen::VectorXd target_pos, double time, double step){
        this -> time_step = step;
        int number = 1 + time / step;

        /*
        for(int i = 0; i < number; i++){
            double counter = cubic interpolation
        }
        */

        Eigen::Vector3d p0 = {starting_pos[0], starting_pos[1], starting_pos[2]};
        Eigen::Vector3d ps = {target_pos[0], target_pos[1], target_pos[2]};
        Eigen::Vector3d p0_vel = {starting_pos[3], starting_pos[4], starting_pos[5]};
        Eigen::Vector3d ps_vel = {target_pos[3], target_pos[4], target_pos[5]};
        double t0 = 0.0;
        double ts = 2.0;      

       // parameters definition
        Eigen::Vector3d a0 = p0;
        Eigen::Vector3d a1 = p0_vel;
        Eigen::Vector3d a3 = (-2 * ps + ts*ps_vel - p0_vel * ts + 2 * p0_vel * ts + 2 * p0) / (ts * ts * ts);
        Eigen::Vector3d a2 = (ps_vel - 3 * a3 * ts * ts - p0_vel) / (2 * ts);
    
        for(double t = 0; t < 2.0; t = t + 0.01){
            Eigen::VectorXd point(6);

            //cubic polinomial
            Eigen::Vector3d new_pos = a0 + a1 * t + a2 * t * t + a3 * t * t * t;

            point << new_pos[0], new_pos[1], new_pos[2], 0.0, 0.0, 0.0;
            points.push_back(point);
        }

       //points.push_back(starting_pos);
       //points.push_back(target_pos);
    }
    
    double Move_linear::get_time(){
        return time_step;
    }
}
