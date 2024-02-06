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
       points.push_back(starting_pos);
       points.push_back(target_pos);
    }
    
    double Move_linear::get_time(){
        return time_step;
    }
}
