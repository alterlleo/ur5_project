#include "move_linear.h"
#include "../../Stay_away_from/Stay_away_from.h"


namespace Project{
    Move_linear::Move_linear(){ }

    Move_linear::Move_linear(Eigen::VectorXd starting_pos, Eigen::VectorXd target_pos, double time, double step){
        this -> time_step = step;

        points.push_back(starting_pos);
        points.push_back(target_pos);
    }
    
    double Move_linear::get_time(){
        return time_step;
    }
}
