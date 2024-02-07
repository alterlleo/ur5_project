#ifndef __MOVE_LINEAR_H__
#define __MOVE_LINEAR_H__

#include <vector>
#include <Eigen/Dense>
#include "../../utilities/utilities.h"
// #include "../../obstacle/obstacle.h"
#include <obstacle.h>

namespace Project{

    class Move_linear{
        public:
            Move_linear();
            Move_linear(Eigen::VectorXd starting_pos, Eigen::VectorXd target_pos, double time, double step = 0.001);
            
            double get_time();
            double time_step;
            std::vector<Eigen::VectorXd> points;
    };
}

#endif

