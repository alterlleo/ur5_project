#ifndef __LINEAR_H__
#define __LINEAR_H__

#include "trajectory.h"

namespace Project{

    class Move_linear : public Trajectory{
        public:
            Move_linear();
            Move_linear(Eigen::VectorXd starting_pos, Eigen::VectorXd target_pos, double time, double step = 0.001);
    };
}

#endif

