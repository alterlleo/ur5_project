#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include <libraries.h>

namespace Project{

    class Trajectory{
        public:
            double time_step;
            std::vector<Eigen::VectorXd> points;
    };
}

#endif
