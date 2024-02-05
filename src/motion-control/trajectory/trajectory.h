#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include <libraries.h>

namespace Project{

    class Trajectory{
        public:
        	double get_time();
            double time_step;
            std::vector<Eigen::VectorXd> points;
    };
}

#endif
