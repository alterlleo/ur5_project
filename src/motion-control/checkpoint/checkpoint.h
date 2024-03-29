#ifndef __CHECKPOINT_H__
#define __CHECKPOINT_H__

#include <iostream>
#include "Eigen/Dense"

namespace Project{

    /**
     * @class Checkpoint class which stores 3D position and 3D velocity. It is used for intermediate points in a trajectory
    */
    class Checkpoint{
        public:
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;

            friend std::ostream& operator << (std::ostream& os, const Checkpoint& cp);

    };
}

#endif
