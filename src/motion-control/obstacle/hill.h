#ifndef __HILL_H__
#define __HILL_H__

#include <Eigen/Dense>

namespace Project{

    class Hill{
        public:
            Eigen::Vector2d pos;
            double d;
            double height;

            Hill();
            Hill(Eigen::Vector2d pos, double d, double height);
            Eigen::Vector2d gradient(Eigen::Vector2d pos);
            Eigen::Vector2d get_pos();
    };
}


#endif
