#ifndef __HILL_H__
#define __HILL_H__

#include <Eigen/Dense>

namespace Project{

    class Hill{
        public:

            Hill();
            Hill(Eigen::Vector2d pos, double d, double height);
            Eigen::Vector2d gradient(Eigen::Vector2d pos);
            Eigen::Vector2d get_pos();

        private:
            Eigen::Vector2d pos;
            double d;
            double height;
    };
}


#endif
