#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include <vector>
#include <Eigen/Dense>
#include "../hill/hill.h"

namespace Project{

    class Obstacle{
        public:
            Obstacle();
            Obstacle(Eigen::Vector2d dims);

            void set_start(Eigen::Vector2d pos, double d, double height = 1.0);
            void set_target(Eigen::Vector2d pos, double d, double height = -1.0);
            void add_obstacle(Eigen::Vector2d pos, double d, double height = 1.0);
            Eigen::Vector2d gradient(Eigen::Vector2d pos);
            std::vector<Eigen::Vector2d> path(double step, double precision, int max);

        private:
            Eigen::Vector2d dims;
            Hill start;
            Hill target;
            std::vector<Hill> obstacles;
    };
}



#endif
