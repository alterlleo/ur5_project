#ifndef CPP_OBSTACLEAVOIDANCE_H
#define CPP_OBSTACLEAVOIDANCE_H

#include <vector>

#include <Eigen/Dense>

#include "Hill.h"
#include "Funnel.h"

namespace Project{

    class Stay_away_from{
        public:
            Stay_away_from();
            Stay_away_from(Eigen::Vector2d dims);

            void set_start(Eigen::Vector2d pos, double d, double height = 1.0);
            void set_target(Eigen::Vector2d pos, double d, double height = -1.0);
            void add_obstacle(Eigen::Vector2d pos, double d, double height = 1.0);
            Eigen::Vector2d gradient(Eigen::Vector2d pos);
            std::vector<Eigen::Vector2d> path(double step, double precision, int max);

        private:
            Eigen::Vector2d dims;
            Hill start;
            Funnel target;
            std::vector<Hill> obstacles;
    };
}



#endif //CPP_OBSTACLEAVOIDANCE_H
