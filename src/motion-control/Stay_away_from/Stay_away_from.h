#ifndef CPP_OBSTACLEAVOIDANCE_H
#define CPP_OBSTACLEAVOIDANCE_H

#include <vector>

#include <Eigen/Dense>

#include "Hill.h"
#include "Funnel.h"

namespace Project{

    /**
     * @class Stay_away_from is used in order to keep some distance between end end effector and obstacles
    */
    class Stay_away_from{
        public:

            /**
             * @brief Stay_away_from class default constructor
            */
            Stay_away_from();

            /**
             * @brief Stay_away_from specific constructor
             * @param dims: table dimensions (default obstacle)
            */
            Stay_away_from(Eigen::Vector2d dims);

            /**
             * @brief set the start obstacle as a _____sinonimo hill____
             * @param pos: 2D position of the obstacle
             * @param d: ipothetical diemeter of the obstacle
             * @param height: ipothetical height of the obstalce, by default it is set to 1.0
            */
            void set_start(Eigen::Vector2d pos, double d, double height = 1.0);

            /**
             * @brief set the target obstacle as a _____sinonimo hill____
             * @param pos: 2D position of the obstacle
             * @param d: ipothetical diemeter of the obstacle
             * @param height: ipothetical height of the obstalce, by default it is set to -1.0
            */
            void set_target(Eigen::Vector2d pos, double d, double height = -1.0);

            /**
             * @brief add another obstacle to the list
             * @param pos: 2D position of the obstacle
             * @param d: ipothetical diemeter of the obstacle
             * @param height: ipothetical height of the obstalce, by default it is set to 1.0
            */
            void add_obstacle(Eigen::Vector2d pos, double d, double height = 1.0);

            /**
             * @brief Compute the gradient of the position with respect to all the obstacles
             * @param pos: position
            */
            Eigen::Vector2d gradient(Eigen::Vector2d pos);

            /**
             * @brief Compute a safe path between obstacles
             * @param step: time step between a position and another
             * @param precision: how much distant we want to be from the table limits
             * @param max: maximum time
            */
            std::vector<Eigen::Vector2d> path(double step, double precision, int max);

        private:
            Eigen::Vector2d dims;
            Hill start;
            Funnel target;
            std::vector<Hill> obstacles;
    };
}



#endif //CPP_OBSTACLEAVOIDANCE_H
