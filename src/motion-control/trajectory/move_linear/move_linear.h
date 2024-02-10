#ifndef __MOVE_LINEAR_H__
#define __MOVE_LINEAR_H__

#include <vector>
#include <Eigen/Dense>
#include "../../utilities/utilities.h"
#include "../../Stay_away_from/Stay_away_from.h"
//#include <obstacle.h>

namespace Project{

    class Move_linear{
        public:
            /**
             * @brief Move_linear default constructr. It does nothing, but it is necessary   
            */
            Move_linear();

            /**
             * @brief linear motion between two points
             * @param starting_pos: starting vector
             * @param target_pos: target vector
             * @param time: 
             * @param step: time step 
            */
            Move_linear(Eigen::VectorXd starting_pos, Eigen::VectorXd target_pos, double time, double step = 0.001);
            
            double get_time();
            double time_step;
            std::vector<Eigen::VectorXd> points;
    };
}

#endif

