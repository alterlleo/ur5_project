#include <libraries.h>

namespace Project{

    Move_trajectory::Move_trajectory(){}

    Move_trajectory::Move_trajectory(Eigen::Vector3d start_point, Eigen::Vector3d target_point, double starting_yaw, double target_yaw, Obstacle obstacle, std::vector<Eigen::Vector2d> obstacle_poses, double time, double time_step, double distance, double height){
        this -> time_step = time_step;
        obstacle.set_start({start_point[0], start_point[1]}, 0.05);
        obstacle.set_target({target_point[0], target_point[1]}, 0.05);

        for(auto obstacle_pos : obstacle_poses){
            obstacle.add_obstacle(obstacle_pos, 0.1);
        }

        std::vector<double> starting_heights = move_vertical(start_point[2], height, distance);
        std::vector<Eigen::Vector2d> horizontal_points = move_horizontal(obstacle, distance);
        std::vector<double> final_heights = move_vertical(target_point[2], height, distance);
        std::reverse(final_heights.begin(), final_heights.end());

        std::vector<Checkpoint> checkpoints;


        // vertical motion

        double vertical_vel = starting_heights[1] - starting_heights[0];
        for(auto z : starting_heights){
            checkpoints.push_back({Eigen::Vector3d(start_point[0], start_point[1], z), Eigen::Vector3d(0.0, 0.0, vertical_vel)});
        }

        // horizontal motion
        if(horizontal_points.size() == 1){
            checkpoints.push_back({Eigen::Vector3d(horizontal_points[0][0], horizontal_points[0][1], height), Eigen::Vector3d((target_point[0] - start_point[0]) / 2.0, (target_point[1] - start_point[1]) / 2.0, 0.0)});
        
        } else if(horizontal_points.size() > 1){
            checkpoints.push_back({Eigen::Vector3d(horizontal_points[0][0], horizontal_points[0][1], height), Eigen::Vector3d((horizontal_points[1][0] - start_point[0]) / 2.0, (horizontal_points[1][1] - start_point[1]) / 2.0, 0.0)});

            for(int i = 1; i < horizontal_points.size() - 1; i++){
                checkpoints.push_back({Eigen::Vector3d(horizontal_points[i][0], horizontal_points[i][1], height), Eigen::Vector3d((horizontal_points[i + 1][0] - horizontal_points[i - 1][0]) / 2.0, (horizontal_points[i + 1][1] - horizontal_points[i - 1][1]) / 2.0, 0.0)});
            }

            checkpoints.push_back({Eigen::Vector3d(horizontal_points.back()[0], horizontal_points.back()[1], height), Eigen::Vector3d((target_point[0] - horizontal_points[horizontal_points.size() - 2][0]) / 2.0, (target_point[1] - horizontal_points[horizontal_points.size() - 2][1]) / 2.0, 0.0)});
        }

        vertical_vel = final_heights[1] - final_heights[0];
        for(auto z : final_heights){
            checkpoints.push_back({Eigen::Vector3d(target_point[0], target_point[1], z), Eigen::Vector3d(0.0, 0.0, vertical_vel)});
        }

        /*
Spline s(checkpoints);

        int nPoints = 1 + time / timeStep;
        const int nCurves = s.getCurves().size();
//        std::cout << nCurves << " curves" << std::endl << std::endl;
        for (int i = 0; i < nPoints; ++i) {
            double index = nCurves * quinticInterpolation((double) i / (nPoints - 1));
            int curveIndex = std::min((int) index, nCurves - 1);

            double rotationIndex = 0.0;
            if (index >= 1.0 and index <= nCurves - 1) {
                rotationIndex = (index - 1.0) / (nCurves - 2);
            } else if (index > nCurves - 2) {
                rotationIndex = 1.0;
            }
            rotationIndex = quinticInterpolation(rotationIndex);
//            std::cout << index << " " << curveIndex << " " << index - curveIndex << " " << rotationIndex << std::endl;

            Eigen::Vector3d position = s.getCurves()[curveIndex].point(index - curveIndex);
            Eigen::VectorXd point(6);
            double yaw = startYaw + rotationIndex * (targetYaw - startYaw);
			point << position[0], position[1], position[2], 0.0, 0.0, yaw;
            points.push_back(point);
        }

        */
    }

    std::vector<double> Move_trajectory::move_vertical(double starting_height, double height, double distance){
        std::vector<double> vertical_points;
        double z = abs(starting_height - height);
        double x;

        vertical_points.push_back(starting_height);
        if(z <= distance * 5.0 / 3.0){
            x = (height - starting_height) * 3.0 / 5.0;
            vertical_points.push_back(starting_height + x);

        } else{
            int d = (z - distance * 5.0 / 3.0) / distance;
            x = (height - starting_height) / (5.0 / 3.0 + d);

            for(int i = 1; i <= d + 1; i++){
                vertical_points.push_back(starting_height + x * i);
            }
        }

        return vertical_points;
    }


    std::vector<Eigen::Vector2d> Move_trajectory::move_horizontal(Project::Obstacle obstacle, double distance, double step, double precision, int max){
        std::vector<Eigen::Vector2d> horizontal_points;
        std::vector<Eigen::Vector2d> path = obstacle.path(step, precision, max);
        double lenght = path.size() * step;

        if(lenght <= distance * 2.0 / 3){

        } else if(lenght <= distance * 4.0 / 3){
            horizontal_points.push_back(path.at(path.size() / 2.0));

        } else{
            int horizontal_steps = floor(lenght / distance - 4.0 / 3);
            double checkpoints = lenght / (horizontal_steps + 4.0 / 3);

            for(int i = 0; i <= horizontal_steps; i++){
                int tmp = checkpoints / step * (2.0 / 3 + i);
                horizontal_points.push_back(path.at(tmp));
            }
        }

        return horizontal_points;
    }

    double Move_trajectory::get_time(){
        return time_step;
    }


}