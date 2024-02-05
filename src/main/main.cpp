#include <libraries>
#include "structs.h"

using namespace Project;

int main(int argc, char **argv){
    std::cout << "starting grasping operation node ... \n";
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;

    UR5 ur5(node);
    Eigen::VectoXd current_pos;
    Eigen::Vector3d start_pos;
    Eigen::Vector3d target_pos;

    double start_yaw = 0.0;
    double target_yaw = 0.0;
    double time = 5.0;
    double step = 0.001;

    Eigen::Vector2d table_dims{1.000, 0.650};
    Eigen::Vector2d singularity{0.500, 0.450};
    Obstacle obstacle{table_dims};
    obstacle.add_obstacle(singularity, 0.25);
    std::vector<Eigen::Vector2d> obstacles_pos;
    Move_trajectory trajectory;
    bool response;
    std::vector<ObjectPose> objects;
    std::vector<Eigen::Vector3d> targets_pos;
    targets_pos.reserve(11);
    /*
    aggiungi target finali per classe
    
    */

   current_pos = ur5.get_position();
   Eigen::VectorXd joint_target(6);
   joint_target << current_pos.head(3), 0.0, 0.0, 0.0;
   response = ur5.move_linear(joint_target, 1.0);

   target_pos << 0.0, 0.5, 0.45;
   response = ur5.move_to_position(target_pos, 0.0, obstacle, obstacles_pos);

   ur5.send_gripper_state(2.0);

   ObjectPose tmp;
    tmp.x = 0.9;
	tmp.y = 0.3;
	tmp.z = 0.0;
	tmp.theta = 0.0;
	tmp.face = TOP;
	tmp.name = name;
	tmp.gazeboName = "X1-Y1-Z2";

   float height = 0.05;
   float clamp = 1.0;

   response = ur5.move_to_object(tmp, obstacle, obstacles_pos, height);

   ur5.send_gripper_state(clamp);
   
   current_pos = ur5.get_position();
   
   response = ur5.move_to_position_with_object("X1-Y1-Z2", {0.05, 0.1, 0.0}, 0.1, obstacle, obstacles_pos);

   ur5.send_gripper_state(2.0);

   // move up
   current_pos = ur5.get_position();
   joint_target << current_pos.head(2), 0.35, 0.0, 0.0, 0.0;
   response = ur5.move_linear(joint_target, 3.0);

   return 0;   
}

