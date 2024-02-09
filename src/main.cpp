
#include <libraries.h>
#include <structs.h>

using namespace Project;

int main(int argc, char **argv){
    std::cout << "starting grasping operation node ... \n";
    ros::init(argc, argv, "custom_joint_publisher");
    ros::NodeHandle node;

    UR5 ur5(node);
    Eigen::VectorXd current_pos;
    Eigen::Vector3d start_pos;
    Eigen::Vector3d target_pos;


    double start_yaw = 0.0;
    double target_yaw = 0.0;
    double time = 5.0;
    double step = 0.001;

    Eigen::Vector2d table_dims{1.000, 0.650};
    Eigen::Vector2d singularity{0.500, 0.450};
    Stay_away_from obstacle(table_dims);
    obstacle.add_obstacle(singularity, 0.25);

    /* inizialize objects */
    std::vector<Eigen::Vector2d> obstacles_pos;
    bool response;
    
    //std::vector<ObjectPose> objects;
    //objects.reserve(11);

    ObjectPose objects[11];

    /* define final positions for each block */
    
    //std::vector<Eigen::Vector3d> destinations;
    //destinations.reserve(11);
    
    Eigen::Vector3d destinations[11];

    destinations[Block_name::X1_Y1_Z2] = {0.05, 0.1, 0.0};
	destinations[Block_name::X1_Y2_Z1] = {0.05, 0.2, 0.0};
	destinations[Block_name::X1_Y2_Z2] = {0.05, 0.31, 0.0};
	destinations[Block_name::X1_Y2_Z2_CHAMFER] = {0.05, 0.4, 0.0};
	destinations[Block_name::X1_Y2_Z2_TWINFILLET] = {0.05, 0.5, 0.0};
	destinations[Block_name::X1_Y3_Z2] = {0.18, 0.4, M_PI_2};
	destinations[Block_name::X1_Y3_Z2_FILLET] = {0.18, 0.1, M_PI_2};
	destinations[Block_name::X1_Y4_Z1] = {0.18, 0.3, M_PI_2};
	destinations[Block_name::X1_Y4_Z2] = {0.18, 0.5, M_PI_2};
	destinations[Block_name::X2_Y2_Z2] = {0.15, 0.2, 0.0};
	destinations[Block_name::X2_Y2_Z2_FILLET] = {0.25, 0.2, 0.0};

    current_pos = ur5.get_position();
    Eigen::VectorXd joint_target(6);
    joint_target << current_pos.head(3), 0.0, 0.0, 0.0;
    response = ur5.linear_motion(joint_target, 1.0);

    target_pos << 0.0, 0.5, 0.45;

    //move out of the vision area
    /*
    Move_trajectory trajectory = Move_trajectory((ur5.get_position()).head(3), target_pos, ((ur5.get_position())[5]), 0.0, obstacle, obstacles_pos, time, step);
    bool res = ur5.trajectory_without_object(trajectory);
    if(!res){
        return false;
    }
    */


    response = ur5.move_to_position_without_object(target_pos, 0.0, obstacle, obstacles_pos);

    ur5.send_gripper_state(2.0);

    /*ObjectPose tmp;
    tmp = ObjectPose(0.9, 0.3, 0.0, 0.0, TOP, X1_Y1_Z2, "X1-Y1-Z2");*/


    objects[0]  = ObjectPose(0.9, 0.3, 0.0, 0.0, TOP, X1_Y1_Z2, "X1-Y1-Z2");
    objects[1]  = ObjectPose(0.9, 0.5, 0.0, 0.0, TOP, X1_Y2_Z1, "X1-Y2-Z1");
    objects[2]  = ObjectPose(0.8, 0.2, 0.0, 0.0, TOP, X1_Y2_Z2, "X1-Y2-Z2");
    objects[3]  = ObjectPose(0.7, 0.1, 0.0, 0.0, TOP, X1_Y2_Z2_CHAMFER, "X1-Y2-Z2-CHAMFER");
    objects[4]  = ObjectPose(0.8, 0.4, 0.0, 0.0, TOP, X1_Y2_Z2_TWINFILLET, "X1-Y2-Z2-TWINFILLET");
    objects[5]  = ObjectPose(0.7, 0.5, 0.0, 0.0, TOP, X1_Y3_Z2, "X1-Y3-Z2");
    objects[6]  = ObjectPose(0.5, 0.5, 0.0, 0.0, TOP, X1_Y3_Z2_FILLET, "X1-Y3-Z2-FILLET");
    objects[7]  = ObjectPose(0.6, 0.1, 0.0, 0.0, TOP, X1_Y4_Z1, "X1-Y4-Z1");
    objects[8]  = ObjectPose(0.7, 0.2, 0.0, 0.0, TOP, X1_Y4_Z2, "X1-Y4-Z2");
    objects[9]  = ObjectPose(0.5, 0.2, 0.0, 0.0, TOP, X2_Y2_Z2, "X2-Y2-Z2");
    objects[10] = ObjectPose(0.8, 0.7, 0.0, 0.0, TOP, X2_Y2_Z2_FILLET, "X2-Y2-Z2-FILLET");
  
    
    /*tmp.x = 0.9;
	tmp.y = 0.3;
	tmp.z = 0.0;
	tmp.theta = 0.0;
	tmp.face = TOP;
	tmp.name = X1_Y1_Z2;
	tmp.gazebo_name = "X1-Y1-Z2";*/
    

    float height = 0.05;
    float clamp = 1.0;

    
    //for (int i = 0; i < 11; i++){
        int i = 0;
        response = ur5.move_to_object(objects[i], obstacle, obstacles_pos, height);

        ur5.send_gripper_state(0.0);
   
        current_pos = ur5.get_position();
   
        response = ur5.move_to_position_with_object(objects[i].gazebo_name, destinations[/*static_cast<Project::Block_name>(i)*/i], 0.0, obstacle, obstacles_pos);

        ur5.send_gripper_state(2.0);

        // move up
        current_pos = ur5.get_position();
        joint_target << current_pos.head(2), 0.35, 0.0, 0.0, 0.0;
        response = ur5.linear_motion(joint_target, 3.0);
        sleep(1);
    //}
   

    return 0;   
}

