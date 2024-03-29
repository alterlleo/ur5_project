#include "objects.h"

using namespace Project;

Eigen::Vector3d transform_position(Eigen::Vector3d position){
    return {TABLE_XMAX - position[0], TABLE_YMAX - position[1], TABLE_Z + position[2]};
}

bool object_positioning(ros::ServiceClient &spawner, std::string name, std::string object_name, Eigen::Vector3d position, Eigen::Vector3d rotation){
    
    // memorization of the model.sdf path by playing with strings
    std::ifstream path("/home/leo/Models/" + object_name + "/model.sdf");
    std::stringstream buff;
    buff << path.rdbuf();
    std::string sdf = buff.str();
    
    // setup model pose
    Eigen::Quaterniond quat(rpy2rotm(rotation));
    Eigen::Vector3d transformed_position = transform_position(position);
    geometry_msgs::Quaternion q;
    q.w = quat.w();
    q.x = quat.x();
    q.y = quat.y();
    q.z = quat.z();
    geometry_msgs::Point coordinates;
    coordinates.x = transformed_position[0];
    coordinates.y = transformed_position[1];
    coordinates.z = transformed_position[2];

    // memorization of position (coordinates) and rotation in a single variable pos
    geometry_msgs::Pose pos;
    pos.position = coordinates;
    pos.orientation = q;

    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = name;
    srv.request.model_xml = sdf;
    srv.request.robot_namespace = "";
    srv.request.initial_pose = pos;
    srv.request.reference_frame = "world";

    // check if the service is all right, otherwise it's a mess
    if(!spawner.call(srv)){
        ROS_ERROR("-----> Service gazebo/spawn_sdf_model ERROR <----");
        return false;
    } else{
        return srv.response.success;
    }
}


// ---------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "object_positioning");
    ros::NodeHandle node;

    ros::ServiceClient spawner_client = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    spawner_client.waitForExistence();

    // first draft of positioning:
    object_positioning(spawner_client, "X1-Y1-Z2", "X1-Y1-Z2", {0.8, 0.2, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z1", "X1-Y2-Z1", {0.9, 0.2, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z2", "X1-Y2-Z2", {0.9, 0.3, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-CHAMFER", {0.7, 0.1, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z2-TWINFILLET", "X1-Y2-Z2-TWINFILLET", {0.7, 0.6, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y3-Z2", "X1-Y3-Z2", {0.8, 0.6, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y3-Z2-FILLET", "X1-Y3-Z2-FILLET", {0.9, 0.5, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y4-Z1", "X1-Y4-Z1", {0.5, 0.1, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y4-Z2", "X1-Y4-Z2", {0.6, 0.6, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X2-Y2-Z2", "X2-Y2-Z2", {0.6, 0.2, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X2-Y2-Z2-FILLET", "X2-Y2-Z2-FILLET", {0.7, 0.4, 0.0}, {0.0, 0.0, 0.0});
    

    return 0;
}
