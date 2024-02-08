#include "objects.h"

using namespace Project;

/*Eigen::Vector3d transform_position(Eigen::Vector3d position){
    return {1.0 - position[0], 0.8 - position[1], position[2] + 0.87};
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
    coordinates.x = position[0];
    coordinates.y = position[1];
    coordinates.z = position[2];

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
    object_positioning(spawner_client, "X1-Y1-Z2", "X1-Y1-Z2", {0.9, 0.3, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z1", "X1-Y2-Z1", {0.9, 0.5, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z2", "X1-Y2-Z2", {0.8, 0.2, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-CHAMFER", {0.7, 0.1, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y2-Z2-TWINFILLET", "X1-Y2-Z2-TWINFILLET", {0.8, 0.4, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y3-Z2", "X1-Y3-Z2", {0.7, 0.5, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y3-Z2-FILLET", "X1-Y3-Z2-FILLET", {0.5, 0.5, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y4-Z1", "X1-Y4-Z1", {0.6, 0.1, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X1-Y4-Z2", "X1-Y4-Z2", {0.7, 0.2, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X2-Y2-Z2", "X2-Y2-Z2", {0.5, 0.2, 0.0}, {0.0, 0.0, 0.0});
	object_positioning(spawner_client, "X2-Y2-Z2-FILLET", "X2-Y2-Z2-FILLET", {0.8, 0.7, 0.0}, {0.0, 0.0, 0.0});
    

    return 0;
}
*/



Eigen::Vector3d transformPosition(Eigen::Vector3d pos) {
	return {1.0 - pos[0], 0.8 - pos[1], pos[2] + 0.87};
}

bool spawnObject(std::string brickName, Eigen::Vector3d position, Eigen::Vector3d rpy, std::string modelName, ros::ServiceClient &spawner_client) {
	Eigen::Quaterniond quat(rpy2rotm(rpy));
	position = transformPosition(position);
	//gazebo_ros_link_attacher::Attach attachReq;

	geometry_msgs::Quaternion g_q;
	g_q.w = quat.w();
	g_q.x = quat.x();
	g_q.y = quat.y();
	g_q.z = quat.z();

	geometry_msgs::Point g_point;
	g_point.x = position[0];
	g_point.y = position[1];
	g_point.z = position[2];

	geometry_msgs::Pose g_pose;
	g_pose.position = g_point;
	g_pose.orientation = g_q;

	std::ifstream in("/home/leo/Models/" + brickName + "/model.sdf");
	std::stringstream buffer;
	buffer << in.rdbuf();
	std::string sdf = buffer.str();

	gazebo_msgs::SpawnModel srv;
	srv.request.model_name = modelName;
	srv.request.model_xml = sdf;
	srv.request.robot_namespace = "";
	srv.request.initial_pose = g_pose;
	srv.request.reference_frame = "world";
	if (!spawner_client.call(srv))
	{
		ROS_ERROR("Failed to call service gazebo/spawn_sdf_model");
		return false;
	}

	return  srv.response.success;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "objects_spawner");
	ros::NodeHandle node;
	
	// ros::ServiceServer attach_service = node.advertiseService("/link_attacher_node/attach", Attach);
	
			
	ros::ServiceClient spawner_client = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	spawner_client.waitForExistence();


//// faccia sinistra
//	spawnObject("X1-Y2-Z1", {0.9, 0.4, 0.016}, {0.0, M_PI_2, -0.6}, "ghgressf", spawner_client);
//	spawnObject("X1-Y2-Z2", {0.9, 0.3, 0.016}, {0.0, M_PI_2, -0.5}, "ghkgdifsf", spawner_client);
//	spawnObject("X1-Y2-Z2-CHAMFER", {0.9, 0.1, 0.016}, {0.0, M_PI_2, -0.6}, "ghnutyrtrytsf", spawner_client);
//	spawnObject("X1-Y2-Z2-TWINFILLET", {0.9, 0.3, 0.016}, {0.0, M_PI_2, -0.6}, "ghnutjygyhrtrytsf", spawner_client);
//	spawnObject("X1-Y3-Z2", {0.9, 0.3, 0.016}, {0.0, M_PI_2, -0.6}, "ghnhrjtyf", spawner_client);
//	spawnObject("X1-Y3-Z2-FILLET", {0.9, 0.3, 0.016}, {0.0, M_PI_2, -0.6}, "ghgjtsf", spawner_client);
//	spawnObject("X1-Y4-Z1", {0.7, 0.1, 0.016}, {0.0, M_PI_2, -0.7}, "ghg", spawner_client);
//	spawnObject("X1-Y4-Z2", {0.9, 0.4, 0.016}, {0.0, M_PI_2, -0.7}, "gruyjgdfsske", spawner_client);
//	spawnObject("X2-Y2-Z2", {0.9, 0.3, 0.031}, {0.0, M_PI_2, 2.4}, "gtjyf", spawner_client);
//	spawnObject("X2-Y2-Z2-FILLET", {0.9, 0.3, 0.031}, {0.0, M_PI_2, 2.4}, "gtjuyhro67def", spawner_client);

//// faccia destra
	//      spawnObject("X1-Y2-Z2-CHAMFER", {0.9, 0.3, 0.016}, {0.0, -M_PI_2, -0.6}, "heykuyert", spawner_client);
	//      spawnObject("X1-Y3-Z2-FILLET", {0.7, 0.1, 0.016}, {0.0, -M_PI_2, -0.6}, "yhterher", spawner_client);
	//	spawnObject("X2-Y2-Z2-FILLET", {0.9, 0.3, 0.031}, {0.0, -M_PI_2, 2.4}, "gtkyyetrefy", spawner_client);


////faccia sotto
	spawnObject("X1-Y1-Z2", {0.9, 0.3, 0.0}, {0.0, 0.0, 0.0}, "X1-Y1-Z2", spawner_client);
	spawnObject("X1-Y2-Z1", {0.9, 0.5, 0.0}, {0.0, 0.0, 0.0}, "X1-Y2-Z1", spawner_client);
	/*	spawnObject("X1-Y2-Z2", {0.8, 0.2, 0.0}, {0.0, 0.0, 0.0}, "X1-Y2-Z2", spawner_client);
	spawnObject("X1-Y2-Z2-CHAMFER", {0.7, 0.1, 0.0}, {0.0, 0.0, 0.0}, "X1-Y2-Z2-CHAMFER", spawner_client);
	spawnObject("X1-Y2-Z2-TWINFILLET", {0.8, 0.4, 0.0}, {0.0, 0.0, 0.0}, "X1-Y2-Z2-TWINFILLET", spawner_client);
	spawnObject("X1-Y3-Z2", {0.7, 0.5, 0.0}, {0.0, 0.0, 0.0}, "X1-Y3-Z2", spawner_client);
	spawnObject("X1-Y3-Z2-FILLET", {0.5, 0.5, 0.0}, {0.0, 0.0, 0.0}, "X1-Y3-Z2-FILLET", spawner_client);
	spawnObject("X1-Y4-Z1", {0.6, 0.1, 0.0}, {0.0, 0.0, 0.0}, "X1-Y4-Z1", spawner_client);
	spawnObject("X1-Y4-Z2", {0.7, 0.2, 0.0}, {0.0, 0.0, 0.0}, "X1-Y4-Z2", spawner_client);
	spawnObject("X2-Y2-Z2", {0.5, 0.2, 0.0}, {0.0, 0.0, 0.0}, "X2-Y2-Z2", spawner_client);
	spawnObject("X2-Y2-Z2-FILLET", {0.8, 0.7, 0.0}, {0.0, 0.0, 0.0}, "X2-Y2-Z2-FILLET", spawner_client);	*/
//// faccia sopra
//	spawnObject("X1-Y1-Z2", {0.9, 0.3, 0.057}, {0.0, M_PI, -0.4}, "gfgsuytfdhgw", spawner_client);
//	spawnObject("X1-Y2-Z1", {0.6, 0.3, 0.057}, {0.0, M_PI, +0.4}, "gujhjytru", spawner_client);
//	spawnObject("X1-Y2-Z2", {0.3, 0.4, 0.057}, {0.0, M_PI, +0.4}, "gujhggerru", spawner_client);
//	spawnObject("X1-Y2-Z2-TWINFILLET", {0.9, 0.4, 0.057}, {0.0, M_PI, +0.4}, "guhhertesru", spawner_client);
//	spawnObject("X1-Y3-Z2", {0.9, 0.4, 0.057}, {0.0, M_PI, +0.4}, "guhjty[sru", spawner_client);
//	spawnObject("X1-Y4-Z1", {0.9, 0.4, 0.038}, {0.0, M_PI, +0.4}, "guhfu", spawner_client);
//	spawnObject("X1-Y4-Z2", {0.9, 0.4, 0.057}, {0.0, M_PI, +0.4}, "gubhgsfdhh", spawner_client);
//	spawnObject("X2-Y2-Z2", {0.9, 0.3, 0.057}, {0.0, M_PI, +0.4}, "johrtytr", spawner_client);

//// faccia davanti
//	spawnObject("X1-Y1-Z2", {0.9, 0.4, 0.0155}, {-M_PI_2, 0.0, 2.4}, "gsghhyttr", spawner_client);
//	spawnObject("X1-Y2-Z1", {0.9, 0.3, 0.031}, {-M_PI_2, 0.0, 0.4}, "gttgtretytr", spawner_client);
//	spawnObject("X1-Y2-Z2", {0.9, 0.3, 0.031}, {-M_PI_2, 0.0, 0.4}, "grgjytytr", spawner_client);
//	spawnObject("X1-Y2-Z2-CHAMFER", {0.9, 0.3, 0.031}, {-M_PI_2, 0.0, 2.4}, "gthgsfdteytr", spawner_client);
//	spawnObject("X1-Y3-Z2", {0.9, 0.1, 0.048}, {-M_PI_2, 0.0, 2.4}, "gtfuhrtytruwf", spawner_client);
//	spawnObject("X1-Y3-Z2", {0.5, 0.1, 0.048}, {-M_PI_2, 0.0, 2.4}, "gtfhtruwf", spawner_client);
//	spawnObject("X1-Y3-Z2-FILLET", {0.9, 0.4, 0.048}, {-M_PI_2, 0.0, 2.4}, "gtfjfhjef", spawner_client);
//	spawnObject("X1-Y4-Z1", {0.9, 0.4, 0.064}, {-M_PI_2, 0.0, 2.4}, "ghhitlef", spawner_client);
//	spawnObject("X1-Y4-Z2", {0.9, 0.4, 0.064}, {-M_PI_2, 0.0, 2.4}, "gtjbgfddjref", spawner_client);
//	spawnObject("X2-Y2-Z2", {0.9, 0.3, 0.031}, {-M_PI_2, 0.0, 2.4}, "gtju67u67def", spawner_client);
//	spawnObject("X2-Y2-Z2-FILLET", {0.9, 0.3, 0.031}, {-M_PI_2, 0.0, 2.4}, "gtju67u67def", spawner_client);

	return 0;
}  // server errore se non ha matrice trasformazione omogenea se non è su faccia giusta
