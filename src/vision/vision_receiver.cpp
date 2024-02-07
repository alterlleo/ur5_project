#include "vision_receiver.h"

namespace Project{

    std::vector <ObjectPose> vision_client(ros::NodeHandle &node){
		ros::ServiceClient vision_receiver = node.serviceClient<robotics_project::VisionResults>("vision");
		vision_receiver.waitForExistence();

		robotics_project::VisionResults service;

        std::vector <ObjectPose> objects_poses;
		if (vision_receiver.call(service)) {
			for (auto pose: service.response.x.poses) {
                ObjectPose object;

                // label assignement
				if (pose.name == "X1-Y1-Z2") {
					object.name = Block_name::X1_Y1_Z2;
				} else if (pose.name == "X1-Y2-Z1") {
					object.name = Block_name::X1_Y2_Z1;
				} else if (pose.name == "X1-Y2-Z2") {
					object.name = Block_name::X1_Y2_Z2;
				} else if (pose.name == "X1-Y2-Z2-CHAMFER") {
					object.name = Block_name::X1_Y2_Z2_CHAMFER;
				} else if (pose.name == "X1-Y2-Z2-TWINFILLET") {
					object.name = Block_name::X1_Y2_Z2_TWINFILLET;
				} else if (pose.name == "X1-Y3-Z2") {
					object.name = Block_name::X1_Y3_Z2;
				} else if (pose.name == "X1-Y3-Z2-FILLET") {
					object.name = Block_name::X1_Y3_Z2_FILLET;
				} else if (pose.name == "X1-Y4-Z1") {
					object.name = Block_name::X1_Y4_Z1;
				} else if (pose.name == "X1-Y4-Z2") {
					object.name = Block_name::X1_Y4_Z2;
				} else if (pose.name == "X2-Y2-Z2") {
					object.name = Block_name::X2_Y2_Z2;
				} else if (pose.name == "X2-Y2-Z2-FILLET") {
					object.name = Block_name::X2_Y2_Z2_FILLET;
				}

                //pose and rotation assignement
				object.x = pose.pose.x;
				object.y = pose.pose.y;
				object.theta = pose.pose.theta;
				object.face = pose.face;
				
                // pupulate objectpose array
				objects.push_back(object);
			}
		} else {
			std::cout << "vision service didn't work, you should re-do \n" << std::endl;
		}
        
		return objects;
    }
}
