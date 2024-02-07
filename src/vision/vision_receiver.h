#ifndef __VISION_RECEIVER__
#define __VISION_RECEIVER__

#include <libraries.h>
#include <structs.h>

namespace Project {
	std::vector <ObjectPose> vision_client(ros::NodeHandle &node);
}

#endif 