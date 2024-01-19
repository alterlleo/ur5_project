#ifndef __OBJECTS_H__
#define __OBJECTS_H__

#include <UR5-project.h>
#include <iostream>
#include <fstream>

using namespace Project;

/**
 * @brief   This function load the 3d models from memory and spawn them in the gazebo simulation. The gazebo simulation must be already active
 * @param   spawner: the service client from gazebo/spawn_sdf_model
 * @param   name: the model name, it's up to you
 * @param   object_name: the name of the model
 * @param   position: xyz coordinates where the model should be placed
 * @param   rotation: rpy values
*/
bool object_positioning(ros::ServiceClient &spawner, std::string name, std::string object_name, Eigen::Vector3d position, Eigen::Vector3d rotation);

#endif
