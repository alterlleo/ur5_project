#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <iostream>
#include <Eigen/Dense>

#include <cmath>
#include <cmath>

namespace Project{
    Eigen::Matrix3d rpy2rotm(double roll, double pitch, double yaw);
    Eigen::Matrix3d rpy2rotm(Eigen::Vector3d rpy);
    Eigen::Vector3d rotm2rpy(Eigen::Matrix3d rotm);

	Eigen::Matrix4d homTfromRPYXYZ(Eigen::VectorXd p);
}

#endif