#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <iostream>
#include <Eigen/Dense>

#include <cmath>
#include <cmath>

namespace Project{

    /**
     * @brief Compute the roll, pitch and yaw rotation to a rotation matrix
     * @param roll
     * @param pitch
     * @param yaw
    */
    Eigen::Matrix3d rpy2rotm(double roll, double pitch, double yaw);

    /**
     * @brief compute the roll, pitch and yaw rotation to a rotation matrix
     * @param rpy: 3D vector where roll, pitch and yaw are stored
    */
    Eigen::Matrix3d rpy2rotm(Eigen::Vector3d rpy);

    /**
     * @brief Compute the rotation matrix to roll, pitch and yaw 3D vector
     * @param rotm: rotation matrix
    */
    Eigen::Vector3d rotm2rpy(Eigen::Matrix3d rotm);

    /**
     * @brief compute the Roto-Transformation matrix from a 6D vector where are stored 3D position and 3D velocity
    */
	Eigen::Matrix4d homTfromRPYXYZ(Eigen::VectorXd p);
}

#endif