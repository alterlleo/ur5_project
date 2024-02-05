#include "utilities.h"

namespace Project{

    Eigen::Matrix3d rpy2rotm(double roll, double pitch, double yaw){
        Eigen::Matrix3d rotm;
        rotm << cos(yaw) * cos(pitch),  cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll),   cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll),
                sin(yaw) * cos(pitch),  sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),   sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll),
                -sin(pitch),            cos(pitch) * sin(roll),                                     cos(pitch) * cos(roll);

                return rotm;
    }

    Eigen::Matrix3d rpy2rotm(Eigen::Vector3d rpy){
        return rpy2rotm(rpy[0], rpy[1], rpy[2]);
    }

    Eigen::Vector3d rotm2rpy(Eigen::Matrix3d rotm){
        Eigen::Vector3d rpy(atan2(rotm(2, 1), rotm(2, 2)), atan2(-rotm(2, 0), sqrt(pow(rotm(2, 1), 2) + pow(rotm(2, 2), 2))), atan2(rotm(1, 0), rotm(0, 0)));
        return rpy;
    }

    Eigen::Matrix4d homTfromRPYXYZ(Eigen::VectorXd p) {

		double x = p[0];
		double y = p[1];
		double z = p[2];
		double g = p[3];
		double b = p[4];
		double a = p[5];
		Eigen::Matrix4d t;
		t << cos(a) * cos(b), cos(a) * sin(b) * sin(g) - sin(a) * cos(g), cos(a) * sin(b) * cos(g) + sin(a) * sin(g), x,
				sin(a) * cos(b), sin(a) * sin(b) * sin(g) + cos(a) * cos(g), sin(a) * sin(b) * cos(g) -
																			 cos(a) * sin(g), y,
				-sin(b), cos(b) * sin(g), cos(b) * cos(g), z,
				0.0, 0.0, 0.0, 1.0;
		return t;
	}
}

