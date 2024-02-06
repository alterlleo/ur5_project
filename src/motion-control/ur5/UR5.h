#ifndef __UR5_H__
#define __UR5_H__

#include <libraries.h>
#include <structs.h>
#include <iostream>
#include "../src/motion-control/trajectory/move_linear/move_linear.h"

namespace Project {
	class UR5 {
	public:
		UR5(ros::NodeHandle &node);

		Eigen::VectorXd get_position();
		Eigen::VectorXd get_joint_states();
		Eigen::VectorXd get_gripper_states();

		static Eigen::Matrix4d direct(Eigen::VectorXd q);
		static std::vector <Eigen::VectorXd> inverse(Eigen::VectorXd p);
		static Eigen::MatrixXd jacobian(Eigen::VectorXd q);

		void send_des_state(const Eigen::VectorXd &joint_pos);
		void send_gripper_state(const float n);
		
		// consider deleting move_to_position and move and follow_trajectory (without object) <-------

		// bool move_to_position(Eigen::Vector3d target, double final_yaw, ObstacleAvoidance &obstacle,	std::vector <Eigen::Vector2d> &obstacle_poses, double time = 5.0);
		bool move_to_object(ObjectPose object_pose, Obstacle &obstacle_av,
					      std::vector <Eigen::Vector2d> &obstacle_pos, float height);
		bool linear_motion(Eigen::VectorXd target, double time = 5.0);
		
		bool move_to_position_with_object(std::string model_name, Eigen::Vector3d target, double final_yaw, Obstacle &obstacle, std::vector <Eigen::Vector2d> &obstacle_poses, double time = 5.0);
		
		bool trajectory_without_object(Move_linear &trajectory);
		bool trajectory_without_object(Move_trajectory &trajectory);
							 
		bool trajectory_with_object(std::string model_name, Move_trajectory &trajectory);
		bool trajectory_with_object(std::string model_name, Move_linear &trajectory);

	protected:

	private:
		static constexpr double a[6] = {0.0000, -0.425, -0.3922, 0.0000, 0.0000, 0.0000};
		static constexpr double d[6] = {0.1625, 0.000, 0.0000, 0.1333, 0.0997, 0.2500};
		static constexpr double speed_limits[6] = {3.15, 3.15, 3.15, 3.20, 3.20, 3.20};

		static Eigen::Matrix4d transformation01(double th);
		static Eigen::Matrix4d transformation12(double th);
		static Eigen::Matrix4d transformation23(double th);
		static Eigen::Matrix4d transformation34(double th);
		static Eigen::Matrix4d transformation45(double th);
		static Eigen::Matrix4d transformation56(double th);
		static Eigen::VectorXd get_joint_velocities(Eigen::MatrixXd jacobian, Eigen::VectorXd velocity);

		static Eigen::VectorXd
		get_vel(Eigen::Vector3d cur_position, Eigen::Vector3d desiredPosition, Eigen::Matrix3d currentRotation,
				Eigen::Matrix3d desiredRotation, double timeStep);

		bool real_robot;
		ros::NodeHandle *node;
		ros::ServiceClient client;
		ros::Publisher pub_des_jstate;
	};
}

#endif
