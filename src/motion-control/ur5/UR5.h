#ifndef CPP_UR5_H
#define CPP_UR5_H

#include <roboticsProject.h>
#include <structs.h>

#include <iostream>

namespace Project {
	class UR5 {
	public:
		UR5(ros::NodeHandle &node);

		bool followTrajectory(Trajectory &trajectory);

		Eigen::VectorXd get_position();
		Eigen::VectorXd get_joint_states();
		Eigen::VectorXd get_gripper_states();

		static Eigen::Matrix4d direct(Eigen::VectorXd q);
		static std::vector <Eigen::VectorXd> inverse(Eigen::VectorXd p);
		static Eigen::MatrixXd jacobian(Eigen::VectorXd q);

		void send_des_state(const Eigen::VectorXd &joint_pos);
		void send_gripper_state(const float n);

		bool move_to_position(Eigen::Vector3d target, double targetYaw, ObstacleAvoidance &obstacleAvoidance,
							std::vector <Eigen::Vector2d> &obstaclePositions, double time = 5.0);
		bool move_to_object(ObjectPose object_pose, ObstacleAvoidance &obstacle_av,
					      std::vector <Eigen::Vector2d> &obstacle_pos, float height);
		bool move_linear(Eigen::VectorXd target, double time = 5.0);

		bool attach(std::string modelName1, std::string linkName1, std::string modelName2, std::string linkName2);
		bool detach(std::string modelName1, std::string linkName1, std::string modelName2, std::string linkName2);

		bool grasp(std::string modelName, std::string linkName = "link");
		bool release(std::string modelName, std::string linkName = "link");

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

		ros::Publisher pub_des_jstate;
		ros::ServiceClient attach_srv, detach_srv;
	};
} // Robotics

#endif //CPP_UR5_H
