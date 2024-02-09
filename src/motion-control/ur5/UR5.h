#ifndef __UR5_H__
#define __UR5_H__


#include <libraries.h>
#include <structs.h>
#include <iostream>
//#include "../src/motion-control/trajectory/move_linear/move_linear.h"

namespace Project {

	/**
	 * @class Main class that controls the ur5 movement
	*/
	class UR5 {
	public:

		/**
		 * @brief UR5 constructor
		 * @param node: rosnode address
		*/
		UR5(ros::NodeHandle &node);

		/**
		 * @brief Get current position of the end-effector
		*/
		Eigen::VectorXd get_position();

		/**
		 * @brief Get current state of all the joints
		*/
		Eigen::VectorXd get_joint_states();

		/**
		 * @brief Get current state of the gripper
		*/
		Eigen::VectorXd get_gripper_states();

		/**
		 * @brief Compute the direct kinematics
		 * @param q: joint configuration
		*/
		static Eigen::Matrix4d direct(Eigen::VectorXd q);

		/**
		 * @brief Compute the inverse kinematics
		 * @param p: position
		*/
		static std::vector <Eigen::VectorXd> inverse(Eigen::VectorXd p);

		/**
		 * @brief Compute the jacobian matrix
		 * @param q: joint configuration
		*/
		static Eigen::MatrixXd jacobian(Eigen::VectorXd q);

		/**
		 * @brief this publishing mechanism allows other ROS nodes to receive and use the desired state to control the UR5 robot
		 * @param joint_pos;
		*/
		void actuate_ur5(const Eigen::VectorXd &joint_pos);

		/**
		 * @brief publish message on a specific topic, allowing other ROS nodes to receive and use the gripper state information
		 * @param n
		*/
		void send_gripper_state(const float n);
		
		// consider deleting move_to_position and move and follow_trajectory (without object) <-------

		/**
		 * @brief Compute the trajectory and send joint states in order to move the ur5 without a grasped object
		 * @param target: final position
		 * @param final_yaw
		 * @param obstacle: address of a Stay_away_from instance
		 * @param obstacle_poses: 
		 * @param time
		*/
		bool move_to_position_without_object(Eigen::Vector3d target, double final_yaw, Stay_away_from &obstacle,	std::vector <Eigen::Vector2d> &obstacle_poses, double time = 5.0);
		
		/**
		 * @brief 
		 * @param
		 * @param
		 * @param
		 * @param
		 * @param
		*/
		bool move_to_object(ObjectPose object_pose, Stay_away_from &obstacle_av, std::vector <Eigen::Vector2d> &obstacle_pos, float height);
		bool linear_motion(Eigen::VectorXd target, double time = 5.0);
		
		bool move_to_position_with_object(std::string model_name, Eigen::Vector3d target, double final_yaw, Stay_away_from &obstacle, std::vector <Eigen::Vector2d> &obstacle_poses, double time = 5.0);
		
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
