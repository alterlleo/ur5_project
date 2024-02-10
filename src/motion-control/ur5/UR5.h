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
		 * @param joint_pos: vector
		*/
		void actuate_ur5(const Eigen::VectorXd &joint_pos);

		/**
		 * @brief publish message on a specific topic, allowing other ROS nodes to receive and use the gripper state information
		 * @param n
		*/
		void actuate_gripper(const float n);
		
		// consider deleting move_to_position and move and follow_trajectory (without object) <-------

		/**
		 * @brief compute the trajectory and send joint states in order to move the ur5 without a grasped object
		 * @param target: final position
		 * @param final_yaw: final yaw
		 * @param obstacle: address of a Stay_away_from instance
		 * @param obstacle_poses: 2D position of obstacle
		 * @param time: time 
		*/
		bool move_to_position_without_object(Eigen::Vector3d target, double final_yaw, Stay_away_from &obstacle,	std::vector <Eigen::Vector2d> &obstacle_poses, double time = 5.0);
		
		/**
		 * @brief move the robot from initial pose to the object pose, then the movement is "linear"
		 * @param object_pose: target position 
		 * @param obstacle_av: obstacle to avoid
		 * @param obstacle_pos: 2D position of obstacle
		 * @param height: desired height
		*/
		bool move_to_object(ObjectPose object_pose, Stay_away_from &obstacle_av, std::vector <Eigen::Vector2d> &obstacle_pos, float height);
		
		/**
		 * @brief move the robot in a linear trajectory
		 * @param target: targetvector 
		 * @param time: time = 5.0
		*/
		bool linear_motion(Eigen::VectorXd target, double time = 5.0);
		
		/**
		 * @brief move the robot with the grasped object to the right position
		 * @param model_name: name of model 
		 * @param target: target 3D position
		 * @param final_yaw: final yaw
		 * @param obstacle: obstacle to avoid
		 * @param obstacle_poses: vector of 2D poses of obstacles
		 * @param time: max time
		*/
		bool move_to_position_with_object(std::string model_name, Eigen::Vector3d target, double final_yaw, Stay_away_from &obstacle, std::vector <Eigen::Vector2d> &obstacle_poses, double time = 5.0);
		
		/**
		 * @brief UR5 linear trajectory to be traversed
		 * @param trajectory: linear trajectory
		*/
		bool trajectory_without_object(Move_linear &trajectory);

		/**
		 * @brief UR5 trajectory to be traversed
		 * @param trajectory: trajectory
		*/
		bool trajectory_without_object(Move_trajectory &trajectory);

		/**
		 * @brief use the same trajectory for both the robot and for the object
		 * @param model_name: name of model 
		 * @param trajectory: trajectory
		*/					 
		bool trajectory_with_object(std::string model_name, Move_trajectory &trajectory);

		/**
		 * @brief use the same linear trajectory for both the robot and for the object
		 * @param model_name: name of model 
		 * @param trajectory: linear trajectory
		*/					
		bool trajectory_with_object(std::string model_name, Move_linear &trajectory);

	protected:

	private:
		static constexpr double a[6] = {0.0000, -0.425, -0.3922, 0.0000, 0.0000, 0.0000};
		static constexpr double d[6] = {0.1625, 0.000, 0.0000, 0.1333, 0.0997, 0.2500};
		static constexpr double speed_limits[6] = {3.15, 3.15, 3.15, 3.20, 3.20, 3.20};

		/**
		 * @brief transformation matrix 0->1
		 * @param th: theta
		*/
		static Eigen::Matrix4d transformation01(double th);

		/**
		 * @brief transformation matrix 1->2
		 * @param th: theta
		*/
		static Eigen::Matrix4d transformation12(double th);
		
		/**
		 * @brief transformation matrix 2->3
		 * @param th: theta
		*/
		static Eigen::Matrix4d transformation23(double th);

		/**
		 * @brief transformation matrix 3->4
		 * @param th: theta
		*/
		static Eigen::Matrix4d transformation34(double th);

		/**
		 * @brief transformation matrix 4->5
		 * @param th: theta
		*/
		static Eigen::Matrix4d transformation45(double th);
		
		/**
		 * @brief transformation matrix 5->6
		 * @param th: theta
		*/
		static Eigen::Matrix4d transformation56(double th);

		/**
		 * @brief compute the joint velocities based on the provided Jacobian matrix and desired end-effector velocities
		 * @param jacobian: jacobian
		 * @param velocity: velocity
		*/
		static Eigen::VectorXd get_joint_velocities(Eigen::MatrixXd jacobian, Eigen::VectorXd velocity);

		/**
		 * @brief generate the desired velocity for the robot to reach the desired position and orientation in a specified time step
		 * @param cur_pos: 3D current position
		 * @param desired_pos: 3D desired position
		 * @param cur_rotation: current rotation matrix
		 * @param desired_rotation: desired rotation matrix
		 * @param time_step: time step
		*/
		static Eigen::VectorXd get_vel(Eigen::Vector3d cur_pos, Eigen::Vector3d desired_pos, Eigen::Matrix3d cur_rotation, Eigen::Matrix3d desired_rotation, double time_step);

		bool real_robot;
		ros::NodeHandle *node;
		ros::ServiceClient client;
		ros::Publisher pub_des_jstate;
	};
}

#endif
