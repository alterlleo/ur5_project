#include "UR5.h"

namespace Project {
    
	constexpr double UR5::d[6];
	constexpr double UR5::a[6];
	constexpr double UR5::speed_limits[6];

    //constructor
	UR5::UR5(ros::NodeHandle &node) {
		this->node = &node;
		if (this->node) {
			node.getParam("/real_robot", real_robot);
			pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
		} 
	}

    //this publishing mechanism allows other ROS nodes to receive and use the desired state to control the UR5 robot.
	void UR5::actuate_ur5(const Eigen::VectorXd &state) {
		std_msgs::Float64MultiArray jointState_msg_sim;
		for (int i = 0; i < state.size(); ++i) {
			jointState_msg_sim.data.push_back(state[i]);
		}
		pub_des_jstate.publish(jointState_msg_sim);
	}

    //publish message on a specific topic, allowing other ROS nodes to receive and use the gripper state information
	void UR5::actuate_gripper(const float n) {						
		std_msgs::Float64MultiArray jointState_msg_sim;
        	Eigen::VectorXd gripperStates = get_gripper_states();
		Eigen::VectorXd jointStates;
		if (gripperStates.size() == 2) {
			jointStates = get_joint_states();
			for (int i = 0; i < jointStates.size(); ++i) {
				jointState_msg_sim.data.push_back(jointStates[i]);
			}
			jointState_msg_sim.data.push_back(n);
			jointState_msg_sim.data.push_back(n);
			pub_des_jstate.publish(jointState_msg_sim);
		}
	}
	
	//generate the desired velocity for the robot to reach the desired position and orientation in a specified time step
	Eigen::VectorXd UR5::get_vel(Eigen::Vector3d cur_pos, Eigen::Vector3d desired_pos, Eigen::Matrix3d cur_rotation,
		Eigen::Matrix3d desired_rotation, double time_step) {
		Eigen::VectorXd vel(6);
		Eigen::Vector3d linear_error = desired_pos - cur_pos;
		Eigen::Matrix3d relativeRotation = cur_rotation.transpose() * desired_rotation;
		Eigen::Vector3d angular_vel;
		double angle = atan2(sqrt(pow(relativeRotation(2, 1) - relativeRotation(1, 2), 2) +
								  pow(relativeRotation(0, 2) - relativeRotation(2, 0), 2) +
								  pow(relativeRotation(1, 0) - relativeRotation(0, 1), 2)),
							 relativeRotation(0, 0) + relativeRotation(1, 1) + relativeRotation(2, 2) - 1.0);
		if (abs(angle) < 0.000001) {
			angular_vel << 0.0, 0.0, 0.0;
		} else {
			angular_vel = angle / (2.0 * sin(angle)) / time_step *
						 Eigen::Vector3d(relativeRotation(2, 1) - relativeRotation(1, 2),
										 relativeRotation(0, 2) - relativeRotation(2, 0),
										 relativeRotation(1, 0) - relativeRotation(0, 1));
			angular_vel = cur_rotation * angular_vel;
		}
		vel << linear_error / time_step, angular_vel;
		return vel;
	}

    //compute the joint velocities based on the provided Jacobian matrix and desired end-effector velocities
    Eigen::VectorXd UR5::get_joint_velocities(Eigen::MatrixXd jacobian, Eigen::VectorXd velocity) {
		Eigen::VectorXd jointVelocities(6), ratios(6);
		double minR, maxR;
		jointVelocities = jacobian.inverse() * velocity;
		ratios << jointVelocities[0] / speed_limits[0], jointVelocities[1] / speed_limits[1],
				jointVelocities[2] / speed_limits[2], jointVelocities[3] / speed_limits[3],
				jointVelocities[4] / speed_limits[4], jointVelocities[5] / speed_limits[5];
		minR = ratios.minCoeff();
		maxR = ratios.maxCoeff();
		jointVelocities = jointVelocities / std::max(1.0, std::max(std::abs(minR), std::abs(maxR)));
		return jointVelocities;
	}

    //move the robot from initial pose to the object pose, then the movement is "linear"
    
	bool UR5::move_to_object(ObjectPose object_pose, Stay_away_from &obstacle_av, std::vector <Eigen::Vector2d> &obstacle_pos, float height){
        Eigen::VectorXd new_joint_states;
		Eigen::VectorXd joint_states;
        Eigen::VectorXd target(6);
		Eigen::VectorXd gripper_states;
		
		bool res;

		target << object_pose.x, object_pose.y, height + 0.01, 0.0, 0.0, object_pose.theta;
		// res = move_to_position(target.head(3), target[5], obstacle_av, obstacle_pos);

		Move_trajectory trajectory = Move_trajectory((get_position()).head(3), target.head(3), ((get_position())[5]), target[5], obstacle_av, obstacle_pos, 5.0, 0.001);
		res = trajectory_without_object(trajectory);
		if(!res){
			return false;
		}

		joint_states = get_joint_states();

		double adjustement_yaw = - M_PI + object_pose.theta;

		// set gripper rotation
		target << (get_position()).head(5), adjustement_yaw;
		res = linear_motion(target, 1.0);

		target << object_pose.x, object_pose.y, height - 0.035, 0.0, 0.0, adjustement_yaw;	// little offset to grab properly

		sleep(1);

		res = linear_motion(target, 1.0);
		
		//return trajectory_without_object(trajectory);
		return res;
	}

    //move the robot from object to the final position 
    
    
    
	bool UR5::move_to_position_without_object(Eigen::Vector3d target, double final_yaw, Stay_away_from &obstacle,
							 std::vector <Eigen::Vector2d> &obstacle_poses, double time){
		double time_step = 0.001;
		Eigen::VectorXd cur_position;
		Eigen::Vector3d start;
		double starting_yaw;
		Move_trajectory trajectory;

		cur_position = get_position();
		start = cur_position.head(3);
		starting_yaw = cur_position[5];

		trajectory = Move_trajectory(start, target, starting_yaw, final_yaw, obstacle, obstacle_poses,
										  time, time_step);
		return trajectory_without_object(trajectory);
	}
	
	
	

    //move the robot in a linear trajectory
	bool UR5::linear_motion(Eigen::VectorXd target, double time) {
		Eigen::VectorXd cur_position = get_position();
		Move_linear trajectory = Move_linear(cur_position, target, time, 0.001);
		return trajectory_without_object(trajectory);
	}

    //follow a given trajectory
    
    
	bool UR5::trajectory_without_object(Move_linear &trajectory) {
	
		double time_step = trajectory.get_time();
		ros::Rate loop_rate(1.0 / (time_step * 1.0));
		sensor_msgs::JointState joint_state;
		sensor_msgs::JointStateConstPtr msg;
		Eigen::Vector3d cur_position, new_position;
		Eigen::VectorXd q(6), new_q(6), vel(6), newVel(6), joint_vel_norm(6), grip_states, state;
		Eigen::Matrix4d dir, new_dir;
		Eigen::Matrix3d cur_orientation, new_orientation;
		bool close_enough;

		grip_states = get_gripper_states();
		state.resize(grip_states.size() + 6);

		q = get_joint_states();

		for (auto desired: trajectory.points) {
			Eigen::Vector3d desired_pos = desired.head(3);
			Eigen::Matrix3d desired_orientation = rpy2rotm(desired[3], desired[4], desired[5]);
			do {
				dir = direct(q);
				cur_orientation = Eigen::Matrix3d(dir.block(0, 0, 3, 3));
				cur_position << dir(0, 3), dir(1, 3), dir(2, 3);

				vel = get_vel(cur_position, desired_pos, cur_orientation, desired_orientation, time_step);

				joint_vel_norm = get_joint_velocities(jacobian(q), vel);
				new_q = q + joint_vel_norm * time_step;

				new_dir = direct(new_q);
				new_orientation = Eigen::Matrix3d(new_dir.block(0, 0, 3, 3));
				new_position << new_dir(0, 3), new_dir(1, 3), new_dir(2, 3);

				newVel = get_vel(new_position, desired_pos, new_orientation, desired_orientation, time_step);
				close_enough = newVel.norm() * time_step < 0.002;

				q = new_q;

				state << q, grip_states;
				actuate_ur5(state);
				
				// add here the grasping simulation

				ros::spinOnce();
				loop_rate.sleep();
			} while (!close_enough);
		}
		return true;
	}
	
	bool UR5::trajectory_without_object(Move_trajectory &trajectory) {
	
		double time_step = trajectory.get_time();
		ros::Rate loop_rate(1.0 / (time_step * 1.0));
		sensor_msgs::JointState joint_state;
		sensor_msgs::JointStateConstPtr msg;
		Eigen::Vector3d cur_position, new_position;
		Eigen::VectorXd q(6), new_q(6), vel(6), newVel(6), joint_vel_norm(6), grip_states, state;
		Eigen::Matrix4d dir, new_dir;
		Eigen::Matrix3d cur_orientation, new_orientation;
		bool close_enough;

		grip_states = get_gripper_states();
		state.resize(grip_states.size() + 6);

		q = get_joint_states();

		for (auto desired: trajectory.points) {
			Eigen::Vector3d desired_pos = desired.head(3);
			Eigen::Matrix3d desired_orientation = rpy2rotm(desired[3], desired[4], desired[5]);
			do {
				dir = direct(q);
				cur_orientation = Eigen::Matrix3d(dir.block(0, 0, 3, 3));
				cur_position << dir(0, 3), dir(1, 3), dir(2, 3);

				vel = get_vel(cur_position, desired_pos, cur_orientation, desired_orientation, time_step);

				joint_vel_norm = get_joint_velocities(jacobian(q), vel);
				new_q = q + joint_vel_norm * time_step;

				new_dir = direct(new_q);
				new_orientation = Eigen::Matrix3d(new_dir.block(0, 0, 3, 3));
				new_position << new_dir(0, 3), new_dir(1, 3), new_dir(2, 3);

				newVel = get_vel(new_position, desired_pos, new_orientation, desired_orientation, time_step);
				close_enough = newVel.norm() * time_step < 0.002;

				q = new_q;

				state << q, grip_states;
				actuate_ur5(state);
				
				// add here the grasping simulation

				ros::spinOnce();
				loop_rate.sleep();
			} while (!close_enough);
		}
		return true;
	}
	
	
	
	// move the robot with the grasped object to the right position
	
	bool UR5::move_to_position_with_object(std::string model_name, Eigen::Vector3d target, double final_yaw, Stay_away_from &obstacle, std::vector <Eigen::Vector2d> &obstacle_poses, double time){
		
		double timeStep = 0.001;
		Eigen::VectorXd cur_position;
		Eigen::Vector3d start;
		double starting_yaw;
		Move_trajectory trajectory;

		cur_position = get_position();
		start = cur_position.head(3);
		starting_yaw = M_PI - cur_position[5];

		trajectory = Move_trajectory(start, target, starting_yaw, M_PI - final_yaw, obstacle, obstacle_poses,
										  time, timeStep);
		return trajectory_with_object(model_name, trajectory);	
	
	}
	
	// use the same trajectory for both the robot and for the object
							 
	bool UR5::trajectory_with_object(std::string model_name, Move_trajectory &trajectory){
		
		double time_step = trajectory.get_time();
			ros::Rate loop_rate(1.0 / (time_step * 1.0));
			sensor_msgs::JointState joint_state;
			sensor_msgs::JointStateConstPtr msg;
			Eigen::Vector3d cur_position, new_position;
			Eigen::VectorXd q(6), new_q(6), vel(6), newVel(6), joint_vel_norm(6), grip_states, state;
			Eigen::Matrix4d dir, new_dir;
			Eigen::Matrix3d cur_orientation, new_orientation;
			bool close_enough;

			grip_states = get_gripper_states();
			state.resize(grip_states.size() + 6);

			q = get_joint_states();

			for (auto desired: trajectory.points) {
				Eigen::Vector3d desired_pos = desired.head(3);
				Eigen::Matrix3d desired_orientation = rpy2rotm(desired[3], desired[4], desired[5]);
				do {
					dir = direct(q);
					cur_orientation = Eigen::Matrix3d(dir.block(0, 0, 3, 3));
					cur_position << dir(0, 3), dir(1, 3), dir(2, 3);

					vel = get_vel(cur_position, desired_pos, cur_orientation, desired_orientation, time_step);

					joint_vel_norm = get_joint_velocities(jacobian(q), vel);
					new_q = q + joint_vel_norm * time_step;

					new_dir = direct(new_q);
					new_orientation = Eigen::Matrix3d(new_dir.block(0, 0, 3, 3));
					new_position << new_dir(0, 3), new_dir(1, 3), new_dir(2, 3);

					newVel = get_vel(new_position, desired_pos, new_orientation, desired_orientation, time_step);
					close_enough = newVel.norm() * time_step < 0.002;

					q = new_q;

					state << q, grip_states;
					actuate_ur5(state);
					
								
					// moving also the object by the same trajectory
					
					client = node -> serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

					gazebo_msgs::SetModelState modelstate;
					modelstate.request.model_state.model_name = model_name;
					modelstate.request.model_state.pose.position.x = - desired_pos.x() + 1.000;
					modelstate.request.model_state.pose.position.y = - desired_pos.y() + 0.800;
					modelstate.request.model_state.pose.position.z = desired_pos.z() + 0.86;

					Eigen::Quaterniond tmp(desired_orientation);

					modelstate.request.model_state.pose.orientation.x = tmp.x();
					modelstate.request.model_state.pose.orientation.y = tmp.y();
					modelstate.request.model_state.pose.orientation.z = tmp.z();
					modelstate.request.model_state.pose.orientation.w = tmp.w();
					
					
					if(client.call(modelstate)){
						ROS_INFO("Successfully moved the object %s \n", model_name.c_str());
					} else{
						ROS_ERROR("Object does not move \n");
					}

					//ros spin me round
					
					ros::spinOnce();
					loop_rate.sleep();
				} while (!close_enough);
			}
			return true;
	
	}
	
	bool UR5::trajectory_with_object(std::string model_name, Move_linear &trajectory){
		
		double time_step = trajectory.get_time();
			ros::Rate loop_rate(1.0 / (time_step * 1.0));
			sensor_msgs::JointState joint_state;
			sensor_msgs::JointStateConstPtr msg;
			Eigen::Vector3d cur_position, new_position;
			Eigen::VectorXd q(6), new_q(6), vel(6), newVel(6), joint_vel_norm(6), grip_states, state;
			Eigen::Matrix4d dir, new_dir;
			Eigen::Matrix3d cur_orientation, new_orientation;
			bool close_enough;

			grip_states = get_gripper_states();
			state.resize(grip_states.size() + 6);

			q = get_joint_states();

			for (auto desired: trajectory.points) {
				Eigen::Vector3d desired_pos = desired.head(3);
				Eigen::Matrix3d desired_orientation = rpy2rotm(desired[3], desired[4], desired[5]);
				do {
					dir = direct(q);
					cur_orientation = Eigen::Matrix3d(dir.block(0, 0, 3, 3));
					cur_position << dir(0, 3), dir(1, 3), dir(2, 3);

					vel = get_vel(cur_position, desired_pos, cur_orientation, desired_orientation, time_step);

					joint_vel_norm = get_joint_velocities(jacobian(q), vel);
					new_q = q + joint_vel_norm * time_step;

					new_dir = direct(new_q);
					new_orientation = Eigen::Matrix3d(new_dir.block(0, 0, 3, 3));
					new_position << new_dir(0, 3), new_dir(1, 3), new_dir(2, 3);

					newVel = get_vel(new_position, desired_pos, new_orientation, desired_orientation, time_step);
					close_enough = newVel.norm() * time_step < 0.002;

					q = new_q;

					state << q, grip_states;
					actuate_ur5(state);
					
								
					// moving also the object by the same trajectory
					
					client = node -> serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

					gazebo_msgs::SetModelState modelstate;
					modelstate.request.model_state.model_name = model_name;
					modelstate.request.model_state.pose.position.x = - desired_pos.x() + 1.000;
					modelstate.request.model_state.pose.position.y = - desired_pos.y() + 0.800;
					modelstate.request.model_state.pose.position.z = desired_pos.z() + 0.86;

					Eigen::Quaterniond tmp(desired_orientation);

					modelstate.request.model_state.pose.orientation.x = tmp.x();
					modelstate.request.model_state.pose.orientation.y = tmp.y();
					modelstate.request.model_state.pose.orientation.z = tmp.z();
					modelstate.request.model_state.pose.orientation.w = tmp.w();
					
					
					if(client.call(modelstate)){
						ROS_INFO("Successfully moved the object %s \n", model_name.c_str());
					} else{
						ROS_ERROR("Object does not move \n");
					}

					//ros spin me round
					
					ros::spinOnce();
					loop_rate.sleep();
				} while (!close_enough);
			}
			return true;
	
	}

    //get current joint positions of the robot 
	Eigen::VectorXd UR5::get_joint_states() {
		sensor_msgs::JointState joint_state;
		sensor_msgs::JointStateConstPtr msg;
		Eigen::VectorXd q(6);
		msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", *node);
		if (msg == NULL) {
			ROS_INFO("No message received");
		} else {
			joint_state = *msg;
		}
		if (joint_state.position.size() == 6) {
			q <<	joint_state.position[2], joint_state.position[1], joint_state.position[0],
					joint_state.position[3], joint_state.position[4], joint_state.position[5];
		} else if (joint_state.position.size() == 8) {
			q <<	joint_state.position[4], joint_state.position[3], joint_state.position[0],
					joint_state.position[5], joint_state.position[6], joint_state.position[7];
		}
		return q;
	}
    
    //get current position of the robot,
	Eigen::VectorXd UR5::get_position() {
		Eigen::Matrix4d dir = direct(get_joint_states());
		Eigen::Vector3d rpy = rotm2rpy(dir.block(0, 0, 3, 3));
		Eigen::VectorXd position(6);
		position << dir(0, 3), dir(1, 3), dir(2, 3), rpy;
		return position;
	}

    //get current joint positions of the robot
	Eigen::VectorXd UR5::get_gripper_states() {
		sensor_msgs::JointState joint_state;
		sensor_msgs::JointStateConstPtr msg;
		Eigen::VectorXd q;
		msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", *node);
		if (msg == NULL) {
			ROS_INFO("No joint state messages received");
		} else {
			joint_state = *msg;
		}
		if (joint_state.position.size() == 6) {
			q.resize(0);
		} else if (joint_state.position.size() == 8) {
			q.resize(2);
			q << joint_state.position[1], joint_state.position[2];
		}
		return q;
	}

    //calculate the forward kinematics of the UR5 robot given a set of joint angles
	Eigen::Matrix4d UR5::direct(Eigen::VectorXd q) {
		double th1 = q[0];
		double th2 = q[1];
		double th3 = q[2];
		double th4 = q[3];
		double th5 = q[4];
		double th6 = q[5];
		Eigen::Vector3d position;

		Eigen::Matrix4d transformationB6 = transformation01(th1) * transformation12(th2) * transformation23(th3) * transformation34(th4) * transformation45(th5) * transformation56(th6);
		transformationB6(1, 0) = -transformationB6(1, 0);
		transformationB6(0, 1) = -transformationB6(0, 1);
		transformationB6(1, 2) = -transformationB6(1, 2);
		transformationB6(2, 1) = -transformationB6(2, 1);
		transformationB6(0, 3) = -transformationB6(0, 3) + 0.5;
		transformationB6(1, 3) = transformationB6(1, 3) + 0.45;
		transformationB6(2, 3) = -transformationB6(2, 3) + 0.88;
		return transformationB6;
	}

    //transformation matrix 0->1
	Eigen::Matrix4d UR5::transformation01(double th) {
		Eigen::Matrix4d t;
		t << cos(th), -sin(th), 0.0, 0.0,
				sin(th), cos(th), 0.0, 0.0,
				0.0, 0.0, 1.0, d[0],
				0.0, 0.0, 0.0, 1.0;
		return t;
	}

    //transformation matrix 1->2
	Eigen::Matrix4d UR5::transformation12(double th) {
		Eigen::Matrix4d t;
		t << cos(th), -sin(th), 0.0, 0.0,
				0.0, 0.0, -1.0, 0.0,
				sin(th), cos(th), 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0;
		return t;
	}

    //transformation matrix 2->3
	Eigen::Matrix4d UR5::transformation23(double th) {
		Eigen::Matrix4d t;
		t << cos(th), -sin(th), 0.0, a[1],
				sin(th), cos(th), 0.0, 0.0,
				0.0, 0.0, 1.0, d[2],
				0.0, 0.0, 0.0, 1.0;
		return t;
	}

    //transformation matrix 3->4
	Eigen::Matrix4d UR5::transformation34(double th) {
		Eigen::Matrix4d t;
		t << cos(th), -sin(th), 0.0, a[2],
				sin(th), cos(th), 0.0, 0.0,
				0.0, 0.0, 1.0, d[3],
				0.0, 0.0, 0.0, 1.0;
		return t;
	}

    //transformation matrix 4->5
	Eigen::Matrix4d UR5::transformation45(double th) {
		Eigen::Matrix4d t;
		t << cos(th), -sin(th), 0.0, 0.0,
				0.0, 0.0, -1.0, -d[4],
				sin(th), cos(th), 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0;
		return t;
	}

    //transformation matrix 5->6
	Eigen::Matrix4d UR5::transformation56(double th) {
		Eigen::Matrix4d t;
		t << cos(th), -sin(th), 0.0, 0.0,
				0.0, 0.0, 1.0, d[5],
				-sin(th), -cos(th), 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0;
		return t;
	}

    //compute jacobian
	Eigen::MatrixXd UR5::jacobian(Eigen::VectorXd q) {
		double th1 = q[0];
		double th2 = q[1];
		double th3 = q[2];
		double th4 = q[3];
		double th5 = q[4];
		double th6 = q[5];

		double A1 = a[0];
		double A2 = a[1];
		double A3 = a[2];
		double A4 = a[3];
		double A5 = a[4];
		double A6 = a[5];

		double D1 = d[0];
		double D2 = d[1];
		double D3 = d[2];
		double D4 = d[3];
		double D5 = d[4];
		double D6 = d[5];

		double j1[6] = {
				D5 * (cos(th1) * cos(th5) + cos(th2 + th3 + th4) * sin(th1) * sin(th5)) + D3 * cos(th1) +
				D4 * cos(th1) - A3 * cos(th2 + th3) * sin(th1) - A2 * cos(th2) * sin(th1) -
				D5 * sin(th2 + th3 + th4) * sin(th1),
				D5 * (cos(th5) * sin(th1) - cos(th2 + th3 + th4) * cos(th1) * sin(th5)) + D3 * sin(th1) +
				D4 * sin(th1) + A3 * cos(th2 + th3) * cos(th1) + A2 * cos(th1) * cos(th2) +
				D5 * sin(th2 + th3 + th4) * cos(th1),
				0.0,
				0.0,
				0.0,
				1.0
		};

		double j2[6] = {
				-cos(th1) *
				(A3 * sin(th2 + th3) + A2 * sin(th2) + D5 * (sin(th2 + th3) * sin(th4) - cos(th2 + th3) * cos(th4)) -
				 D5 * sin(th5) * (cos(th2 + th3) * sin(th4) + sin(th2 + th3) * cos(th4))),
				-sin(th1) *
				(A3 * sin(th2 + th3) + A2 * sin(th2) + D5 * (sin(th2 + th3) * sin(th4) - cos(th2 + th3) * cos(th4)) -
				 D5 * sin(th5) * (cos(th2 + th3) * sin(th4) + sin(th2 + th3) * cos(th4))),
				A3 * cos(th2 + th3) - (D5 * sin(th2 + th3 + th4 + th5)) / 2 + A2 * cos(th2) +
				(D5 * sin(th2 + th3 + th4 - th5)) / 2 + D5 * sin(th2 + th3 + th4),
				sin(th1),
				-cos(th1),
				0.0
		};
		double j3[6] = {
				cos(th1) * (D5 * cos(th2 + th3 + th4) - A3 * sin(th2 + th3) + D5 * sin(th2 + th3 + th4) * sin(th5)),
				sin(th1) * (D5 * cos(th2 + th3 + th4) - A3 * sin(th2 + th3) + D5 * sin(th2 + th3 + th4) * sin(th5)),
				A3 * cos(th2 + th3) - (D5 * sin(th2 + th3 + th4 + th5)) / 2 + (D5 * sin(th2 + th3 + th4 - th5)) / 2 +
				D5 * sin(th2 + th3 + th4),
				sin(th1),
				-cos(th1),
				0.0
		};
		double j4[6] = {
				D5 * cos(th1) * (cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * sin(th5)),
				D5 * sin(th1) * (cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * sin(th5)),
				D5 * (sin(th2 + th3 + th4 - th5) / 2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5) / 2),
				sin(th1),
				-cos(th1),
				0.0
		};
		double j5[6] = {
				-D5 * sin(th1) * sin(th5) - D5 * cos(th2 + th3 + th4) * cos(th1) * cos(th5),
				D5 * cos(th1) * sin(th5) - D5 * cos(th2 + th3 + th4) * cos(th5) * sin(th1),
				-D5 * (sin(th2 + th3 + th4 - th5) / 2 + sin(th2 + th3 + th4 + th5) / 2),
				sin(th2 + th3 + th4) * cos(th1),
				sin(th2 + th3 + th4) * sin(th1),
				-cos(th2 + th3 + th4)
		};
		double j6[6] = {
				0.0,
				0.0,
				0.0,
				cos(th5) * sin(th1) - cos(th2 + th3 + th4) * cos(th1) * sin(th5),
				-cos(th1) * cos(th5) - cos(th2 + th3 + th4) * sin(th1) * sin(th5),
				-sin(th2 + th3 + th4) * sin(th5)
		};

		Eigen::MatrixXd j(6, 6);
		j << -j1[0], -j2[0], -j3[0], -j4[0], -j5[0], -j6[0],
				+j1[1], +j2[1], +j3[1], +j4[1], +j5[1], +j6[1],
				-j1[2], -j2[2], -j3[2], -j4[2], -j5[2], -j6[2],
				-j1[3], -j2[3], -j3[3], -j4[3], -j5[3], -j6[3],
				+j1[4], +j2[4], +j3[4], +j4[4], +j5[4], +j6[4],
				-j1[5], -j2[5], -j3[5], -j4[5], -j5[5], -j6[5];
		return j;
	}
} // Robotics
