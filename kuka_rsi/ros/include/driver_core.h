/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) TODO FILL IN YEAR HERE \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   ROS package name: ipa_kuka_rsi
 *
 * \author
 *   Author: Joshua Hampp
 *
 * \date Date of creation: 06/21/2014
 *
 * \brief
 *   ROS driver for the KUKA RSI XML protocol
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

//msgs, srvs, ...
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>
#include <brics_actuator/JointVelocities.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <moveit_msgs/MoveGroupAction.h>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ipa_kuka_rsi/DynConfig.h>


namespace RSI {class RSI_Server;}

//! node for RSI driver
class RSINode {

	ros::NodeHandle n_;				///< create a handle for this node, initialize node
	
	ros::Publisher pub_jointstate_;	///< declaration of topics to publish (joint state)
	ros::Publisher pub_diag_;		///< declaration of topics to publish (diagnostics)

	ros::Subscriber sub_joint_command_;		///< subscriber for a single joint command
	ros::Subscriber sub_joint_path_command_;///< subscriber for a trajectory

	ros::ServiceServer srv_init_;		///< trigger service to init. driver
	ros::ServiceServer srv_recover_;	///< trigger service to reover driver
	ros::ServiceServer srv_stop_;		///< trigger service to stop motion
	ros::ServiceServer srv_motion_mode_;///< service to change motion mode
	//ros::ServiceServer srv_stop_motion, srv_joint_path_cmd_;
	
	dynamic_reconfigure::Server<ipa_kuka_rsi::DynConfig> dyn_rec_server_;

	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_joint_; 	///< action server to follow a trajectory (joint)
	actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> as_move_group_; 			///< action server to move to a pose (cart.)
	
	tf::TransformBroadcaster tf_broadcaster_;	///< transformation broadcaster for cartesian position
	tf::TransformListener tf_listener_;			///< TF listener for cart. movement

	bool auto_initialize_;		///< flag: was initialized
	bool initialized_;			///< flag: was initialized
	
	typedef enum {MM_CART=0, MM_AXIS=1} MOTION_MODE;
	MOTION_MODE mode_motion_;	///< flag: mode of controlled motion
	
	ros::Time safety_time_limit_;	///< if this time is non zero and in the past motions will be stopped
	
	// parameters
	std::vector<std::string> joint_names_;		///< names of each joint (also number of joints)
	std::vector<double> joint_limit_upper_;	///< max. limits for each joint in rad
	std::vector<double> joint_limit_lower_;	///< min. limits for each joint in rad
	int frequency_;					///< frequency to publish joint states and diagnostics
	double max_vel_;				///< max. velocity for joint movement
	double tolerance_;				///< tolerance in rad when goal counts as reached
	std::string tip_link_;			///< link name of tip of the arm
	std::string base_link_;			///< link name of the base of the arm
	std::string rsi_joint_name_;	///< name of the RSI property for setting joint angles (default: ASPos)
	std::string rsi_cart_name_;		///< name of the RSI property for setting cart. pose (default: RSol)
	
	//internals
	int current_movement_id_;		///< id of the current movement to prevent more than one trajectory at the same time
	std::string sR_[3];				///< name of cart. rotation (ABC)
	std::string sT_[3];				///< name of cart. translation (XYZ)
	double initial_values_[6+6];	/// initial values of axis and cart.
	boost::shared_ptr<RSI::RSI_Server> rsi_comm_;	///< network connection and parsing to RSI XML


	/// reorder a vector with a given list of indices, if not possible returns false
	template< class T >
	bool _reorder(std::vector<T> &v, std::vector<size_t> const &order )  {
		std::vector<T> cp(order.size());	//not efficient but we have only small vectors...
		for(size_t i=0; i<order.size(); i++)
			if(order[i]<v.size()) cp[i] = v[order[i]];
			else return false;
		v = cp;
		return true;
	}

	/// reorder trajectory message to match our ordering of joint names with validation
	bool reorderMsg(trajectory_msgs::JointTrajectory &traj);
	///reorder joint tolerance message to match our ordering of joint names with validation
	bool reorderMsg(std::vector<control_msgs::JointTolerance> &tol);
	
	/// internal helper function for setting the aim of single trajectory point (joint)
	bool setJointTrajectory(const trajectory_msgs::JointTrajectoryPoint &pt);
	/// internal helper function for setting the aim of single pose (cart.)
	bool setCartTrajectory(const Eigen::Affine3d &TT, Eigen::Vector3d *trans=NULL, Eigen::Vector3d *rpy=NULL);
	/// internal helper function to stop movement
	void _stop();
	/// internal saturation function for max./min. angles (limits)
	float saturate(float input, float min, float max);
	
public:
	RSINode();
	
	/// ROS callback for single joint command topic
	void cb_joint_command(const trajectory_msgs::JointTrajectoryPoint &jtp);
	/// ROS callback for the joint trajetory topic
	void cb_joint_path_command(trajectory_msgs::JointTrajectory jt);
	/// ROS callback for the joint trajetory action
	void cb_follow_joint_traj();
	/// ROS callback for the move group action to move to a cart. pose
	void cb_move_group();
	
	/// service callback to initialize driver and RSI connection
	bool cb_init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
	/// service callback to change motion mode
	bool cb_motion_mode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res);
	/// service callback to stop movement
	bool cb_stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
	/// service callback for recovering (not impelmented)
	bool cb_recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
	
	/// publishes the state of the driver (diagnostics, joints, tf)
	void publishState();
	
	/// getter for publishing frequency
	int getFrequency() const {return frequency_;}
	
	/// use this function to change motion mode while run time
	void setMotionMode(const MOTION_MODE mode);
	/// getter function
	MOTION_MODE getMotionMode() const {return mode_motion_;}
	
private:

	/// internal control mechanism to move to a cart. goal
	void _cb_move_group(moveit_msgs::MoveGroupGoalConstPtr goal);
	/// internal control mechanism to follow a trajectory
	bool follow_joint_traj(control_msgs::FollowJointTrajectoryGoal &goal, const bool action);
	
	/// dynamic reconfigure callback
	void cb_dyn_reconfig(ipa_kuka_rsi::DynConfig &config, uint32_t level);
};
