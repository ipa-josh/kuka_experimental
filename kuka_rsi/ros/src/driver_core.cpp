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
 *   driver for the KUKA RSI XML protocol
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
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

//msgs, srvs, ...
#include <cob_srvs/Trigger.h>
#include <brics_actuator/JointVelocities.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <moveit_msgs/MoveGroupAction.h>

//#include <industrial_msgs/CmdJointTrajectory.h>
//#include <industrial_msgs/StopMotion.h>

/*
follow joint 
http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec
http://docs.ros.org/hydro/api/moveit_msgs/html/action/MoveGroup.html
*/

//RSI protocol
#include <ipa_kuka_rsi/rsi_server.h>
#include <boost/lexical_cast.hpp>

#include "math.h"
#define RAD_TO_DEG(a) ((a)*180.0/M_PI)
#define DEG_TO_RAD(a) ((a)/180.0*M_PI)

/// where defined, configurable???
#define IPOC_HZ 250

class RSINode {
	/// create a handle for this node, initialize node
	ros::NodeHandle n_;
	
	/// declaration of topics to publish
	ros::Publisher pub_jointstate_;
	ros::Publisher pub_diag_;
	
	ros::Subscriber sub_joint_command_, sub_joint_path_command_;
	
	/// declaration of service servers
	ros::ServiceServer srv_init_, srv_recover_, srv_stop_;
	//ros::ServiceServer srv_stop_motion, srv_joint_path_cmd_;
	
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_joint_; 
	actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction> as_move_group_; 
	tf::TransformListener tf_listener_;
	
	bool initialized_;
	std::map<std::string, std::string> rsi2joint_, joint2rsi_;
	int frequency_;
	std::vector<std::string> joint_names_;
	std::vector<double> joint_limit_upper_, joint_limit_lower_;
	double max_vel_, tolerance_;
	std::string tip_link_, base_link_, rsi_joint_name_, rsi_cart_name_;
	int current_movement_id_;
	
	std::string sR_[3], sT_[3];
	
	boost::shared_ptr<RSI::RSI_Server> rsi_comm_;
	
	template< class T >
	bool _reorder(std::vector<T> &v, std::vector<size_t> const &order )  {
		std::vector<T> cp(order.size());	//not efficient but we have only small vectors...
		for(size_t i=0; i<order.size(); i++)
			if(order[i]<v.size()) cp[i] = v[order[i]];
			else return false;
		v = cp;
		return true;
	}

	///reorder trajectory message to match our ordering of joint names
	bool reorderMsg(trajectory_msgs::JointTrajectory &traj) {
		//build map
		std::vector<size_t> int2ext(joint_names_.size());
		size_t found=0;
		for(size_t j=0; j<joint_names_.size(); j++) {
			for(size_t i=0; i<traj.joint_names.size(); i++)
				if(joint_names_[j]==traj.joint_names[i]) {
					int2ext[j] = i;
					++found;
					break;
				}
			if(found!=j+1) ROS_INFO("missing joint name: %s", joint_names_[j].c_str());
		}
				
		if(found!=joint_names_.size()) {
			ROS_ERROR("missing joint names in trajectory");
			return false;
		}
		
		bool r = true;
		for(size_t i=0; i<traj.points.size(); i++) {
			r &= _reorder(traj.points[i].positions, int2ext);
			_reorder(traj.points[i].velocities, int2ext);
			_reorder(traj.points[i].accelerations, int2ext);
			
			if(traj.points[i].positions.size()!=found)
				r = false;
		}
		
		return r;
	}
	///reorder trajectory message to match our ordering of joint names
	bool reorderMsg(std::vector<control_msgs::JointTolerance> &tol) {
		//build map
		std::vector<size_t> int2ext(joint_names_.size());
		size_t found=0;
		for(size_t j=0; j<joint_names_.size(); j++)
			for(size_t i=0; i<tol.size(); i++)
				if(joint_names_[j]==tol[i].name) {
					int2ext[j] = i;
					++found;
					break;
				}

		if(found!=joint_names_.size()) {
			ROS_ERROR("missing joint names in trajectory");
			return false;
		}
		
		return _reorder(tol, int2ext) && (tol.size()==found);
	}
	
	bool setJointTrajectory(const trajectory_msgs::JointTrajectoryPoint &pt) {
		ROS_ASSERT(pt.positions.size() == joint_names_.size());
		
		for(size_t i=0; i<joint_names_.size(); i++) {
			const double angle = RAD_TO_DEG(pt.positions[i]);
			if(angle<joint_limit_lower_[i] || angle>joint_limit_upper_[i]) {
				ROS_ERROR("joint %s(%d) outside of soft limit: %f<%f<%f [rad]", joint_names_[i].c_str(), (int)i, joint_limit_lower_[i], angle, joint_limit_upper_[i]);
				return false;
			}
		}
		
		rsi_comm_->setStopped(false);
		for(size_t i=0; i<joint_names_.size(); i++) {
			const bool ret = rsi_comm_->set(rsi_joint_name_+".A"+boost::lexical_cast<std::string>(i+1), RAD_TO_DEG(pt.positions[i]) );
			ROS_ASSERT(ret);
			if(!ret) {
				ROS_ERROR("internal config problem: could not set property");
				rsi_comm_->setStopped(true);
				return false;
			}
		}
		
		return true;
	}
	
	bool setCartTrajectory(const Eigen::Affine3d &TT, Eigen::Vector3d *trans=NULL, Eigen::Vector3d *rpy=NULL) {
		Eigen::Vector3d _rpy; 
		_rpy(0) = atan2( TT(2,1), TT(2,2) );
		_rpy(1) = atan2( -TT(2,0), sqrt( TT(2,1)*TT(2,1) + TT(2,2)*TT(2,2) )  );
		_rpy(2) = atan2( TT(1,0), TT(0,0) );
		
		rsi_comm_->setStopped(false);
		for(size_t i=0; i<3; i++) {
			bool ret = rsi_comm_->set(rsi_cart_name_+"."+sR_[i], _rpy(i) );
			ROS_ASSERT(ret);
			ret &= rsi_comm_->set(rsi_cart_name_+"."+sT_[i], TT.translation()(i) );
			ROS_ASSERT(ret);
			
			if(!ret) {
				ROS_ERROR("internal config problem: could not set property");
				rsi_comm_->setStopped(true);
				return false;
			}
		}
		
		if(trans) *trans = TT.translation();
		if(rpy) *rpy = _rpy;
		
		return true;
	}
	
	void _stop()
	{
		rsi_comm_->setStopped(true);
		current_movement_id_++;
	}
	
public:
	RSINode():
		as_joint_("follow_joint_trajectory", false),
		as_move_group_("move_cartesian", false),
		initialized_(false),
		current_movement_id_(0)
	{
		//mapping between rpy/xyz to kuka naming
		sR_[0] = "A";
		sR_[1] = "B";
		sR_[2] = "C";
		
		sT_[0] = "X";
		sT_[1] = "Y";
		sT_[2] = "Z";
		
		//now let's start with ROS stuff
		n_ = ros::NodeHandle("~");
		
		pub_diag_ = n_.advertise<diagnostic_msgs::DiagnosticArray> ("diagnostics", 1);
		pub_jointstate_ = n_.advertise<sensor_msgs::JointState> ("joint_states", 1);
		
		srv_init_ = n_.advertiseService("init", &RSINode::cb_init, this);
		srv_recover_ = n_.advertiseService("recover", &RSINode::cb_recover, this);
		srv_stop_ = n_.advertiseService("stop", &RSINode::cb_stop, this);
		//srv_stop_motion = n_.advertiseService("stop_motion", &RSINode::cb_stop_motion, this);
		//srv_joint_path_cmd_ = n_.advertiseService("joint_path_command", &RSINode::cb_joint_path_command, this);
		
		as_joint_.registerGoalCallback(boost::bind(&RSINode::cb_follow_joint_traj, this));
		as_joint_.start();
		
		as_move_group_.registerGoalCallback(boost::bind(&RSINode::cb_move_group, this));
		as_move_group_.start();
		
		n_.param<bool>("initialized", initialized_, false);
		n_.param<int>("frequency", frequency_, 30);
		n_.param<double>("max_velocity", max_vel_, 0.1);
		n_.param<double>("tolerance", tolerance_, 0.01);
		n_.param<std::string>("RSI_joint_name", rsi_joint_name_, "ASPos");
		n_.param<std::string>("RSI_cart_name", rsi_cart_name_, "RSol");
		
		if(initialized_)
			ROS_INFO("initialized parameter is set so no init call is necessary");
		
		/// Get joint names
		XmlRpc::XmlRpcValue JointNamesXmlRpc;
		if (n_.hasParam("joint_names")) {
			n_.getParam("joint_names", JointNamesXmlRpc);
			
			joint_names_.resize(JointNamesXmlRpc.size());
			for (int i = 0; i < JointNamesXmlRpc.size(); i++)
				joint_names_[i] = (std::string) JointNamesXmlRpc[i];
		} else {
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
		}
		
		for(size_t i=0; i<joint_names_.size(); i++) {
			joint_limit_lower_.push_back(-std::numeric_limits<double>::max());
			joint_limit_upper_.push_back( std::numeric_limits<double>::max());
		}
		
		if (n_.hasParam("joint_limits")) {
			n_.getParam("joint_limits", JointNamesXmlRpc);
			
			ROS_ASSERT(JointNamesXmlRpc.size()<=(int)joint_names_.size());
			for (int i = 0; i < std::min((int)JointNamesXmlRpc.size(), (int)joint_names_.size()); i++) {
				ROS_ASSERT(JointNamesXmlRpc[i].size()==2);
				joint_limit_lower_[i] = (double) JointNamesXmlRpc[i][0];
				joint_limit_upper_[i] = (double) JointNamesXmlRpc[i][1];
			}
		} else ROS_WARN("Parameter joint_limits not set --> NO SOFT LIMITS!");
		
		if (n_.hasParam("tip_link")) 
			n_.getParam("tip_link", tip_link_);
		else ROS_WARN("Parameter tip_link is not set --> cartesian movement is proabably not working");
		
		if (n_.hasParam("base_link")) 
			n_.getParam("base_link", base_link_);
		else ROS_WARN("Parameter base_link is not set --> cartesian movement is proabably not working");
		
		if(n_.hasParam("config")) {
			std::string fn;
			n_.getParam("config", fn);
			ROS_DEBUG("using config file: %s", fn.c_str());
			rsi_comm_.reset(new RSI::RSI_Server(fn));
		} else
			ROS_ERROR("parameter for config file (e.g. ERXConfig.xml) was not specified\naborting...");
			
		sub_joint_command_ = n_.subscribe("joint_command", 1, &RSINode::cb_joint_command, this);
		sub_joint_path_command_ = n_.subscribe("joint_path_command", 1, &RSINode::cb_joint_path_command, this);
	}
	
	void cb_joint_command(const trajectory_msgs::JointTrajectoryPoint &jtp)
	{
		if(jtp.positions.size()!=joint_names_.size()) {
			ROS_ERROR("incorrent message as size of pos. and joint names are incoisistent");
			return;
		}
		
		if(!setJointTrajectory(jtp))
			ROS_ERROR("failed to set JointTrajectoryPoint");
	}
	
	void cb_joint_path_command(trajectory_msgs::JointTrajectory jt)
	{
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = jt;
		control_msgs::JointTolerance tol;
		tol.position = tolerance_;
		tol.velocity = 0;
		tol.acceleration = 0;
		goal.path_tolerance.resize(joint_names_.size(), tol);
		goal.goal_tolerance.resize(joint_names_.size(), tol);
		goal.goal_time_tolerance = ros::Duration(10.);
		
		follow_joint_traj(goal, false);
	}
	
	void cb_follow_joint_traj()
	{
		control_msgs::FollowJointTrajectoryGoalConstPtr goal_ptr = as_joint_.acceptNewGoal();
		if(!goal_ptr) {
			ROS_ERROR("missing goal");
			return;
		}
		
		control_msgs::FollowJointTrajectoryGoal goal = *goal_ptr;
		new boost::thread(&RSINode::follow_joint_traj, this, goal, true);
	}
	
	bool follow_joint_traj(control_msgs::FollowJointTrajectoryGoal &goal, const bool action) {
		bool wait=false;
		size_t pos=0;
		control_msgs::FollowJointTrajectoryFeedback feed;
		ros::Rate r(getFrequency());
		
		feed.header = goal.trajectory.header;
		feed.joint_names = joint_names_;
		feed.actual.positions.resize(joint_names_.size());
		feed.error.positions.resize(joint_names_.size());
		
		control_msgs::FollowJointTrajectoryResult result;
		if(!reorderMsg(goal.trajectory)) {
			result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
			if(action) as_joint_.setSucceeded(result);
			ROS_ERROR("malformed goal (trajectory)");
			return false;
		}
		if(!reorderMsg(goal.goal_tolerance)) {
			result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
			if(action) as_joint_.setSucceeded(result);
			ROS_ERROR("malformed goal (tolerance)");
			return false;
		}
		
		int id = ++current_movement_id_;
		while( current_movement_id_==id && (!action || (!as_joint_.isPreemptRequested())) && pos<goal.trajectory.points.size()) {
			
			if(!wait) {
				if(	goal.trajectory.points[pos].positions.size()!=goal.goal_tolerance.size() ||
					joint_names_.size()!=goal.goal_tolerance.size() ) {
					ROS_ERROR("mismatching size of control points %d vs. %d vs. %d",
						(int)goal.trajectory.points[pos].positions.size(), (int)goal.goal_tolerance.size(), (int)joint_names_.size());
					break;
				}
				if(!setJointTrajectory(goal.trajectory.points[pos]))
					break;
				wait=true;
			}
			
			bool reached = true;
			for(size_t i=0; i<joint_names_.size(); i++) {
				double v;
				const bool ret = rsi_comm_->get("AIPos.A"+boost::lexical_cast<std::string>(i+1), v);
				v = DEG_TO_RAD(v);
				ROS_ASSERT(ret);
				
				feed.actual.positions[i] = v;
				feed.error.positions[i] = v-goal.trajectory.points[pos].positions[i];
				
				if( std::abs(v-goal.trajectory.points[pos].positions[i]) > goal.goal_tolerance[i].position )
				{
					reached=false;
					break;
				}
			}
			
			if(reached) {
				++pos;
				wait=false;
			}
			else {
				feed.header.stamp = ros::Time::now();
				feed.desired = goal.trajectory.points[pos];
				if(action) as_joint_.publishFeedback(feed);
				r.sleep();
			}
		}
		
		ROS_DEBUG("finished movement (joint)");
			
		if(pos<goal.trajectory.points.size())
			result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
		if(action) as_joint_.setSucceeded(result);
		
		return pos==goal.trajectory.points.size();
	}
	
	void cb_move_group()
	{
		moveit_msgs::MoveGroupGoalConstPtr goal = as_move_group_.acceptNewGoal();
		
		if(!goal) {
			ROS_INFO("missing goal");
			return;
		}
		
		new boost::thread(&RSINode::_cb_move_group, this, goal);
	}
	
	void _cb_move_group(moveit_msgs::MoveGroupGoalConstPtr goal)
	{
		Eigen::Vector3d aimT, aimRPY;
		bool wait=false;
		size_t pos=0;
		ros::Rate r(getFrequency());
		
		ROS_INFO("points %d", (int)goal->request.goal_constraints.size());
		while(as_move_group_.isActive() && !as_move_group_.isPreemptRequested() && pos<goal->request.goal_constraints.size()) {
			
			if(!wait) {
				if(	goal->request.goal_constraints[pos].position_constraints.size()!=1 ||
					goal->request.goal_constraints[pos].orientation_constraints.size()!=1 ||
					goal->request.goal_constraints[pos].position_constraints[0].link_name!=tip_link_ ||
					goal->request.goal_constraints[pos].orientation_constraints[0].link_name!=tip_link_ ||
					goal->request.goal_constraints[pos].position_constraints[0].header.frame_id!=goal->request.goal_constraints[pos].orientation_constraints[0].header.frame_id) {
					ROS_ERROR("invalid goal constraint, sould be: num %d=1 %d=1 and link_name %s=%s %s=%s frame_id %s=%s",
						(int)goal->request.goal_constraints[pos].position_constraints.size(),
						(int)goal->request.goal_constraints[pos].orientation_constraints.size(),
						goal->request.goal_constraints[pos].position_constraints[0].link_name.c_str(),
						tip_link_.c_str(),
						goal->request.goal_constraints[pos].orientation_constraints[0].link_name.c_str(),
						tip_link_.c_str(),
						goal->request.goal_constraints[pos].position_constraints[0].header.frame_id.c_str(),
						goal->request.goal_constraints[pos].orientation_constraints[0].header.frame_id.c_str()
					);
					break;
				}
				
				tf::StampedTransform transform;
				try{
				  tf_listener_.lookupTransform(base_link_, goal->request.goal_constraints[pos].position_constraints[0].header.frame_id,
										   goal->request.goal_constraints[pos].position_constraints[0].header.stamp, transform);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("%s",ex.what());
				  break;
				}
				
				Eigen::Affine3d aff;
				Eigen::Vector3d off;
				Eigen::Quaterniond rot;
				tf::transformTFToEigen(transform, aff);
				tf::vectorMsgToEigen(goal->request.goal_constraints[pos].position_constraints[0].target_point_offset, off);
				tf::quaternionMsgToEigen(goal->request.goal_constraints[pos].orientation_constraints[0].orientation, rot);
				
				aff = Eigen::Translation3d(off)*rot*aff;
				
				if(!setCartTrajectory(aff, &aimT, &aimRPY))
					break;
				wait=true;
			}
			
			bool reached = true;
			Eigen::Vector3d off, rot;
			for(size_t i=0; i<3; i++) {
				const bool ret1 = rsi_comm_->get("RIst."+sT_[i], off(i));
				ROS_ASSERT(ret1);
				const bool ret2 = rsi_comm_->get("RIst."+sR_[i], rot(i));
				ROS_ASSERT(ret2);
			}
			
			//assuming RSI will output the same RPY for the desired position (ignoring singularity?)
			if( (off-aimT).squaredNorm()<goal->request.goal_constraints[pos].position_constraints[0].weight &&
				(off-aimRPY).squaredNorm()<goal->request.goal_constraints[pos].orientation_constraints[0].weight )
				reached = false;
			
			if(reached) {
				++pos;
				wait=false;
			}
			else
				r.sleep();
		}
		
		ROS_DEBUG("finished movement (cart.)");
			
		moveit_msgs::MoveGroupResult result;
		result.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
		if(pos<goal->request.goal_constraints.size()) 
			result.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
		as_move_group_.setSucceeded(result);
	}
	
	/*!
	 * \brief Executes the service callback for init.
	 *
	 * Connects to the hardware and initialized it.
	 * \param req Service request
	 * \param res Service response
	 */
	bool cb_init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
		if (!initialized_)
		{
			ROS_INFO("Initializing RSI...");

			if(!rsi_comm_ || !rsi_comm_->received_something())
			{
				res.success.data = false;
				res.error_message.data = "UDP Connection failed (RSI Init)";
				return false;
			}

			initialized_ = true;
			res.success.data = true;
			res.error_message.data = "RSI initialized successfully";
		} else {
			res.success.data = true;
			res.error_message.data = "RSI already initialized";
			ROS_WARN("...initializing RSI not successful. error: %s",res.error_message.data.c_str());
		}
		return true;
	}
	
	/*bool cb_stop_motion(industrial_msgs::CmdJointTrajectory::Request &req, industrial_msgs::CmdJointTrajectory::Response &res) {
		_stop();
		res.code = industrial_msgs::ServiceReturnCode::SUCCESS;
		return true;
	}
	
	bool cb_joint_path_cmd(industrial_msgs::StopMotion::Request &req, industrial_msgs::StopMotion::Response &res) {
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory = req.trajectory;
		control_msgs::JointTolerance tol;
		tol.position = path_tolerance_;
		tol.velocity = 0;
		tol.acceleration = 0;
		goal.path_tolerance.resize(joint_names_.size(), tol);
		
		res.code = industrial_msgs::ServiceReturnCode::FAILURE;
		if(follow_joint_traj(goal, false))
			res.code = industrial_msgs::ServiceReturnCode::SUCCESS;
		return true;
	}*/
	
	bool cb_stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
		_stop();
		res.success.data = true;
		res.error_message.data = "stopped motion";
		return true;
	}
	
	bool cb_recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
		res.success.data = false;
		res.error_message.data = "RSI cannot be recovered";
		return true;
	}
	
	float saturate(float input, float min, float max)
	{
		if ((max < min) || (min > max))
		{
			ROS_WARN("Output value saturation seems erroneous: min: %f, max: %f", min, max);
		}

		float output = input;
		if (output > max) 
		{
			output = max;
		} 
		else if (output < min) 
		{
			output = min;
		}
		return output;
	}

	/*!
	 * \brief Executes the callback from the command_vel topic.
	 *
	 * Set the current velocity target.
	 * \param msg JointVelocities
	 */
	void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg) {
		ROS_DEBUG("Received new velocity command");
		if (!initialized_||!rsi_comm_) {
			ROS_WARN("Skipping command: RSI not initialized");
			return;
		}

		// command velocities to RSI
		for(size_t i=0; i<joint_names_.size(); i++) {
			// Saturation for safety
			const bool ret = rsi_comm_->set("AK.A"+boost::lexical_cast<std::string>(i+1), saturate(RAD_TO_DEG(msg->velocities[i].value) / IPOC_HZ, -max_vel_, max_vel_) );
			ROS_ASSERT(ret);
		}
	}
	
	/*!
	 * \brief Publishes the state of the rsi as ros messages.
	 *
	 * Published to "/joint_states" as "sensor_msgs/JointState"
	 * Published to "state" as "pr2_controllers_msgs/JointTrajectoryState"
	 */
	void publishState() {
		if (initialized_ && rsi_comm_) {
			// get state from RSI
			std::vector<double> pos_vec(joint_names_.size());
			
			bool ok=true;
			for(size_t i=0; i<joint_names_.size(); i++) {
				const bool ret = rsi_comm_->get("AIPos.A"+boost::lexical_cast<std::string>(i+1), pos_vec[i]);
				pos_vec[i] = DEG_TO_RAD(pos_vec[i]);
				ok &= ret;
			}

			sensor_msgs::JointState joint_state_msg;
			joint_state_msg.header.stamp = ros::Time::now();
			joint_state_msg.name = joint_names_;
			joint_state_msg.position = pos_vec;

			if(ok)
				pub_jointstate_.publish(joint_state_msg);
			else
				ROS_WARN("not able to publish joint state");
		}

		// publishing diagnotic messages
		diagnostic_msgs::DiagnosticArray diagnostics;
		diagnostics.status.resize(1);

		// set data to diagnostics
		diagnostics.status[0].level = 2;
		diagnostics.status[0].name = n_.getNamespace();
		
		std::string estr;
		if(rsi_comm_) rsi_comm_->get("EStr", estr);

		if (!rsi_comm_) {
			diagnostics.status[0].message = "connection problem";
		} else if (!rsi_comm_->received_something()) {
			diagnostics.status[0].message = "connection did not receive anything yet";
		} else if (rsi_comm_->receivedAgo()>2.) {
			diagnostics.status[0].message = "connection did not receive anything within timeout threshold";
		} else if (estr.find("Error: ")==0) {
		} else if (initialized_) {
			diagnostics.status[0].level = 0;
			diagnostics.status[0].message = "RSI initialized and running";
		} else {
			diagnostics.status[0].level = 1;
			diagnostics.status[0].message = "RSI not initialized";
		}
		
		diagnostics.status[0].message += ": "+estr;
		
		// publish diagnostic message
		pub_diag_.publish(diagnostics);
	}
	
	int getFrequency() const {return frequency_;}
};

int main(int argc, char** argv) {
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "kuka_rsi");

	// create RSINode
	RSINode rsi_node;

	/// main loop
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	ros::Rate loop_rate(rsi_node.getFrequency()); // Hz
	while (ros::ok()) {
		rsi_node.publishState();
		loop_rate.sleep();
	}

	return 0;
}
