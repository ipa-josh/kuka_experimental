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
 
#include "driver_core.h"

//includes for ROS Industrial compatibility (needs catkin)
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
#include <boost/algorithm/string.hpp> 

#include "math.h"
#define RAD_TO_DEG(a) ((a)*180.0/M_PI)
double DEG_TO_RAD(double a) {
	while(a>  180.) a-=360.;
	while(a<=-180.) a+=360.;
	return (a)/180.0*M_PI;
}

/// where defined, configurable???
#define IPOC_HZ 250

bool RSINode::reorderMsg(trajectory_msgs::JointTrajectory &traj) {
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

bool RSINode::reorderMsg(std::vector<control_msgs::JointTolerance> &tol) {
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

bool RSINode::setJointTrajectory(const trajectory_msgs::JointTrajectoryPoint &pt) {
	if(!initialized_) {
		ROS_ERROR("Driver is not initialized yet!");
		return false;
	}
	
	if(mode_motion_!=MM_AXIS) {
		ROS_ERROR("Motion mode is not setup for joints!");
		return false;
	}

	if(pt.positions.size() != joint_names_.size()) {
		ROS_ERROR("number of joints does not match");
		return false;
	}
	
	safety_time_limit_ = ros::Time();
	
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
		if(!ret) {
			ROS_ERROR("internal config problem: could not set property");
			rsi_comm_->setStopped(true);
			return false;
		}
	}
		
	if(pt.time_from_start.toSec()>0)
		safety_time_limit_ = ros::Time::now() + pt.time_from_start;
	
	return true;
}

bool RSINode::setCartTrajectory(const Eigen::Affine3d &TT, Eigen::Vector3d *trans, Eigen::Vector3d *rpy) {
	if(!initialized_) {
		ROS_ERROR("Driver is not initialized yet!");
		return false;
	}
	
	if(mode_motion_!=MM_CART) {
		ROS_ERROR("Motion mode is not setup for Cartesian movment!");
		return false;
	}
	
	Eigen::Vector3d _rpy; 
	_rpy(0) = atan2( TT(2,1), TT(2,2) );
	_rpy(1) = atan2( -TT(2,0), sqrt( TT(2,1)*TT(2,1) + TT(2,2)*TT(2,2) )  );
	_rpy(2) = atan2( TT(1,0), TT(0,0) );
	
	std::cout<<"rpy\n"<<_rpy.transpose()<<std::endl;
	std::cout<<"tr\n"<<TT.translation().transpose()<<std::endl;
	
	rsi_comm_->setStopped(false);
	for(size_t i=0; i<3; i++) {
		bool ret = rsi_comm_->set(rsi_cart_name_+"."+sR_[i], RAD_TO_DEG(_rpy(i)) );
		ret &= rsi_comm_->set(rsi_cart_name_+"."+sT_[i], TT.translation()(i)*1000. );
		
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

void RSINode::_stop()
{
	rsi_comm_->setStopped(true);
	current_movement_id_++;
}

float RSINode::saturate(float input, float min, float max)
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

RSINode::RSINode():
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
	srv_motion_mode_ = n_.advertiseService("motion_mode", &RSINode::cb_motion_mode, this);
	//srv_stop_motion = n_.advertiseService("stop_motion", &RSINode::cb_stop_motion, this);
	//srv_joint_path_cmd_ = n_.advertiseService("joint_path_command", &RSINode::cb_joint_path_command, this);
	
	as_joint_.registerGoalCallback(boost::bind(&RSINode::cb_follow_joint_traj, this));
	as_joint_.start();
	
	as_move_group_.registerGoalCallback(boost::bind(&RSINode::cb_move_group, this));
	as_move_group_.start();
	
	n_.param<bool>("initialized", auto_initialize_, false);
	n_.param<int>("frequency", frequency_, 30);
	n_.param<double>("max_velocity", max_vel_, 0.1);
	n_.param<double>("tolerance", tolerance_, 0.01);
	n_.param<std::string>("RSI_joint_name", rsi_joint_name_, "ASPos");
	n_.param<std::string>("RSI_cart_name", rsi_cart_name_, "RSol");
	
	if(auto_initialize_)
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
	
	mode_motion_ = MM_AXIS;
	std::string mm;
	if (n_.hasParam("mode_motion")) {
		n_.getParam("mode_motion", mm);
		boost::algorithm::to_lower(mm);
		if(mm.find("cart")==0)
			mode_motion_ = MM_CART;
		else if(mm.find("ax")==0 || mm.find("joint")==0)
			mode_motion_ = MM_AXIS;
		else
			ROS_ERROR("Parameter mode_motion %s is not valid --> axis movement will be used", mm.c_str());
	}
	else ROS_ERROR("Parameter mode_motion is not set --> axis movement will be used");

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
	
	dynamic_reconfigure::Server<ipa_kuka_rsi::DynConfig>::CallbackType f;
	f = boost::bind(&RSINode::cb_dyn_reconfig, this, _1, _2);
	dyn_rec_server_.setCallback(f);
}

void RSINode::cb_joint_command(const trajectory_msgs::JointTrajectoryPoint &jtp)
{
	if(jtp.positions.size()!=joint_names_.size()) {
		ROS_ERROR("incorrent message as size of pos. and joint names are incoisistent");
		return;
	}
	
	if(!setJointTrajectory(jtp))
		ROS_ERROR("failed to set JointTrajectoryPoint");
}

void RSINode::cb_joint_path_command(trajectory_msgs::JointTrajectory jt)
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

void RSINode::cb_follow_joint_traj()
{
	control_msgs::FollowJointTrajectoryGoalConstPtr goal_ptr = as_joint_.acceptNewGoal();
	if(!goal_ptr) {
		ROS_ERROR("missing goal");
		return;
	}
	
	control_msgs::FollowJointTrajectoryGoal goal = *goal_ptr;
	new boost::thread(&RSINode::follow_joint_traj, this, goal, true);
}

void RSINode::cb_move_group()
{
	moveit_msgs::MoveGroupGoalConstPtr goal = as_move_group_.acceptNewGoal();
	
	if(!goal) {
		ROS_INFO("missing goal");
		return;
	}
	
	new boost::thread(&RSINode::_cb_move_group, this, goal);
}

bool RSINode::cb_init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
	if (!initialized_)
	{
		ROS_INFO("Initializing RSI...");

		if(!rsi_comm_ || !rsi_comm_->received_something())
		{
			res.success.data = false;
			res.error_message.data = "UDP Connection failed (RSI Init)";
			return false;
		}
		
		//setting initial values
		for(size_t i=0; i<joint_names_.size(); i++) {
			double v;
			if(rsi_comm_->get("AIPos.A"+boost::lexical_cast<std::string>(i+1), v))
				initial_values_[i] = v;
			else
				initial_values_[i] = 0;
		}
		for(size_t i=0; i<3; i++) {
			double v;
			if(rsi_comm_->get("RIst."+sT_[i], v))
				initial_values_[i+6] = v;
			else
				initial_values_[i+6] = 0;
				
			if(rsi_comm_->get("RIst."+sR_[i], v))
				initial_values_[i+9] = v;
			else
				initial_values_[i+9] = 0;
		}

		initialized_ = true;
		res.success.data = true;
		res.error_message.data = "RSI initialized successfully";
		
		ROS_INFO("%s", res.error_message.data.c_str());
	} else {
		res.success.data = true;
		res.error_message.data = "RSI already initialized";
		ROS_WARN("...initializing RSI not successful. error: %s",res.error_message.data.c_str());
	}
	return true;
}

/*bool RSINode::cb_stop_motion(industrial_msgs::CmdJointTrajectory::Request &req, industrial_msgs::CmdJointTrajectory::Response &res) {
	_stop();
	res.code = industrial_msgs::ServiceReturnCode::SUCCESS;
	return true;
}

bool RSINode::cb_joint_path_cmd(industrial_msgs::StopMotion::Request &req, industrial_msgs::StopMotion::Response &res) {
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

bool RSINode::cb_motion_mode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res) {
	res.success.data = true;		
	std::string mm = req.operation_mode.data;
	boost::algorithm::to_lower(mm);
	
	if(mm.find("cart")==0)
		setMotionMode( MM_CART );
	else if(mm.find("ax")==0 || mm.find("joint")==0)
		setMotionMode( MM_AXIS );
	else {
		res.success.data = false;
		res.error_message.data = "Parameter is not valid, allowed parameters: joint, cartesian";
		ROS_ERROR("Parameter '%s' is not valid", mm.c_str());
	}
	
	return true;
}

bool RSINode::cb_stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
	_stop();
	res.success.data = true;
	res.error_message.data = "stopped motion";
	return true;
}

bool RSINode::cb_recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res) {
	res.success.data = false;
	res.error_message.data = "RSI cannot be recovered";
	return true;
}

void RSINode::publishState() {
	if(!safety_time_limit_.isZero() && safety_time_limit_<=ros::Time::now())
		_stop();
	
	if(!initialized_ && auto_initialize_ && rsi_comm_) {
		cob_srvs::Trigger::Request  req;
		cob_srvs::Trigger::Response res;
		cb_init(req, res);
		if(!initialized_) ros::Duration(0.1).sleep();
	}
	
	if (initialized_ && rsi_comm_) {
		// get state from RSI
		std::vector<double> pos_vec(joint_names_.size()), torques;
		
		bool ok=true;
		for(size_t i=0; i<joint_names_.size(); i++) {
			const bool ret = rsi_comm_->get("AIPos.A"+boost::lexical_cast<std::string>(i+1), pos_vec[i]);
			pos_vec[i] = DEG_TO_RAD(pos_vec[i]);
			ok &= ret;
			
			double torque;
			if(rsi_comm_->get("ATorque"+boost::lexical_cast<std::string>(i+1), torque))
				torques.push_back(torque);
		}

		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.header.stamp = ros::Time::now();
		joint_state_msg.name = joint_names_;
		joint_state_msg.position = pos_vec;
		if(torques.size()==pos_vec.size()) //we got all
			joint_state_msg.effort = torques;

		if(ok)
			pub_jointstate_.publish(joint_state_msg);
		else
			ROS_WARN("not able to publish joint state (because 'AIPos' seems to be missing)");
			
		tf::Transform transform;
		bool cart = true;
		tf::Vector3 trans, rpy;
		for(size_t i=0; i<3; i++) {
			cart &= rsi_comm_->get("RIst."+sR_[i], rpy[i]  );
			cart &= rsi_comm_->get("RIst."+sT_[i], trans[i]);
			
			rpy[i] = DEG_TO_RAD(rpy[i]);
			trans[i] = trans[i]/1000.;
			
			if(!cart) break;
		}
		
		if(cart) {
			transform.setOrigin( trans );
			tf::Quaternion q;
			q.setRPY(rpy.x(), rpy.y(), rpy.z());
			transform.setRotation(q);
			
			tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_link_, tip_link_));
		}
	}

	// publishing diagnotic messages
	diagnostic_msgs::DiagnosticArray diagnostics;
	diagnostics.status.resize(1);

	// set data to diagnostics
	diagnostics.status[0].level = 2;
	diagnostics.status[0].name = n_.getNamespace();
	
	std::string estr;
	if(rsi_comm_) {
		rsi_comm_->get("EStr", estr);
		int64_t status;
		if(rsi_comm_->get("AStatus", status)) {
			if(status==1) estr+="\nAxis Control in motion";
			else if(status==0) estr+="\nAxis Control not in motion";
			else estr+="\nError: Axis Control in limitation";
		}
		if(rsi_comm_->get("RStatus", status)) {
			if(status==1) estr+="\nCart. Control in motion";
			else if(status==0) estr+="\nCart. Control not in motion";
			else estr+="\nError: Cart. Control in limitation";
		}
	}

	if (!rsi_comm_) {
		diagnostics.status[0].message = "connection problem";
	} else if (!rsi_comm_->received_something()) {
		diagnostics.status[0].message = "connection did not receive anything yet";
	} else if (rsi_comm_->receivedAgo()>2.) {
		diagnostics.status[0].message = "connection did not receive anything within timeout threshold";
	} else if (estr.find("Error: ")!=std::string::npos) {
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

void RSINode::setMotionMode(const MOTION_MODE mode) {
	if(mode_motion_!=mode && rsi_comm_) {
		for(size_t i=0; i<joint_names_.size(); i++)
			rsi_comm_->clear(rsi_joint_name_+".A"+boost::lexical_cast<std::string>(i+1) );
		for(size_t i=0; i<3; i++) {
			rsi_comm_->clear(rsi_cart_name_+"."+sR_[i] );
			rsi_comm_->clear(rsi_cart_name_+"."+sT_[i] );
		}
	}
	mode_motion_ = mode;
}

void RSINode::_cb_move_group(moveit_msgs::MoveGroupGoalConstPtr goal)
{
	Eigen::Vector3d aimT, aimRPY;
	bool wait=false;
	size_t pos=0;
	ros::Rate r(getFrequency());
	
	bool error = false;
	while(!error && as_move_group_.isActive() && !as_move_group_.isPreemptRequested() && pos<goal->request.goal_constraints.size()) {
		
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
		
		bool reached = false;
		Eigen::Vector3d off, rot;
		for(size_t i=0; i<3; i++) {
			bool ret = rsi_comm_->get("RIst."+sT_[i], off(i));
			ret &= rsi_comm_->get("RIst."+sR_[i], rot(i));
			
			off(i) -= initial_values_[i+6];
			off(i) /= 1000.;
			rot(i)  = DEG_TO_RAD(rot(i)-initial_values_[i+9]);
			
			if(!ret) {
				ROS_ERROR("internal config problem: could not get property ('RIst')");
				rsi_comm_->setStopped(true);
				error = true;
				break;
			}
		}
		
		std::cout<<"off "<<off.transpose()<<std::endl;
		std::cout<<"aim "<<aimT.transpose()<<std::endl;
		std::cout<<"rot "<<rot.transpose()<<std::endl;
		std::cout<<"aim "<<aimRPY.transpose()<<std::endl;
		
		//assuming RSI will output the same RPY for the desired position (ignoring singularity?)
		if( (off-aimT).squaredNorm()<goal->request.goal_constraints[pos].position_constraints[0].weight &&
			(rot-aimRPY).squaredNorm()<goal->request.goal_constraints[pos].orientation_constraints[0].weight )
			reached = true;
		
		if(reached) {
			++pos;
			wait=false;
		}
		else
			r.sleep();
	}
	
	ROS_INFO("finished movement (cart.)");
		
	moveit_msgs::MoveGroupResult result;
	result.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
	if(pos<goal->request.goal_constraints.size()) 
		result.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
	as_move_group_.setSucceeded(result);
}

bool RSINode::follow_joint_traj(control_msgs::FollowJointTrajectoryGoal &goal, const bool action) {
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
		ROS_ERROR("malformed goal (trajectory): joint names do not match");
		return false;
	}
	if(!reorderMsg(goal.goal_tolerance)) {
		result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
		if(action) as_joint_.setSucceeded(result);
		ROS_ERROR("malformed goal (tolerance): joint names do not match");
		return false;
	}
	
	bool error = false;
	int id = ++current_movement_id_;
	while( !error && current_movement_id_==id && (!action || (!as_joint_.isPreemptRequested())) && pos<goal.trajectory.points.size()) {
		
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
			v = DEG_TO_RAD(v-initial_values_[i]);
			if(!ret) {
				error=true;
				ROS_ERROR("could not read actual joint position ('AIPos.A%d')", (int)(i+1));
				break;
			}
			
			feed.actual.positions[i] = v;
			feed.error.positions[i] = v-goal.trajectory.points[pos].positions[i];
			
			std::cout<<i<<": "<<v<<" "<<goal.trajectory.points[pos].positions[i]<<std::endl;
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
	
	ROS_INFO("finished movement (joint)");
		
	if(pos<goal.trajectory.points.size())
		result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
	if(action) as_joint_.setSucceeded(result);
	
	return pos==goal.trajectory.points.size();
}

void RSINode::cb_dyn_reconfig(ipa_kuka_rsi::DynConfig &cfg, uint32_t level)
{
	ROS_DEBUG("cb_dyn_reconfig %d",level);
	if(level)
		return;
	
	tip_link_ = cfg.tip_link;
	base_link_ = cfg.base_link;
	
	tolerance_ = cfg.tolerance;
	max_vel_ = cfg.max_velocity;
	frequency_ = cfg.frequency;
	setMotionMode( (MOTION_MODE)cfg.motion_mode );
	
	rsi_joint_name_ = cfg.RSI_joint_name;
	rsi_cart_name_ = cfg.RSI_cart_name;
	
	ROS_DEBUG("changed settings");
}

int main(int argc, char** argv) {
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "kuka_rsi");

	// create RSINode
	RSINode rsi_node;

	/// main loop
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	int freq;
	ros::Rate loop_rate(freq = rsi_node.getFrequency()); // Hz
	while (ros::ok()) {
		rsi_node.publishState();
		loop_rate.sleep();
		
		if(freq!=rsi_node.getFrequency())
			loop_rate = ros::Rate(freq = rsi_node.getFrequency());
	}

	return 0;
}
