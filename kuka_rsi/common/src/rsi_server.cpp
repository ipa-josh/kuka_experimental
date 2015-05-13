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
 
#include <ipa_kuka_rsi/rsi_server.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/lexical_cast.hpp>

using namespace RSI;

RSI_Server_Config::RSI_Server_Config(const std::string &config_filename) : ok_(false)
{
	boost::property_tree::ptree pt;
	try {
		read_xml(config_filename, pt);
	} catch(...) {
		std::cerr<<"failed to parse: "<<config_filename<<std::endl;
		return;
	}
	
	ok_ = config_.parse(pt);
}

RSI_Server::RSI_Server(const RSI_Server_Config &config):
	config_(config.getConfig()),
	tcp_server_(io_service_, config_.config.port),
	toKRC_(config_.config, config_.send), fromKRC_(config_.recv),
	init_(false), stopped_(true)
{
	if(!config.ok()) return;
	
	tcp_server_.getSignal().connect( boost::bind(&RSI_Server::on_data, this, _1, _2, _3) );
	
	if(config_.config.protocol==ERC_Config::Config::UDP)
		udp_conn_.reset( new UDP_Connection(io_service_, config_.config.port, tcp_server_.getSignal()) );
	
	tcp_server_.start_accept();
	
	//now start asynchronous network event handling
	thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}


void RSI_Server::on_data(const char *data, const size_t size, Any_Session *writer)
{
	boost::upgrade_lock<boost::shared_mutex> lock(access_);
	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
			
	boost::property_tree::ptree pt_in;
	std::istringstream ss(std::string(data, size));
	try {
		boost::property_tree::xml_parser::read_xml(ss, pt_in);
	} catch(...) {
		std::cerr<<"failed to parse response (XML malformed)"<<std::endl;
		std::cout<<"content: "<<size<<"\n"<<std::string(data, size)<<std::endl;
		return;
	}
	
	if(!fromKRC_.parse(pt_in)) {
		std::cerr<<"could not parse response from KRC"<<std::endl;
		std::cout<<std::string(data, size)<<std::endl;
		return;
	}
	
	//respond immediately
	if(!config_.config.onlysend) {
		//if we stopped we copy actual state to aim
		if(stopped_) {
			//first clear all parameters and set absolute values to current values
			toKRC_.clear();
			
			int p=1;
			double v;
			while(fromKRC_.get("AIPos.A"+boost::lexical_cast<std::string>(p), v)) {
				toKRC_.set("ASPos.A"+boost::lexical_cast<std::string>(p), v);
				++p;
			}
			const char *C[]={"X","Y","Z","A","B","C"};
			for(p=0; p<6; p++) {
				if(fromKRC_.get("RIst."+std::string(C[p]), v))
					toKRC_.set("RSol."+std::string(C[p]), v);
			}
		}
		
		std::ostringstream ostr;
		boost::property_tree::ptree pt_out = toKRC_.getXML(fromKRC_.getIPOC());
		boost::property_tree::xml_parser::write_xml(ostr, pt_out);
		writer->write(ostr.str());
	}
	
	//update timer
	init_ = true;
	timer_.restart();
}
