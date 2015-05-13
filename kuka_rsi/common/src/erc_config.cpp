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
 
 #include <ipa_kuka_rsi/erc_config.h>
#include <map>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

using namespace ERC_Config;

bool Config::parse(const boost::property_tree::ptree &pt)
{
	try {
		std::map<std::string, PROTOCOL> protocol_map = boost::assign::map_list_of("TCP", TCP)("UDP", UDP);
		std::map<std::string, bool> protocollength_map = boost::assign::map_list_of("On", true)("Off", false);
		std::map<std::string, bool> onlysend_map = boost::assign::map_list_of("TRUE", true)("FALSE", false);
		
		ip = pt.get<std::string>("IP_NUMBER");
		port = pt.get<uint16_t>("PORT");
		protocol = protocol_map[pt.get<std::string>("PROTOCOL")];
		sentype = pt.get<std::string>("SENTYPE");
		protocollength = protocollength_map[pt.get<std::string>("PROTCOLLENGTH")];
		onlysend = onlysend_map[pt.get<std::string>("ONLYSEND", "FALSE")];
	} catch(...) {
		std::cerr<<"could not parse ERC configuration (Config)"<<std::endl;
		return false;
	}
	
	return true;
}

bool Element::parse(const boost::property_tree::ptree &pt)
{
	try {
		std::map<std::string, TYPE> protocol_map = boost::assign::map_list_of("BOOL", BOOL)("LONG", LONG)("FLOAT", FLOAT)("DOUBLE", DOUBLE)("STRING", STRING);
		std::map<std::string, bool> protocollength_map = boost::assign::map_list_of("On", true)("Off", false);
		std::map<std::string, bool> onlysend_map = boost::assign::map_list_of("TRUE", true)("FALSE", false);				
		
		tag = pt.get<std::string>("TAG");
		type = protocol_map[pt.get<std::string>("TYPE")];
		indx = pt.get<std::string>("INDX");
		unit = pt.get<uint32_t>("UNIT", 0);
		holdon = (pt.get<uint32_t>("HOLDON", false)!=0);
	} catch(...) {
		std::cerr<<"could not parse ERC configuration (Element)"<<std::endl;
		return false;
	}
	
	return true;
}

bool SendRecv::parse(const boost::property_tree::ptree &pt)
{
	try {
		BOOST_FOREACH(const boost::property_tree::ptree::value_type &v,
            pt.get_child("ELEMENTS")) {
				if(v.first!="ELEMENT") continue;
				elements.push_back(Parsable::get<Element>(v.second.get_child("<xmlattr>")));
				
				//hardcoded mapping!
				if(elements.back().tag=="DEF_RIst" || elements.back().tag=="DEF_RSol") {
					elements.back().tag = std::string(elements.back().tag.begin()+4, elements.back().tag.end());
					Element cp = elements.back(), cp2;
					elements.pop_back();
					
					cp2=cp; cp2.tag+=".X";
					elements.push_back(cp2);
					cp2=cp; cp2.tag+=".Y";
					elements.push_back(cp2);
					cp2=cp; cp2.tag+=".Z";
					elements.push_back(cp2);
					cp2=cp; cp2.tag+=".A";
					elements.push_back(cp2);
					cp2=cp; cp2.tag+=".B";
					elements.push_back(cp2);
					cp2=cp; cp2.tag+=".C";
					elements.push_back(cp2);
				}
				else if(elements.back().tag=="DEF_AIPos" || elements.back().tag=="DEF_ASPos" || elements.back().tag=="DEF_MACur") {
					elements.back().tag = std::string(elements.back().tag.begin()+4, elements.back().tag.end());
					Element cp = elements.back(), cp2;
					elements.pop_back();
					
					for(int i=1; i<=6; i++) {
						cp2=cp; cp2.tag+=".A"+boost::lexical_cast<std::string>(i);
						elements.push_back(cp2);
					}
				}
				else if(elements.back().tag=="DEF_EIPos" || elements.back().tag=="DEF_ESPos" || elements.back().tag=="DEF_MECur") {
					elements.back().tag = std::string(elements.back().tag.begin()+4, elements.back().tag.end());
					Element cp = elements.back(), cp2;
					elements.pop_back();
					
					for(int i=1; i<=6; i++) {
						cp2=cp; cp2.tag+=".E"+boost::lexical_cast<std::string>(i);
						elements.push_back(cp2);
					}
				}
				else if(elements.back().tag=="DEF_Delay") {
					elements.back().tag = "Delay";
				}
				else if(elements.back().tag=="DEF_EStr") {
					elements.back().tag = "EStr";
				}
				else continue;
				
		}
	} catch(...) {
		std::cerr<<"could not parse ERC configuration (Send/Recv)"<<std::endl;
		return false;
	}
	
	return true;
}

bool ERC_Config::ERC_Config::parse(const boost::property_tree::ptree &pt)
{
	try {
		if(!config.parse(pt.get_child("ROOT.CONFIG")))
			return false;
		if(!send.parse(pt.get_child("ROOT.RECEIVE")))
			return false;
		if(!config.onlysend && !recv.parse(pt.get_child("ROOT.SEND")))
			return false;
	} catch(...) {
		std::cerr<<"could not parse ERC configuration"<<std::endl;
		return false;
	}
	
	return true;
}

