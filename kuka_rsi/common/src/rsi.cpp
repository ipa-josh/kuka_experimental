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
 
#include <ipa_kuka_rsi/rsi.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/replace.hpp>

using namespace RSI;

bool Datagram_fromKRC::parse(const boost::property_tree::ptree &pt)
{
	//get timestamp (IPOC)
	try {
		ipoc_ = pt.get<long long>("Rob.IPOC");
	} catch(...) {
		std::cerr<<"could not parse datagram (Datagram_fromKRC::IPOC)"<<std::endl;
		return false;
	}
	
	//get values (defined in config)
	try {
		BOOST_FOREACH(const boost::property_tree::ptree::value_type &v,
            pt.get_child("Rob")) {
			
			//single value
			{
				VALUES::iterator it = vals_.find(v.first);
				if(it!=vals_.end()) {
					//std::cout<<v.first<<": "<<v.second.data()<<std::endl;
					it->second = v.second;
					continue;
				}
			}
				
			if(!v.second.get_child_optional("<xmlattr>")) continue;
			
			BOOST_FOREACH(const boost::property_tree::ptree::value_type &v2,
				v.second.get_child("<xmlattr>")) {
					const std::string name = v.first+"."+v2.first;
					VALUES::iterator it = vals_.find(name);
					if(it==vals_.end()) continue;
					
					it->second = v2.second;
			}
		}
	} catch(...) {
		std::cerr<<"could not parse datagram (Datagram_fromKRC)"<<std::endl;
		return false;
	}
	
	return true;
}

void Datagram_toKRC::buildXMLTree()
{
	root_.clear();
	
	root_.put("Sen.<xmlattr>.Type", config_.sentype);
	ipoc_ = DataXML(&root_.put("Sen.IPOC", (long long)0), ERC_Config::Element::LONG);
	
	for(size_t i=0; i<send_config_.elements.size(); i++) {
		std::string name = send_config_.elements[i].tag;
		boost::replace_first(name, ".", ".<xmlattr>.");
		vals_[send_config_.elements[i].tag] = DataXML(&root_.put("Sen."+name, ""), send_config_.elements[i].type);
	}
	
	//for(VALUES::const_iterator it=vals_.begin(); it!=vals_.end(); it++)
	//	std::cout<<it->first<<std::endl;
}

const boost::property_tree::ptree &Datagram_toKRC::getXML(const long long ipoc)
{
	ipoc_ = (int64_t)ipoc;
	return root_;
}

//specializations...
template<>
bool Datagram_fromKRC::get<bool>(const std::string &name, bool &r) {
	VALUES::iterator it = vals_.find(name);
	if(it==vals_.end() || it->second.type_!=ERC_Config::Element::BOOL) return false;
	r = (bool)it->second.vi_;
	return it->second.set_;
}

template<>
bool Datagram_fromKRC::get<double>(const std::string &name, double &r) {
	VALUES::iterator it = vals_.find(name);
	if(it==vals_.end() || (it->second.type_!=ERC_Config::Element::DOUBLE&&it->second.type_!=ERC_Config::Element::FLOAT)) return false;
	r = it->second.vd_;
	return it->second.set_;
}

template<>
bool Datagram_fromKRC::get<int64_t>(const std::string &name, int64_t &r) {
	VALUES::iterator it = vals_.find(name);
	if(it==vals_.end() || it->second.type_!=ERC_Config::Element::LONG) return false;
	r = it->second.vi_;
	return it->second.set_;
}

template<>
bool Datagram_fromKRC::get<std::string>(const std::string &name, std::string &r) {
	VALUES::iterator it = vals_.find(name);
	if(it==vals_.end() || it->second.type_!=ERC_Config::Element::STRING) return false;
	r = it->second.vs_;
	return it->second.set_;
}

