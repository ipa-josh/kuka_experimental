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
 *   easy to access data representation of the RSI XML protocol (application <-> RSI parameter)
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
 
#pragma once

#include "erc_config.h"
#include <map>
#include <boost/thread/shared_mutex.hpp> 

/*!
 *  \addtogroup RSI
 *  @{
 */

//! RSI XML specific classes
namespace RSI {
	
	//! container of content of all supported data types
	class DataValue {
	public:
		union {
			double vd_;					///< content for double
			int64_t vi_;					///< content for integer
		};
		std::string vs_;					///< content for string
		ERC_Config::Element::TYPE type_;	///< data type
		bool set_;							///< flag for: was the value set at least once
		
		/// default constructor --> error type
		DataValue():type_(ERC_Config::Element::__ERROR), set_(false)
		{}
		
		/// init. constructor of given type
		DataValue(const ERC_Config::Element::TYPE type):type_(type), set_(false)
		{}
		
		/// serialize XML value to this container
		void operator=(const boost::property_tree::ptree &v) {
			switch(type_) {
				case ERC_Config::Element::BOOL:
				case ERC_Config::Element::LONG:
					vi_ = v.get_value<int64_t>();
					set_= true;
					break;
					
				case ERC_Config::Element::FLOAT:
				case ERC_Config::Element::DOUBLE:
					vd_ = v.get_value<double>();
					set_= true;
					break;
					
				case ERC_Config::Element::STRING:
					vs_ = v.get_value<std::string>();
					set_= true;
					break;
					
				default:
					std::cerr<<"Error: unsupported data type for serialization"<<std::endl;
					break;
			}
		}
	};
	
	//! access handler for single XML property (of given type)
	class DataXML {
		boost::property_tree::ptree *val_;	///< pointer to XML property representation
		ERC_Config::Element::TYPE type_;	///< data type of element
		bool set_;							///< flag for: was the value set at least once
		
	public:
	
		/// default constructor --> error type
		DataXML():val_(NULL), type_(ERC_Config::Element::__ERROR), set_(false)
		{}

		/// init. constructor of given type, connected to a XML property tree
		DataXML(boost::property_tree::ptree *val, const ERC_Config::Element::TYPE type):val_(val), type_(type), set_(false)
		{}
		
		/// setter for a double value (value -> XML)
		void operator=(const double v) {
			if(!val_) {
				std::cerr<<"Error: null pointer exception!"<<std::endl;
				return;
			}
			if(!(type_==ERC_Config::Element::FLOAT || type_==ERC_Config::Element::DOUBLE)) {
				std::cerr<<"Error: incompatible data types"<<std::endl;
				return;
			}
			val_->put_value(v);
			set_ = (type_==ERC_Config::Element::FLOAT || type_==ERC_Config::Element::DOUBLE);
		}
		
		/// setter for a integer value (value -> XML)
		void operator=(const int64_t v) {
			if(!val_) {
				std::cerr<<"Error: null pointer exception!"<<std::endl;
				return;
			}
			if(!(type_==ERC_Config::Element::BOOL || type_==ERC_Config::Element::LONG)) {
				std::cerr<<"Error: incompatible data types"<<std::endl;
				return;
			}
			val_->put_value(v);
			set_ = (type_==ERC_Config::Element::BOOL || type_==ERC_Config::Element::LONG);
		}
		/// setter for a string value (value -> XML)
		void operator=(const std::string &v) {
			if(!val_) {
				std::cerr<<"Error: null pointer exception!"<<std::endl;
				return;
			}
			if(!(type_==ERC_Config::Element::STRING)) {
				std::cerr<<"Error: incompatible data types"<<std::endl;
				return;
			}
			val_->put_value(v);
			set_ = (type_==ERC_Config::Element::STRING);
		}
		
		void clear() {
			set_ = false;
			if(!val_) {
				std::cerr<<"Error: null pointer exception!"<<std::endl;
				return;
			}
			val_->clear();
		}
	};
	
	//! process data storage of incoming data (from RSI)
	class Datagram_fromKRC : public Parsable {
		typedef std::map<std::string, DataValue> VALUES;	///< mapping between tag names and values
		
		long long ipoc_;									///< newest IPOC of received data
		const ERC_Config::SendRecv &config_;				///< configuration needed for parsing XML
		VALUES vals_;										///< container of received data
		
	public:
	
		/// constructor needs valid configuration --> build up container
		Datagram_fromKRC(const ERC_Config::SendRecv &config):
			config_(config)
		{
			for(size_t i=0; i<config_.elements.size(); i++)
				vals_[config_.elements[i].tag] = DataValue(config_.elements[i].type);
		}

		/// parse incoming XML data
		virtual bool parse(const boost::property_tree::ptree &pt);
		
		/// getter of property with tag name, returns true on success
		template<class T>
		bool get(const std::string &name, T &r) {
			BOOST_STATIC_ASSERT_MSG(sizeof(T)==-1, "this datatype is not supported");
			return false;
		}
		
		/// get the newest IPOC
		long long getIPOC() const {return ipoc_;}
	};
	
	/// getter of boolean property with tag name, returns true on success
	template<> bool Datagram_fromKRC::get<bool>(const std::string &name, bool &r);
	/// getter of double property with tag name, returns true on success
	template<> bool Datagram_fromKRC::get<double>(const std::string &name, double &r);
	/// getter of integer property with tag name, returns true on success
	template<> bool Datagram_fromKRC::get<int64_t>(const std::string &name, int64_t &r);
	/// getter of string property with tag name, returns true on success
	template<> bool Datagram_fromKRC::get<std::string>(const std::string &name, std::string &r);


	//! builds XML tree from configuration file as response to request (to RSI)
	class Datagram_toKRC {
		typedef std::map<std::string, DataXML> VALUES;	///< mapping between tag names and values
		
		const ERC_Config::Config &config_;			///< configuration of connection
		const ERC_Config::SendRecv &send_config_;	///< contained data of RSI communication
		VALUES vals_;								///< mapped values between tag name and XML properties
		boost::property_tree::ptree root_;			///< root of XML tree
		DataXML ipoc_;								///< fast access to IPOC
		
		/// build an initial tree (only once!)
		void buildXMLTree();
	public:
	
		/// constructor initializes XML tree
		Datagram_toKRC(const ERC_Config::Config &config, const ERC_Config::SendRecv &send_config):
			config_(config), send_config_(send_config)
		{
			buildXMLTree();
		}
		
		/// setter for value with tag name, returns true on success
		template<class T>
		bool set(const std::string &name, const T &v) {
			VALUES::iterator it = vals_.find(name);
			if(it==vals_.end()) return false;
			it->second = v;
			return true;
		}
		
		void clear() {
			for(VALUES::iterator it=vals_.begin(); it!=vals_.end(); it++)
				it->second.clear();
		}
		
		bool clear(const std::string &name) {
			VALUES::iterator it = vals_.find(name);
			if(it==vals_.end()) return false;
			it->second.clear();
			return true;
		}
		
		/// returns XML tree with an updated IPOC
		const boost::property_tree::ptree &getXML(const long long ipoc);
	};
}

/*! @} End of Doxygen Groups*/
