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
 *   parse and store ERC configuration file with these classes
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

#include "parsable.h"
#include <stdint.h>
#include <string>
#include <vector>

/*!
 *  \addtogroup ERC_Config
 *  @{
 */

//! data types to store configuration of ERC configuration
namespace ERC_Config {
	
	//! configuration (mainly for transmission protocol)
	struct Config : public Parsable
	{
		enum PROTOCOL {UDP, TCP};
		
		std::string ip;		///< ip of this PC
		uint16_t port;		///< port to listen to
		PROTOCOL protocol;	///< is UDP supported
		std::string sentype;
		bool protocollength;
		bool onlysend;		///< read-only connection
		
		/// serialize a property tree to this structure
		virtual bool parse(const boost::property_tree::ptree &pt);
	};

	//! a configuration element of Receive or Send tag
	struct Element : public Parsable {
		enum TYPE {BOOL, LONG, FLOAT, DOUBLE, STRING, __ERROR};
		
		std::string tag;	///< name of value, e.g. "Abc.Def" (Def is attribute of Abc) or "Abc" (single value)
		TYPE type;			///< datatype to store
		std::string indx;	///< index (not used here)
		uint32_t unit;		///< unit of content (not used at the moment, TODO: ?)
		bool holdon;
		
		/// serialize a property tree to this structure
		virtual bool parse(const boost::property_tree::ptree &pt);
	};

	//! container for elements of Receive and Send
	struct SendRecv : public Parsable {
		std::vector<Element> elements;	///< expanded elements of communication structure
		
		/// serialize a property tree to this structure, default defines (DEF_*) will be expanded automatically
		virtual bool parse(const boost::property_tree::ptree &pt);
	};
	
	//! container for complete configuration
	struct ERC_Config : public Parsable {
		Config config;			///< general communication configuration
		SendRecv send, recv;	///< content of RSI communication
		
		/// serialize a property tree to this structure
		virtual bool parse(const boost::property_tree::ptree &pt);
	};
}

/*! @} End of Doxygen Groups*/
