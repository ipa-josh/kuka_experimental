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
 *   TCP/UDP server connection wich also handles parsing of configuration and RSI XML protocol
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

#include "rsi.h"
#include "network.h"
#include "erc_config.h"
#include <boost/thread.hpp>
#include <boost/timer.hpp>

/*!
 *  \addtogroup RSI
 *  @{
 */

//! RSI XML specific classes
namespace RSI {
	
	//! simple datarepresentation to cast between filename and parsed config
	class RSI_Server_Config
	{
		ERC_Config::ERC_Config config_;
		bool ok_;
		
	public:
		RSI_Server_Config(const std::string &config_filename);
		
		const ERC_Config::ERC_Config &getConfig() const {return config_;}
		bool ok() const {return ok_;}
	};
	
	//! implements network interface as well as RSI parsing
	class RSI_Server
	{
		//network stuff (order is important)
		boost::asio::io_service io_service_;			///< boost::asio specific
		
		ERC_Config::ERC_Config config_;					///< representation of the ERC configuration file
		
		//network stuff (order is important)
		TCP_Server tcp_server_;							///< TCP server connection
		boost::shared_ptr<UDP_Connection> udp_conn_;	///< UDP connection if specified
		
		//RSI stuff
		Datagram_toKRC   toKRC_;						///< generator of outgoing RSI XML
		Datagram_fromKRC fromKRC_;						///< handler for incoming data
		
		//internals
		boost::shared_mutex access_;					///< limit read/write access while processing data
		boost::thread thread_;							///< thread for network IO
		boost::timer timer_;							///< last time received anything
		bool init_;										///< flag: was connection established
		bool stopped_;									///< flag: is robot in run mode? otherwise default values for RSol and ASPos will be used
		
	public:
		
		/// start RSI server from configuration
		RSI_Server(const RSI_Server_Config &config);
		
		/// wait til thread is closed
		~RSI_Server() {thread_.join();}
		
		/// setter for value with tag name, returns true on success
		template<class T>
		bool set(const std::string &name, const T &v) {
			boost::upgrade_lock<boost::shared_mutex> lock(access_);
			boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
			return toKRC_.set<T>(name, v);
		}
		
		/// setter for clearing all values
		void clear() {
			boost::upgrade_lock<boost::shared_mutex> lock(access_);
			boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
			toKRC_.clear();
		}
		
		/// setter for clearing single values
		bool clear(const std::string &name) {
			boost::upgrade_lock<boost::shared_mutex> lock(access_);
			boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
			return toKRC_.clear(name);
		}
		
		/// getter of property with tag name, returns true on success
		template<class T>
		bool get(const std::string &name, T &v) {
			boost::upgrade_lock<boost::shared_mutex> lock(access_);
			return fromKRC_.get<T>(name, v);
		}
		
		/// is connection established?
		bool received_something() const {return init_;}
		/// time in seconds, when something was received successfully
		double receivedAgo() const {return timer_.elapsed()*100;}
		/// setter for stopped flag
		void setStopped(const bool b) {
			stopped_=b;
			set("Stop", (int64_t)true);	//if implemented --> use it!
		}
		
	private:
	
		/// handler for incoming raw data
		void on_data(const char *data, const size_t size, Any_Session *writer);
	};
}

/*! @} End of Doxygen Groups*/
