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
 
 #include <ipa_kuka_rsi/network.h>

/*
class EmptyNet_Session : public TCP_Session {
public:
  EmptyNet_Session(boost::asio::io_service& io_service)
    : TCP_Session(io_service)
  {
  }
  
	virtual void on_data(const char *data, const size_t size) {}
};

class EmptyUDP_Connection : public UDP_Connection {
public:
  EmptyUDP_Connection(boost::asio::io_service& io_service)
    : UDP_Connection(io_service, 1234)
  {
  }
  
	virtual void on_data(const char *data, const size_t size, std::vector<char> &send_buffer) {}
};

void _test_instantiation()
{
  try
  {
	boost::asio::io_service io_service;
	TCP_Server s(io_service, 1234);
	//s.getSignal().bind();
	s.start_accept();

	io_service.run();
  }
  catch (std::exception& e)
  {
	ROS_ERROR( "Exception: %s", e.what() );
  }
}
*/
