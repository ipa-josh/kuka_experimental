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
 *   TCP and UDP (server) connections with boost::asio
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

#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>

#include <ros/console.h>	//for ROS_ERROR, ROS_INFO, ...

//! abstract connection handler to process incoming data (independent of protocol)
class Any_Session
{
public:
  /// incoming data with "data" of size "size" and handler to write back ("writer")
  typedef boost::signals2::signal<void (const char *data, const size_t size, Any_Session *writer)> SIG_ON_DATA;
  
  /// every session needs a data handler
  Any_Session(SIG_ON_DATA &cb)
    : on_data_(cb)
  {
  }
  
  virtual ~Any_Session() {}
	
  /// write a string to the network connection
  virtual void write(const std::string &buffer)=0;
  
protected:

	SIG_ON_DATA &on_data_;	///< signal handler for incoming data
};


//! implementation connection handler for TCP
class TCP_Session : public Any_Session
{
public:
  
  TCP_Session(boost::asio::io_service& io_service, SIG_ON_DATA &cb)
    : Any_Session(cb), socket_(io_service)
  {
  }
  
  virtual ~TCP_Session() {}

  /// getter for socket connection
  boost::asio::ip::tcp::socket& socket()
  {
    return socket_;
  }

  /// start listening on socket
  virtual void start()
  {
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&TCP_Session::handle_read, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }
  
protected:
	
	/// write async. to TCP socket (implement abstract)
	virtual void write(const std::string &buffer) {
      boost::asio::async_write(socket_,
          boost::asio::buffer( buffer ),
          boost::bind(&TCP_Session::handle_write, this,
            boost::asio::placeholders::error));
	}
	
	/// write async. to TCP socket
	virtual void write(const boost::asio::mutable_buffers_1 &buffer) {
      boost::asio::async_write(socket_,
          buffer,
          boost::bind(&TCP_Session::handle_write, this,
            boost::asio::placeholders::error));
	}

private:

  /// called on incoming data --> decides if an error occured or calls handler
  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error)
    {
		on_data_(data_, bytes_transferred, this);
    }
    else
    {
		ROS_ERROR("error while reading from socket");
      delete this;
    }
  }

  /// called if data is written (error handling)
  void handle_write(const boost::system::error_code& error)
  {
    if (error)
    {
		ROS_ERROR("error while writing to socket");
      delete this;
    }
  }

protected:
  boost::asio::ip::tcp::socket socket_;	///< TCP socket
  enum { max_length = 4096 };	///< max length of incoming data (shoul be enough?)
  char data_[max_length];		///< data buffer of max. length
};

//! server for TCP connection
class TCP_Server
{
public:

  /// TCP server listens on port
  TCP_Server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
  {
  }
  
  /// getter and sett ofer signal handler of new TCP connections
  Any_Session::SIG_ON_DATA &getSignal() {return on_data_;}
  
  /// start now by accepting new TCP connection
  void start_accept()
  {
    TCP_Session* new_session = new TCP_Session(io_service_, on_data_);
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&TCP_Server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }
private:

  /// called on new TCP connection
  void handle_accept(TCP_Session* new_session,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      new_session->start();
		ROS_INFO( "connection established" );
    }
    else
    {
      delete new_session;
		ROS_ERROR( "error while accepting new client" );
    }

    start_accept();
  }

  boost::asio::io_service& io_service_;		///< boost::asio specific
  boost::asio::ip::tcp::acceptor acceptor_;	///< boost::asio specific
  Any_Session::SIG_ON_DATA on_data_;		///< signal handler for incoming data
};


//! implementation connection handler for UDP
class UDP_Connection : public Any_Session
{
public:

  /// listening on port with signal handler for new data
  UDP_Connection(boost::asio::io_service& io_service, short port, SIG_ON_DATA &cb)
    : Any_Session(cb), io_service_(io_service),
      socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port))
  {
    start_receive();
  }
  
private:

  /// start by listening to socket
  void start_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&UDP_Connection::handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }
	
  /// writing data to UDP socket
  virtual void write(const std::string &buffer) {
	  socket_.async_send_to(boost::asio::buffer(buffer), remote_endpoint_,
		  boost::bind(&UDP_Connection::handle_send, this, buffer,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
  }
  
  /// called on incoming data --> decides if an error occured or calls handler
  void handle_receive(const boost::system::error_code& error,
      std::size_t bytes_transferred)
  {
    if (!error)
    {
      on_data_(&recv_buffer_.front(), std::min(bytes_transferred, recv_buffer_.size()), this);
      start_receive();
    }
    else
		ROS_ERROR("error while reading from socket");
  }

  /// called if data is written (error handling)
  void handle_send(std::string /*message*/,
      const boost::system::error_code& error,
      std::size_t /*bytes_transferred*/)
  {
	  if(error)
		ROS_ERROR("error while writing to socket (UDP)");
  }

  boost::asio::io_service& io_service_;				///< boost::asio specific
  boost::asio::ip::udp::socket socket_;				///< boost::asio specific
  boost::asio::ip::udp::endpoint remote_endpoint_;	///< IP/port of remote
  boost::array<char, 4096> recv_buffer_;			///< data buffer for incoming data with fixed size
};
