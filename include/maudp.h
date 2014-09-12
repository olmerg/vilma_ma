/**
 *
	 @brief This is the library to RX/TX by UDP with the microautobox II using asynchronoues function in client mode!!(first sent a package and wait for receive a new data)
	 it is using the boost library , sending package in block mode with a time deadline
	 
 @author olmer Garcia olmerg@gmail.com

 * Copyright 2013 Olmer Garcia Bedoya
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef MAUDP_H_
#define MAUDP_H_
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/asio/deadline_timer.hpp>
#include "protocolo.h"
#include <string>
#include <sstream>

using boost::asio::ip::udp;
using boost::asio::deadline_timer;
namespace microautobox {
/**
* function to convert <type> to str
*/
template<typename T>
std::string toString(const T& value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}
/**
 * class that implement the protocol of communication UDP supported by the microautobox
 *
 * it is really necessary to have two sockets in the linux pc?
 * it could be better to look for a protocol and implement it in matlab?
 * read http://stackoverflow.com/questions/276058/what-is-the-optimal-size-of-a-udp-packet-for-maximum-throughput
 *    SCTP or DCCP
 * note that the number of bytes should not be more than 512 bytes to reduce the risk of fragmentation.
 *
 */
class maudp {

private:  mutable boost::mutex mtx_;

public:
 int server;
 /**
 default constructor of the class
 */
	maudp();
	/**
	 *configure the socket
	 * @param port			port of the server in the embedded pc
	 * @param port_client  port of the microautobox II
	 * @param IP_client    IP of the microautobox II
	 * @param timeout       time out to make a request
	 * @param size_y        number of sensor variables of MA (default 13)
	 * @param size_u		number of commands to MA (default 10)
	 */
	bool configure(short port,short port_client,std::string IP_client,boost::posix_time::time_duration timeout,int size_y=13,int size_u=10);
	/**
	 *destrutor
	 */
	virtual ~maudp();

	/**
	 * synchronous function to execute the request to the server MA
	 * @return message of the request
	 */

	const std::string& run();

	/**
	 * return vector of data to be sent to MA
	 * @return u
	 */

	const std::vector<double>& getU() const {
		boost::lock_guard<boost::mutex> guard(mtx_);
		return u;
	}
/**
 *   set vector of data to be sent to MA
 * @param u
 */
	void setU(const std::vector<double>& u) {
		boost::lock_guard<boost::mutex> guard(mtx_);
		if(this->u.size()==u.size())
		this->u = u;

	}
/**
 *  return vector of data received from MA
 * @return
 */
	const std::vector<double>& getY() const {
		boost::lock_guard<boost::mutex> guard(mtx_);
		//mtx_.lock();		
		return y;
		//mtx_.unlock();		
	}






private:
	/**


	 * asynchronous function to send (TX) a vector of data to the MA by udp
	 * and then wait to receive
	 * 
	 */
	void send_toMA();

	/**
 	  *  set vector of data received from MA
 	  * @param y
 	*/
	void setY(const std::vector<double>& y) {
		boost::lock_guard<boost::mutex> guard(mtx_);
		this->y = y;
	}
	/**
	 * @brief timer function which control the time maximum to sent and receive
	 * a data to the MicroAutobox.
	 * this function cancel the TX or RX if timeout
	 *
	 * TODO: implementar alarmas
	 */
	void handle_timer(const boost::system::error_code& error);
	/**
	 *@brief asynchronous function to send data (TX) and make the request to receive data. 
	 * @param ec   sent errors
	 * @param length sent bytes
	 */
	 void handle_tx(
	      const boost::system::error_code& ec, std::size_t length);
/**
 * function that receive the information from the Microautobox and
 * send the answer to them
 * @param error

 * @param bytes_recvd
 */
	void handle_rx(const boost::system::error_code& error,
	      size_t bytes_recvd);
	

  boost::asio::io_service io_service_;
  udp::socket socket_server;
  udp::endpoint recieved_endpoint_;
  /// The signal_set is used to register for process termination notifications.
//  boost::asio::signal_set signals_;
  /// Handle a request to stop the server.
  void handle_stop();
  /**
   * function to print a data for debug
   * @param comment  sent or received
   */
  void print_trama(const char* );
//  udp::resolver myresolver;
  //udp::resolver::query query;
 // udp::resolver::iterator iterator ;
  protocolo prot;
  boost::shared_ptr<boost::asio::deadline_timer>  deadline_;
  boost::posix_time::time_duration timeout;
  enum { max_length = 1024 };
  unsigned char data_rx[max_length];
  unsigned char *data_tx;
  int checksum;
  int tx_error;
  int rx_error;
  int rx_try;
  int tx_try;
  long errores;
  long erroresMA;
  unsigned short port_client;
  std::vector<double> yrx_1;
  std::vector<double> u_1;
  std::vector<double> y;
  std::vector<double> u;

  std::vector<int> tipodato;
  std::vector<double> trama;
  static const unsigned short bytes_udp=192;
  bool  _message_received,_timeout_triggered;
  std::string message;

private:

/* CRC16 implementation acording to CCITT standards */
static unsigned short crc16tab[256];
/*	
 * Copyright 2001-2010 Georges Menie (www.menie.org)
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of California, Berkeley nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
unsigned short crc16_ccitt(const unsigned char *buf, int len);

};

} /* namespace microautobox */
#endif /* MAUDP_H_ */

