/**
 *
	 @brief This is the library to RX/TX by UDP with the microautobox II using asynchronoues function in client mode!!(first sent a package and wait for receive a new data)
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

#include "maudp.h"
#include "protocolo.h"
#include <iostream>
#include <vector>
#include <boost/thread/mutex.hpp>
//#include "boost/assign/std/vector.hpp"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <iomanip>
//#define DEBUG 1

using boost::asio::ip::udp;
//using namespace boost::assign;
using boost::asio::deadline_timer;

namespace microautobox {
unsigned short maudp::crc16tab[256]= {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
maudp::maudp():
				//io_service_(io_service),
				socket_server(io_service_),
			//	signals_(io_service_),
				prot(bigendian),
				deadline_(new boost::asio::deadline_timer(io_service_))
{
	server=0;

}


bool maudp::configure( short port,short port_client,std::string IP_client, boost::posix_time::time_duration to,int size_y,int size_u)
{
	server=0;
	socket_server.open(udp::v4());
	boost::system::error_code ec;
	socket_server.bind(udp::endpoint(udp::v4(), port),ec);
	
	if(ec)
	{
	message.append("ERROR. Error binding the socket ");
			message.append(ec.message());//ROS_INFO("error to receive data %s ",error.message());
			message.append("\n");	
			return false;
	}
		recieved_endpoint_.address(boost::asio::ip::address::from_string(IP_client));
		recieved_endpoint_.port(port_client);
/*	// Register to handle the signals that indicate when the server should exit.
	// It is safe to register for the same signal multiple times in a program,
	// provided all registration for the specified signal is made through Asio.
	signals_.add(SIGINT);
	signals_.add(SIGTERM);
#if defined(SIGQUIT)
	signals_.add(SIGQUIT);
#endif // defined(SIGQUIT)
	signals_.async_wait(boost::bind(&maudp::handle_stop, this));
*/
	this->port_client=port_client;
	y.reserve(size_y);
	yrx_1.reserve(size_y);
	u.reserve(size_u);
	u_1.reserve(size_u);

	y.resize(size_y,0.0);
	yrx_1.resize(size_y,0.0);
	u.resize(size_u,0.0);
	tipodato.reserve(3+size_y+size_u);
	tipodato.resize(3+size_y+size_u,9);
	tipodato[0]=7;
	tipodato[1]=4;
	tipodato[2]=4;
	trama.reserve(3+size_y+size_u);
	trama.resize(3+size_y+size_u,0.0);

		for(unsigned int i=0;i<u.size();i++)
	{
		u[i]=i;
	}
	u_1.resize(size_u,0.0);
	timeout=to;
	checksum=size_u;

	tx_error=0;
	rx_error=0;
	tx_try=0;
	rx_try=0;
	errores=0;
	//iterator = myresolver.resolve(sender_endpoint_);

	boost::asio::socket_base::receive_buffer_size option(300); //el buffer queda menor a dos paquetes para evitar ver viejos paquetes
	//http://stackoverflow.com/questions/2769555/boost-asio-udp-retrieve-last-packet-in-socket-buffer
	//boost::asio::socket_base::send_buffer_size options(300);
	socket_server.set_option(option);
	//socket_server.set_option(options);
	return true;


}

maudp::~maudp() {
	// TODO Auto-generated destructor stub
}

void maudp::handle_stop()
{
	io_service_.stop();
}

void maudp::handle_rx(const boost::system::error_code& error,
		size_t bytes_recvd) {
	
	if (!error && bytes_recvd ==bytes_udp)
	{
		std::vector<unsigned char> v(data_rx, data_rx + bytes_recvd);
		trama=prot.decode(v,tipodato);
		print_trama("received");
			//this->rx_status=trama[0];
			//algorithm for receive  y
			if(trama[0]==0 && trama[1]>0) //confirmado en el server
			{
				//std::copy ( trama.begin()+3, trama.begin()+3+y_1.size(), y_1.begin());
				this->setY(yrx_1);
				this->rx_error=bytes_recvd;
				this->rx_try=0;
			}
			else{

				if(rx_try<5){
					this->rx_error=bytes_recvd;
					this->rx_try++;
				}
				else  //many errors to receive the data
				{
					this->rx_error=-20;
					errores+=this->rx_try;
					message.append("ALARM. Many errors to receive new data \n");//ROS_INFO(" ");
					errores+=rx_try;
				}
			}
			// this y received  will be confirmed in the next iteration
			std::copy ( trama.begin()+3, trama.begin()+3+yrx_1.size(), yrx_1.begin());
			// check if the u received is the same that i sent in the last iteration
			checksum=0;
			for(unsigned int i=0;i<u_1.size();i++){
				if(abs(u_1[i]-trama[y.size()+3+i])>0){
					checksum++;}
			}
		_message_received=true;

		}
		else
		{
			this->rx_error=-10;
			this->rx_try++;// envie un nuevo dato este se perdio
			std::copy ( u_1.begin(), u_1.end(), trama.begin()+3+y.size());

			message.append("ALARM. Error in the data recieved ");
			message.append(error.message());//ROS_INFO("error to receive data %s ",error.message());
			message.append("\n");
			//errores++;
			checksum=u.size();
			deadline_->cancel();
		}
		trama[2]=rx_error;


}

	 void maudp::handle_tx(
	      const boost::system::error_code& ec, std::size_t length)
	  {
		tx_error=length;
	if (!ec && length ==bytes_udp)
		{
		socket_server.async_receive_from(
					boost::asio::buffer(data_rx, max_length), 		
					recieved_endpoint_,
					boost::bind(&maudp::handle_rx, this,
							boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
	  	}
	else
		{
			message.append("ALARM. Error to send data ");
			message.append(ec.message());
			message.append("\n");
			deadline_->cancel();
		}
	}

void maudp::send_toMA()
{

		

		//deadline_.reset(new boost::asio::deadline_timer(io_service_));
		//deadline_->expires_from_now(timeout);
		deadline_->expires_at(deadline_timer::traits_type::now()+timeout);
		deadline_->async_wait(boost::bind(&maudp::handle_timer, this,boost::asio::placeholders::error));
/// if the last message was not receive by timeout maybe now is available because the server(MA just send if a client make the request)
		boost::asio::socket_base::bytes_readable command(true);
		socket_server.io_control(command);
		std::size_t bytes_readable = command.get();
		if(bytes_readable>0)
			{
			//std::cout<<"paquete pendiente bytes "<<bytes_readable<<"\n";
			bytes_readable = socket_server.receive_from(boost::asio::buffer(data_rx, max_length),
					recieved_endpoint_);
			
			socket_server.io_control(command);
			std::size_t bytes_readable = command.get();
			//std::cout<<"bytes not read in previous call "<<bytes_readable<<"\n";
			message.append("ALARM. bytes not read in previous call ");
			}


	if(checksum==0 && tx_error==bytes_udp) //dato confirmado
	{
//		u[0]++;
//		u[1]=std::rand()/(1.0*RAND_MAX);
		u_1=this->getU();
		std::copy ( u_1.begin(), u_1.end(), trama.begin()+3+y.size());
		tx_try=0;
		tx_error=bytes_udp;
	}
	else
	{
		if(tx_try>5) //el dato no se envio completo por varios intentos
				{
			u_1=this->getU();
			std::copy ( u_1.begin(), u_1.end(), trama.begin()+3+y.size());
			this->tx_error=-20;
			//ROS_INFO("ALARMA. trying to send a new data ");
			errores+=tx_try;
			tx_try=0;
				}
		else //dato no confirmado
		{
			std::copy ( u_1.begin(), u_1.end(), trama.begin()+3+y.size());
			tx_error=bytes_udp;
		}

	}

			trama[0]=checksum;
			trama[1]=tx_error;
  
	print_trama("sent");
	        std::vector<unsigned char> a=prot.maencode(trama,tipodato);
			
			
			
			ds_uint16_t  temp;
			temp.int16_r=crc16_ccitt(&a[6],bytes_udp-6);
			a[5]=temp.uint16_r.byte1;
			a[6]=temp.uint16_r.byte0;
			
			data_tx=&a[0];
		/*	std::cout<< std::internal
			<<std::setfill('0'); // fill between the prefix and the number
						
 			
			std::cout<<std::hex<<chk<<"\n";
				for(unsigned int i=6;i<bytes_udp;i++){
		           std::cout<<std::setw(2)<<std::hex<<(unsigned short)a[i];
	             }
				std::cout<<std::dec<<"\n***"<<2*(bytes_udp-6)<<"*****\n\n";*/
			//std::size_t length_out = a.size();
///TODO: ingresar lo recibido  si fue valido para que llegue a MA con los mismos bits
 	if(this->rx_error==bytes_udp){
  		for(unsigned int i=5;i<5+8*y.size();i++)
  		data_tx[i]=data_rx[i];

 }
			socket_server.async_send_to(
					boost::asio::buffer(data_tx, a.size()),recieved_endpoint_,boost::bind(&maudp::handle_tx, this,
					boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));

			
}
void maudp::handle_timer(const boost::system::error_code& error)
{
	
 if (error != boost::asio::error::operation_aborted)
  {
 	
	//if (deadline_->expires_at() <= deadline_timer::traits_type::now())
	//{	
		_timeout_triggered=true;		
		deadline_->expires_at(boost::posix_time::pos_infin);
	//}
 }	

}

const std::string& maudp::run(){
	io_service_.reset();
	message.clear();
	_message_received=false;	
	_timeout_triggered=false;
	send_toMA();
	while (io_service_.run_one())
	{
   		 if (_message_received)
    		{
       		   deadline_->cancel();
		   _timeout_triggered=false;
		   _message_received=false;
    		}
    		else if (_timeout_triggered)
    		{
        		//ROS_INFO("Timeout waiting request. errors %d",errores);

			_message_received=false;
			_timeout_triggered=false;

			errores++;
			message.append("ALARM. Timeout making the request to MA ");
			message.append(toString<int>(errores));
			socket_server.cancel();
			//break;
    		}
	}
	io_service_.reset();
		return message;
	}
void maudp::print_trama(const char* t)
{
#ifdef DEBUG

	std::cout<<t<<"\n";
	std::cout<<trama[0]<<'\t'<<trama[1]<<'\t'<<trama[2]<<'\n';
	for(unsigned int i=3;i<y.size()+3;i++){
		std::cout<<trama[i]<<'\t';
	}
	std::cout<<"\n";
	for(unsigned int i=3+y.size();i<trama.size();i++){
		std::cout<<trama[i]<<'\t';
	}
	std::cout<<"\n********\n\n";
#endif
}

unsigned short maudp::crc16_ccitt(const unsigned char *buf, int len)
{
	register int counter;
	register unsigned short crc = 0;
	for( counter = 0; counter < len; counter++)
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(char *)buf++)&0x00FF];
	return crc;
}
} /* namespace microautobox */
