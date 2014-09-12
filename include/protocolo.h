/*
 * protocolo.h
 * library to encode and decode an array of double in bynary
 * with the possibility to encoding this variable in
 * @ref datos {manull,maboolean,maint8,mauint8,maint16,mauint16,maint32,mauint32,mafloat,madouble}

 *
 * it is based of the example of Dspace  microautobox II to communicate by udp
 *  
 * FILE:
 *   ds867c_eth_encode32_sfcn.c
 *
 * DESCRIPTION:
 * Encoding of data in various input format to 32bit WORD format
 * it should work both in Simulink as well as in RT Application with dSPACE HW 
 * LaszloJ
 * FILE:
 *   ds867c_eth_decode32_sfcn.c
 *
 * DESCRIPTION:
 * decodes the 32bit WORD-coded data into selected data types
 * 
 * Used and changed by LaszloJ 
 *
 * @author olmer Garcia olmerg@gmail.com
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

#ifndef PROTOCOLO_H_
#define PROTOCOLO_H_
#include <vector>
namespace microautobox {
/**
 * types of variables available in the microautobox
 * Datatype Codes:
BOOLEAN:  1   //   INT8      :  2    //   UINT8    :  3
INT16       :  4   //   UINT16 :  5    //   INT32     :  6
UINT32    :  7   //   FLOAT   :  8    //   DOUBLE:  9
 */
	enum datos{manull,maboolean,maint8,mauint8,maint16,mauint16,maint32,mauint32,mafloat,madouble};
	enum { max_length = 1024 };
	enum { bigendian=1, litleendian=2};

	/*template <typename T> int typeofi(T&)  {return manull;}
	template<> int typeofi(bool&) { return maboolean; }
	template<> int typeofi(char&) { return maint8; }
	template<> int typeofi(unsigned char&) { return mauint8; }
	template<> int typeofi(short &) { return maint16; }
	template<> int typeofi(unsigned short&) { return mauint16; }
	template<> int typeofi(int&) { return maint32; }
	template<> int typeofi(unsigned int&) { return mauint32; }
	template<> int typeofi(float&) { return mafloat; }
	template<> int typeofi(double&) { return madouble; }*/

    typedef struct
             {
              unsigned char byte7;
              unsigned char byte6;
              unsigned char byte5;
              unsigned char byte4;
              unsigned char byte3;
              unsigned char byte2;
              unsigned char byte1;
              unsigned char byte0;
             } uint64_by_uint8_t;

	typedef struct
	             {
	               unsigned char byte3;
	               unsigned char byte2;
	               unsigned char byte1;
	               unsigned char byte0;
	             } uint32_by_uint8_t;

	/*===============================*/

	typedef struct
	             {
	               unsigned char byte1;
	               unsigned char byte0;
	             } uint16_by_uint8_t;

	/*===============================*/

	typedef union
	            {
	              uint32_by_uint8_t  uint32_r ;
	              float              float32_r;
	            } float32_t;

	/*===============================*/

	typedef union
	            {
	              uint64_by_uint8_t  uint64_r ;
	              double             float64_r;
	            } float64_t;

	/*===============================*/

	typedef union
	             {
	               uint32_by_uint8_t  uint32_r ;
	               unsigned int       int32_r  ;
	             } ds_uint32_t;

	/*===============================*/

	typedef union
	             {
	               uint16_by_uint8_t  uint16_r ;
	               unsigned short     int16_r  ;
	             } ds_uint16_t;
/**
 * class to encode and decode an array of double in binary
 * with the possibility to encoding this variable in
 * @ref datos {manull,maboolean,maint8,mauint8,maint16,mauint16,maint32,mauint32,mafloat,madouble}
 *
 * it is based of the example of the microautobox II to communicate by udp
 */
class protocolo {
int PROCESSORTYPE;
public:
/**
 *
 * @param PROCESSORTYPE bigendian or litleendian
 */
	protocolo(int PROCESSORTYPE=bigendian);
	virtual ~protocolo();
	/***
	 * @brief function to convert an array of double to unsigned char in binary representation
	 * @param data is vector of double with the data to encode in unsigned char
	 * @param type @ref datos {manull,maboolean,maint8,mauint8,maint16,mauint16,maint32,mauint32,mafloat,madouble}
	 * @return the vector with the data encoded in unsigned char to be transmited in binary form
	 */
	std::vector<unsigned char>  maencode(std::vector<double> data,std::vector<int> type);
	/***
	 *
	 * @param data
	 * @param tipe @ref datos {manull,maboolean,maint8,mauint8,maint16,mauint16,maint32,mauint32,mafloat,madouble}
	 * @return
	 */

	std::vector<double> decode(std::vector<unsigned char> data,std::vector<int> tipe);

};
}
#endif /* PROTOCOLO_H_ */
