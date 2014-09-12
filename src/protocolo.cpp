/*
 * protocolo.h
 * library to encode and decode an array of double in bynary
 * with the possibility to encoding this variable in
 * @ref datos   {manull,maboolean,maint8,mauint8,maint16,mauint16,maint32,mauint32,mafloat,madouble}
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
#include "protocolo.h"
#include <vector>
namespace microautobox {
protocolo::protocolo(int C_PROCESSORTYPE){
	PROCESSORTYPE=C_PROCESSORTYPE;
}

protocolo::~protocolo() {
	// TODO Auto-generated destructor stub
}


std::vector<unsigned char> protocolo::maencode(std::vector<double> data,std::vector<int> tipe) {
	std::vector<unsigned char> buff;
	  ds_uint16_t  temp;
	  ds_uint32_t  temp0;
	  float32_t temp1;
	  float64_t temp2;
	if (PROCESSORTYPE == 1) /* big-endian*/
	  {
	    for(unsigned int i=0; i < data.size() ; i++)
	    {
		    switch(tipe[i])
		    {
		    case maboolean:
		    	if(data[i]==0)
		    		buff.push_back(0);
		    	else
		    		buff.push_back(1);
		    	break;
		    case maint8:
		    case mauint8:
		    	buff.push_back((unsigned char)data[i]);
		    	break;
		    case maint16:
		   	case mauint16:
		   		temp.int16_r = (unsigned short) data[i];
		   	    	buff.push_back(temp.uint16_r.byte1);
		   	    	buff.push_back(temp.uint16_r.byte0);
		   	    break;
		   	case maint32:
		   	case mauint32:
		   		temp0.int32_r=(unsigned int) data[i];
		   		buff.push_back(temp0.uint32_r.byte3);
		   		buff.push_back(temp0.uint32_r.byte2);
		   		buff.push_back(temp0.uint32_r.byte1);
		   		buff.push_back(temp0.uint32_r.byte0);
		   	break;
		  	case mafloat:
			   		temp1.float32_r=(float) data[i];
			   		buff.push_back(temp1.uint32_r.byte3);
			   		buff.push_back(temp1.uint32_r.byte2);
			   		buff.push_back(temp1.uint32_r.byte1);
			   		buff.push_back(temp1.uint32_r.byte0);
			   		break;
		  	case madouble:
			   		temp2.float64_r=(float) data[i];
			   		buff.push_back(temp2.uint64_r.byte7);
			   		buff.push_back(temp2.uint64_r.byte6);
			   		buff.push_back(temp2.uint64_r.byte5);
			   		buff.push_back(temp2.uint64_r.byte4);
			   		buff.push_back(temp2.uint64_r.byte3);
			   		buff.push_back(temp2.uint64_r.byte2);
			   		buff.push_back(temp2.uint64_r.byte1);
			   		buff.push_back(temp2.uint64_r.byte0);
			   		break;
		    }
	    }

	  }
	else  /* litle-endian*/
	  {
	    for(unsigned int i=0; i < data.size() ; i++)
	    {
	    switch (tipe[i])
	    {
	    case (int)maboolean:
	    	if(data[i]==0)
	    		buff.push_back(0);
	    	else
	    		buff.push_back(1);
	    	break;
	    case maint8:
	    case mauint8:
	    	buff.push_back((unsigned char)data[i]);
	    	break;
	    case maint16:
	   	case mauint16:
	   		temp.int16_r = (unsigned short) data[i];
	   	    	buff.push_back(temp.uint16_r.byte1);
	   	    	buff.push_back(temp.uint16_r.byte0);
	   	    break;
	   	case maint32:
	   	case mauint32:
	   		temp0.int32_r=(unsigned int) data[i];
	   		buff.push_back(temp0.uint32_r.byte0);
	   		buff.push_back(temp0.uint32_r.byte1);
	   		buff.push_back(temp0.uint32_r.byte2);
	   		buff.push_back(temp0.uint32_r.byte3);
	   	break;
	  	case mafloat:
		   		temp1.float32_r=(float) data[i];
		   		buff.push_back(temp1.uint32_r.byte0);
		   		buff.push_back(temp1.uint32_r.byte1);
		   		buff.push_back(temp1.uint32_r.byte2);
		   		buff.push_back(temp1.uint32_r.byte3);
		   		break;
	  	case madouble:
		   		temp2.float64_r=(float) data[i];
		   		buff.push_back(temp2.uint64_r.byte0);
		   		buff.push_back(temp2.uint64_r.byte1);
		   		buff.push_back(temp2.uint64_r.byte2);
		   		buff.push_back(temp2.uint64_r.byte3);
		   		buff.push_back(temp2.uint64_r.byte4);
		   		buff.push_back(temp2.uint64_r.byte5);
		   		buff.push_back(temp2.uint64_r.byte6);
		   		buff.push_back(temp2.uint64_r.byte7);
		   		break;
	    }
	    }

	  }

	return buff;

}

std::vector<double> protocolo::decode(std::vector<unsigned char> data,std::vector<int> tipe)
{
	std::vector<double> buff;

	 ds_uint16_t  temp;
		  ds_uint32_t  temp0;
		  float32_t temp1;
		  float64_t temp2;
		  unsigned int offset=0;
		if (PROCESSORTYPE == 1) /* big-endian*/
		  {
		    for(unsigned int i=0; i <tipe.size() ; i++)
		    {
			    switch(tipe[i])
			    {
			    case maint8:
			    case maboolean:
			    		buff.push_back((int)data[offset]);
			    		++offset;
			    	break;
			    case mauint8:
			    	buff.push_back((unsigned int)data[offset]);
			    	++offset;
			    	break;
			    case maint16:
			   	    	temp.uint16_r.byte1=data[offset];
			   	    	temp.uint16_r.byte0=data[++offset];
			   	    	++offset;
			   	    	buff.push_back((short)temp.int16_r);
			   	    break;
			   	case mauint16:
			   	 	temp.uint16_r.byte1=data[offset];
			   		temp.uint16_r.byte0=data[++offset];
			   		++offset;
			   		buff.push_back((unsigned short)temp.int16_r);
			   	    break;
			   	case maint32:
			   		temp0.uint32_r.byte3=data[offset];
			   		temp0.uint32_r.byte2=data[++offset];
					temp0.uint32_r.byte1=data[++offset];
					temp0.uint32_r.byte0=data[++offset];
					++offset;
				buff.push_back((int)temp0.int32_r);
				break;
			   	case mauint32:
			   		temp0.uint32_r.byte3=data[offset];
			   		temp0.uint32_r.byte2=data[++offset];
			   	 	temp0.uint32_r.byte1=data[++offset];
			   		temp0.uint32_r.byte0=data[++offset];
			   		++offset;
			   		buff.push_back((unsigned int)temp0.int32_r);
			   	break;
			  	case mafloat:
			  		temp1.uint32_r.byte3=data[offset];
			  		temp1.uint32_r.byte2=data[++offset];
			  		temp1.uint32_r.byte1=data[++offset];
			  		temp1.uint32_r.byte0=data[++offset];
			  		++offset;
			  		buff.push_back((float)temp1.float32_r);
				   		break;
			  	case madouble:
			  		temp2.uint64_r.byte7=data[offset];
			  		temp2.uint64_r.byte6=data[++offset];
			  		temp2.uint64_r.byte5=data[++offset];
			  		temp2.uint64_r.byte4=data[++offset];
			  		temp2.uint64_r.byte3=data[++offset];
			  		temp2.uint64_r.byte2=data[++offset];
			  		temp2.uint64_r.byte1=data[++offset];
			  		temp2.uint64_r.byte0=data[++offset];
			  		++offset;
			  		buff.push_back((double)temp2.float64_r);
				   		break;
			    }
		    }

		  }
		else  /* litle-endian*/
		  {

		    for(unsigned int i=0; i <tipe.size() ; i++)
		    {
			    switch(tipe[i])
			    {
			    case maint8:
			    case maboolean:
			    		buff.push_back((int)data[offset]);
			    		++offset;
			    	break;
			    case mauint8:
			    	buff.push_back((unsigned int)data[offset]);
			    	++offset;
			    	break;
			    case maint16:
			   	    	temp.uint16_r.byte0	=data[offset];
			   	    	temp.uint16_r.byte1	=data[++offset];
			   	    	++offset;
			   	    	buff.push_back((short)temp.int16_r);
			   	    break;
			   	case mauint16:
			   	 	temp.uint16_r.byte0=data[offset];
			   		temp.uint16_r.byte1=data[++offset];
			   		++offset;
			   		buff.push_back((unsigned short)temp.int16_r);
			   	    break;
			   	case maint32:
			   		temp0.uint32_r.byte0=data[offset];
			   		temp0.uint32_r.byte1=data[++offset];
			   		temp0.uint32_r.byte2=data[++offset];
			   		temp0.uint32_r.byte3=data[++offset];
					++offset;
				buff.push_back((int)temp0.int32_r);
				break;
			   	case mauint32:
			   		temp0.uint32_r.byte0=data[offset];
			   			temp0.uint32_r.byte1=data[++offset];
			   			temp0.uint32_r.byte2=data[++offset];
			   		temp0.uint32_r.byte3=data[++offset];
			   		++offset;
			   		buff.push_back((unsigned int)temp0.int32_r);
			   	break;
			  	case mafloat:
			  		temp1.uint32_r.byte0=data[offset];
			  		temp1.uint32_r.byte1=data[++offset];
			  		temp1.uint32_r.byte2=data[++offset];
			  		temp1.uint32_r.byte3=data[++offset];
			  		++offset;
			  		buff.push_back((float)temp1.float32_r);
				   		break;
			  	case madouble:
			  		temp2.uint64_r.byte0=data[offset];
			  		temp2.uint64_r.byte1=data[++offset];
			  		temp2.uint64_r.byte2=data[++offset];
			  		temp2.uint64_r.byte3=data[++offset];
			  		temp2.uint64_r.byte4=data[++offset];
			  		temp2.uint64_r.byte5=data[++offset];
			  		temp2.uint64_r.byte6=data[++offset];
			  		temp2.uint64_r.byte7=data[++offset];
			  		++offset;
			  		buff.push_back((double)temp2.float64_r);
				   		break;
			    }
		    }
		  }

	return buff;
}
}

