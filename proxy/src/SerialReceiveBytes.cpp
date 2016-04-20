/**
 * OpenDaVINCI - Tutorial.
 * Copyright (C) 2015 Christian Berger
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <sstream>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "SerialReceiveBytes.hpp"

// We add some of OpenDaVINCI's namespaces for the sake of readability.
using namespace std;
using namespace odcore;
using namespace odcore::wrapper;

std::string buffer_container("");

vector<int> us_vector_compiler(int i, int j);
vector<int> ir_vector_compiler(int i, int j, int k);

vector<int> us_vector(2);
vector<int> ir_vector(3);

vector<int> SerialReceiveBytes::getIRVector(){
    return ir_vector;
}

vector<int> SerialReceiveBytes::getUSVector(){
    return us_vector;
}

void SerialReceiveBytes::nextString(const string &str) {
    cout << "Received " << str.length() << " bytes containing '" << str<< "'" << endl;

    std::string u_str("u");
    std::string i_str("i");
    std::string startD_str("{");
    std::string endD_str("}");
    std::string startSQ_str("[");
    std::string endSQ_str("]");

    // Add the arduino sensor data to buffer container string
    buffer_container += str;
    cout << "Buffer_TEST: " << buffer_container << endl;
    // Keep adding to string until we have a complete sensor packet (Both end delimiters present)
    while (buffer_container.find(startSQ_str) != string::npos && buffer_container.find(endSQ_str) != string::npos) {

    // Check if start delimiter for us sensor exists
		if (buffer_container.find(startD_str) != string::npos) {
    		//cout << "ping_1st_if" << endl;
    // Check string for the 'u' character
    		if (buffer_container.find(u_str) != string::npos) {
    			//cout << "ping_2nd_if" << endl;

    // Take the values from position 3, 3 characters forward.
    			std::string firstVal = buffer_container.substr(3,3);
    			//cout << "PING first value in body: " << firstVal << endl;
    // Find position of the first delimiter ',' and get the next sensory value
                std::size_t comma_str = buffer_container.find(",");
    			std::string secondVal = buffer_container.substr(comma_str+1, 3);
    			//cout << "PING2 second value in body: " << secondVal << endl;


    			std::stringstream first_String(firstVal);
    			std::stringstream second_String(secondVal);
    // Take the integers from the string and put them into an int variable
    // (THIS MAY NEED TO BE CHANGED TO DOUBLES)
   				int value1, value2;
    			first_String >> value1;
    			second_String >> value2;


    // Send the values to the compiler.
    			vector <int> test;
    			us_vector_compiler(value1, value2);

    		}

    		if (buffer_container.find(i_str) != string::npos) {

                //std::size_t comma_str = buffer_container.find(",");
                std::size_t first_i_str = buffer_container.find("i");
    			std::string firstIRVal = buffer_container.substr(first_i_str+1,3);
    			//cout << "PING first value in body for ir: " << firstIRVal << endl;

    			//std::size_t comma_str = buffer_container.find(",");
                std::size_t first_delim_str = buffer_container.find("%");
    			std::string secondIRVal = buffer_container.substr(first_delim_str+1,3);
    			//cout << "PING second value in body for ir: " << secondIRVal << endl;

    			//std::size_t comma_str = buffer_container.find(",");
                std::size_t second_delim_str = buffer_container.find("&");
    			std::string thirdIRVal = buffer_container.substr(second_delim_str+1,3);
    			//cout << "PING third value in body for ir: " << thirdIRVal << endl;

                std::stringstream first_IR_String(firstIRVal);
    			std::stringstream second_IR_String(secondIRVal);
    			std::stringstream third_IR_String(thirdIRVal);

   				int value1, value2, value3;

    			first_IR_String >> value1;
    			second_IR_String >> value2;
    			third_IR_String >> value3;

    			vector <int> test;
    			ir_vector_compiler(value1, value2, value3);

    		}
    		buffer_container = "";
    	}
	}
    //cout << "PING: " << value << endl;


}

vector<int> us_vector_compiler(int i, int j) {
	vector <int> us_vec;
	us_vec.push_back(i);
	//cout << "PING2_first us value in vector compiler: " << us_vec[0] << endl;
	us_vec.push_back(j);
	//cout << "PING2_second us value in vector compiler: " << us_vec[1] << endl;

	us_vector = us_vec;

	return us_vec;
}
vector<int> ir_vector_compiler(int i, int j, int k){
    vector <int> vec;
    vec.push_back(i);
	//cout << "PING2_first IR in vector compiler: " << vec[0] << endl;
	vec.push_back(j);
	//cout << "PING2_Second IR in vector compiler: " << vec[1] << endl;
	vec.push_back(k);
	//cout << "Ping2_Third IR in vector compiler: " << vec[2] << endl;

	ir_vector = vec;

	return vec;
}




