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

#include <opendavinci/odcore/io/StringListener.h>

// This class will handle the bytes received via a serial link.
class SerialReceiveBytes : public odcore::io::StringListener {

public:
		virtual std::vector<int> getUSVector();
    	virtual std::vector<int> getIRVector();
    // Your class needs to implement the method void nextString(const std::string &s).
	virtual void nextString(const std::string &s);

};

