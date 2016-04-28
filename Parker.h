/**
 * Parker is an example application to demonstrate how to 
 *         generate parking commands from an application realized
 *         with OpenDaVINCI
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef PARKER_H_
#define PARKER_H_

#include <vector>

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"

namespace automotive {
    namespace miniature {

        using namespace std;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving 	dynamics simulation.
         */
        class Parker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                Parker(const Parker &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                Parker& operator=(const Parker &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                Parker(const int32_t &argc, char **argv);

                virtual ~Parker();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
		
		/**
                 * This method returns the gaps found during the parking process.
                 *
                 * return Gaps found during the parking process.
                 */
                vector<double> getFoundGaps() const;

		
	
            private:
                virtual void setUp();

                virtual void tearDown();
		
	    private:
		vector<double> m_foundGaps;

        };

    }
} // automotive::miniature

#endif /*PARKER_H_*/
