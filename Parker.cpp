									//PARKER FOR SIMULATION

/*Parker is an example application to demonstrate how to 
 *         generate driving commands from an application realized
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

/*
	DISCLAMER: 	Code is built from the example code provided by Christian Berger.
*/

#include <cmath>
#include <iostream>
#include <cstdio>
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "Parker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace automotive;

        Parker::Parker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "Parker"), 
            m_foundGaps() {}

        Parker::~Parker() {}

        void Parker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void Parker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }
 
        vector<double> Parker::getFoundGaps() const {
            return m_foundGaps;
        }
    
        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Parker::body() {
        double distanceOld = 0;
        double absPathStart = 0;
        double absPathEnd = 0;
        int stageMoving = 0;
        int stageMeasuring = 0;
        bool parkingSpotFound = false;

        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
			
        	Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
            VehicleData vd = containerVehicleData.getData<VehicleData> ();
            
            Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
            SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

            // Create vehicle control data.
            VehicleControl vc;

           /* 	START OF MOVING STATE MACHINE with hardcoded values.
            * 	The first stage is the "looking for a spot" stage 
			*	and differentiates in the way
            * 	it moves on to the next stages compaired to the other stages.	
            */
        	if(stageMoving == 0) {
				//Moving and looking for gap
                vc.setSpeed(1);
          		vc.setSteeringWheelAngle(0);
          	}

        	if((stageMoving > 0) && (stageMoving < 10)) {
				// Going 10 'tics' forward
                vc.setSpeed(1);
                vc.setSteeringWheelAngle(0);
                stageMoving++;
            }
            
            if ((stageMoving >= 10) && (stageMoving < 15)) {
                // Stop when it has passed the gap
                vc.setSpeed(0);
                vc.setSteeringWheelAngle(0);
                stageMoving++;
            }

        	if ((stageMoving >= 15) && (stageMoving < 70)) {
          		// Go backwards, steering wheel to the right to get the appropriate angle
          		vc.setSpeed(-1);
          		vc.setSteeringWheelAngle(25);
          		stageMoving++;
          	}

          	if((stageMoving >= 70) && (stageMoving < 100)) {
          		// Turn left to straighten up while going backwards
          		vc.setSpeed(-1);
          		vc.setSteeringWheelAngle(-25);
          		stageMoving++;
          	}


          	if((stageMoving >= 100) && (stageMoving < 105)) {
          		// Turn left to straighten up
          		vc.setSpeed(0);
          		vc.setSteeringWheelAngle(-25);
          		stageMoving++;
          	}


        	if((stageMoving >= 105) && (stageMoving < 127)){
          		// Go foward steering to the right
          		vc.setSpeed(1);
          		vc.setSteeringWheelAngle(25);
          		stageMoving++;
          	}

        	if ((stageMoving >= 127) && (stageMoving < 132)) {
          		// Go backwards to make the vehicle stop
          		vc.setSpeed(-1);
          		vc.setSteeringWheelAngle(0);
          		stageMoving++;
          	}

          	if ((stageMoving >= 132) && (stageMoving < 135)) {
          		// Stop and stand still
          		vc.setSpeed(0);
          		vc.setSteeringWheelAngle(0);
          		stageMoving++;
          	}

        	if (stageMoving >= 135) {
          		// End component.
          		break;
        	}

			//Measuring state machine.
                switch (stageMeasuring) {
                  case 0:
                  {
                      //Initialize measurement.
                      if(parkingSpotFound == false){
                        distanceOld = sbd.getValueForKey_MapOfDistances(2);
                        stageMeasuring++;
                      }
                  }
                  break;
                  case 1:
                  {
                      if(parkingSpotFound == false){
                        if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(2) < 0)) {
							//Found start of gap
						  stageMeasuring = 2;
                          absPathStart = vd.getAbsTraveledPath();
                        }
                      }
                      distanceOld = sbd.getValueForKey_MapOfDistances(2);
                    }
                  break;
                  case 2:
                    {
                    if(parkingSpotFound == false){
						
                      	if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(2) > 0)) {
                        	//Found end of gap
							stageMeasuring = 1;
                        	absPathEnd = vd.getAbsTraveledPath();                        
                        	
                        	const double GAP_SIZE = (absPathEnd - absPathStart);
                        	cerr << "\nSize = " <<GAP_SIZE << endl;
                        	
                        	if(GAP_SIZE > 0.1){
                          		m_foundGaps.push_back(GAP_SIZE);
                        	}

                        	if((stageMoving < 1) && (GAP_SIZE > 3.5)) {
								//Parking spot found
                          		stageMoving = 1;
                         		parkingSpotFound = true;
							}
                      	} 
                    }
                    distanceOld = sbd.getValueForKey_MapOfDistances(2);
                    }
                  break;
                }

                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

