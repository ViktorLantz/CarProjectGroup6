/**
 * Parker is an example application to demonstrate how to 
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
			//double back_distance = 0;

			

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                // Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();


                //	back_distance = sbd.getValueForKey_MapOfDistances(1);

                /*
                if((back_distance > 1.5) && (back_distance < 2)){
                			vc.setSpeed(0);
                			cout << "\n STOP THE CAR \n";
                			stageMeasuring++;
                		}
                */
               
                // Create vehicle control data.
                VehicleControl vc;
                // Moving state machine.
                if(stageMoving == 0) {
                	//Put the speed to 1 and listen to the most recent steering data
                	vc.setSpeed(1);
                	vc.setSteeringWheelAngle(0);
                }
                if((stageMoving > 0) && (stageMoving < 10)) {
                	//Move slowly forward
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

				if((stageMoving >= 100) && (stageMoving < 103)) {
					// Stop
					vc.setSpeed(0);
					vc.setSteeringWheelAngle(-25);
					stageMoving++;
				}

				if((stageMoving >= 103) && (stageMoving < 127)) {
					// Turn right to straighten up while going forward
					vc.setSpeed(1);
					vc.setSteeringWheelAngle(25);
					stageMoving++;
				}

				if((stageMoving >= 127) && (stageMoving < 137)){
					// Go slightly foward to finalize the parking
					vc.setSpeed(1);
					vc.setSteeringWheelAngle(0);
					stageMoving++;
				}
                if ((stageMoving >= 137) && (stageMoving < 147)) {
                    // Stop.
                    vc.setSpeed(0);
                    vc.setSteeringWheelAngle(0);
                    stageMoving++;
                }
                if (stageMoving >= 147	) {
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
                				//cout << "\n			Entered CASE 0";
                			}
                	}
                	break;
               		case 1:
           				{
           					//cout << "\n			Entered CASE 1";
               				// Checking the distance sequece +, -.


               				if(parkingSpotFound == false){
               					if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(2) < 0)) {
               						//Found distance sequence -, +.
               						stageMeasuring = 2;
               						absPathStart = vd.getAbsTraveledPath();
               						//cout << "			Entered CASE 1, IF-1";
               					}
               				}
               				distanceOld = sbd.getValueForKey_MapOfDistances(2);
               			}
               		break;
               		case 2:
               			{
               				//Ckecking for distance sequence -, +.
               				if(parkingSpotFound == false){

               				if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(2) > 0)) {
               					//Found distance sequence -, +.
               					stageMeasuring = 1;
               					absPathEnd = vd.getAbsTraveledPath();
               					//cout << "\n			Entered CASE 2, IF-1";
               					
               					
               					const double GAP_SIZE = (absPathEnd - absPathStart);
               					cerr << "\nSize = " <<GAP_SIZE << endl;
               					m_foundGaps.push_back(GAP_SIZE);

               					if((stageMoving < 1) && (GAP_SIZE > 4)) {
               						stageMoving = 1;
               						parkingSpotFound = true;
               						cout << "\n Found Parking Spot \n";
               					}
               				}	
               				
               				}
               				distanceOld = sbd.getValueForKey_MapOfDistances(2);	
               			}
               		break;
               	}
                // You can also turn on or off various lights:
                vc.setBrakeLights(false);
                vc.setFlashingLightsLeft(false);
                vc.setFlashingLightsRight(true);

                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

