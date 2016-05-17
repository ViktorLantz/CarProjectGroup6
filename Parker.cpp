/*
				        MAIN PARKER FOR CAR
                            HARDCODED
 * Parker is an example application to demonstrate how to 
 * generate driving commands from an application realized
 * with OpenDaVINCI
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
 
        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Parker::body() {
        int stageMoving = 0;
		int sideSensorValueF = 0;         // Side-front IR-sensor
        int sideSensorValueB = 0;         // Side-back IR-sensor
        int sideSensorGapCounterF = 0;    // Counter for side-front IR-sensor
        int sideSensorGapCounterB = 0;    // Counter for side-back IR-sensor
        bool parkingSpotFound = false;    // When true - stops "looking" for a parking spot.
 
        	while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
			
                // Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

				sideSensorValueF = sbd.getValueForKey_MapOfDistances(2); // Side-front IR-sensor 
                sideSensorValueB = sbd.getValueForKey_MapOfDistances(0); // Side-back IR-sensor
                
                // Create vehicle control data.
                VehicleControl vc;

	            if(parkingSpotFound == false){
    	            if(sideSensorValueF > 50){      // Adds to counter when the vehicle detects gap on the side      
        	            sideSensorGapCounterF++;
            	    }
            	}

                if(sideSensorValueF > 50){      // Adds to counter when the vehicle detects a gap on the side      
                    sideSensorGapCounterF++;
                }

                if(sideSensorValueB > 50){      // -- || --
                    sideSensorGapCounterB++;
                }
                
                if(parkingSpotFound == false){ 
                     // Check if the car should start parking
                	if ((sideSensorGapCounterF > 5) && (sideSensorGapCounterB > 5) && sideSensorValueF < 30 && sideSensorValueB < 30 && sideSensorValueF > 5 && sideSensorValueB > 5){
                    	stageMoving++;
                    	parkingSpotFound = true;
                  	}
                }

           /* 	START OF MOVING STATE MACHINE with hardcoded values.
            * 	Every stage has a "resting" stage in between them, this
            * 	is coded like this to give the car time to readjust the
            * 	wheels and to maintain the same behavior as good as possible 
            * 	regardless of the battery level of the car. The first stage is
            * 	the "looking for a spot" stage and differentiates in the way
            * 	it moves on to the next stage compaired to the other stages. 
			*	vc.setSteeringWheelAngle(1) = right
			*	vc.setSteeringWheelAngle(-1) = left
			*	vc.setSteeringWheelAngle(5) = neutral
			*	vc.setSpeed(1) = foward
			*	vc.setSpeed(2) = brake
			*	vc.setSpeed(4) = neutral
			*	vc.setSpeed(-1) = backwards	
            */
				
				// Going forward
                if(stageMoving == 0) {
                	vc.setSpeed(1);
                	vc.setSteeringWheelAngle(5);
                }
				// Brake
                if((stageMoving > 0) && (stageMoving < 2)) {
                	vc.setSpeed(2);
                	vc.setSteeringWheelAngle(5);
                	stageMoving++;
                }
				// Neutral with neutral steering
                if((stageMoving >= 2) && (stageMoving < 40)) {
                	vc.setSpeed(4);
                	vc.setSteeringWheelAngle(5);
                	stageMoving++;
                }
                // Stage when parking starts
				// Neutral steering right 
                if ((stageMoving >= 40) && (stageMoving < 60)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(1);
                    stageMoving++;    
                }
				// Backwards steering right
                if ((stageMoving >= 60) && (stageMoving < 80)) {
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(1);
                    stageMoving++;
                }
				// Neutral steering right
                if ((stageMoving >= 80) && (stageMoving < 100)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(1);
                    stageMoving++;
                }
				// Neutral steering right
                if ((stageMoving >= 100) && (stageMoving < 110)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(1);
                    stageMoving++;
                }
				// Neutral steering left
                if ((stageMoving >= 110) && (stageMoving < 130)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(-1);
                    stageMoving++;
                }
				// Neutral steering left
                if ((stageMoving >= 130) && (stageMoving <150)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(-1);
                    stageMoving++;
                }
				// Backwards steering left
                if ((stageMoving >= 150) && (stageMoving < 170)) {
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-1);
                    stageMoving++;
                }
				// Neutral steering left
                if ((stageMoving >= 170) && (stageMoving < 180)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(-1);
                    stageMoving++;
                }
				// Neutral steering left
                if ((stageMoving >= 180) && (stageMoving < 190)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(-1);
                    stageMoving++;
                }
				// Neutral with neutral steering
                if ((stageMoving >= 190) && (stageMoving < 210)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(5);
                    stageMoving++;
                }
				// Neutral steering right
                if ((stageMoving >= 210) && (stageMoving < 235)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(1);
                    stageMoving++;
                }
				// Forward with neutral steering
                if ((stageMoving >= 235) && (stageMoving < 250)) {
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(5);
                    stageMoving++;
                }
				// Neutral steering right
                if ((stageMoving >= 250) && (stageMoving < 255)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(1);
                    stageMoving++;
                }
				// Neutral with neutral steering
                if ((stageMoving >= 255) && (stageMoving < 265)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(5);
                    stageMoving++;
                }
				// Neutral with neutral steering
                if ((stageMoving >= 265) && (stageMoving < 267)) {
                    vc.setSpeed(4);
                    vc.setSteeringWheelAngle(5);
                    cout << "END COMPONENT" << endl;
                    break;
                }

            //  END OF STATE MACHINE

               	// Create container for finally sending the data.
   			    Container c(vc);

   				// Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        } //automotive::miniature::parker::src

    } //automotive::miniature::parker
} // automotive::miniature

