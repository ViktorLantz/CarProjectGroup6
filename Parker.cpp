									//PARKER FOR SIMULATION
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

            // Moving state machine.
        	if(stageMoving == 0) {
                vc.setSpeed(1);
          		vc.setSteeringWheelAngle(0);
          	}

        	if((stageMoving > 0) && (stageMoving < 10)) {
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
          		// Go backwards to center vehicle
          		vc.setSpeed(-1);
          		vc.setSteeringWheelAngle(0);
          		stageMoving++;
          	}

          	if ((stageMoving >= 132) && (stageMoving < 135)) {
          		// Stop.
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
                        	stageMeasuring = 1;
                        	absPathEnd = vd.getAbsTraveledPath();                        
                        	
                        	const double GAP_SIZE = (absPathEnd - absPathStart);
                        	cerr << "\nSize = " <<GAP_SIZE << endl;
                        	
                        	if(GAP_SIZE > 0.1){
                          		m_foundGaps.push_back(GAP_SIZE);
                        	}

                        	if((stageMoving < 1) && (GAP_SIZE > 3.5)) {
                          		stageMoving = 1;
                         		parkingSpotFound = true;
                          		cout << "\n     Found Parking Spot";
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

