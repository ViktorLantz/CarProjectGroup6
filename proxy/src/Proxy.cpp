/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
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

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <string>
#include <memory>
#include <sstream>

#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

#include "OpenCVCamera.h"

//#ifdef HAVE_UEYE
  //  #include "uEyeCamera.h"
//#endif

#include "Proxy.h"
#include "SerialReceiveBytes.hpp"

//  << MODIFIED >>
SerialReceiveBytes receiveHandler;
namespace automotive {
    namespace miniature {
        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;
        using namespace odcore::wrapper;


        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera()
	{}


        Proxy::~Proxy() {
        }

        void Proxy::setUp() {

            // This method will be call automatically _before_ running body().
            if (getFrequency() < 20) {
                cerr << endl << endl << "Proxy: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
            }



            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Create built-in recorder.
            const bool useRecorder = kv.getValue <uint32_t> ("proxy.useRecorder") == 1;
            if (useRecorder) {
                // URL for storing containers.
                stringstream recordingURL;
                recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS() << ".rec";
                // Size of memory segments.
                const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue <uint32_t> ("global.buffer.memorySegmentSize");
                // Number of memory segments.
                const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue <uint32_t> ("global.buffer.numberOfMemorySegments");
                // Run recorder in asynchronous mode to allow real-time recording in background.
                const bool THREADING = true;
                // Dump shared images and shared data?
                const bool DUMP_SHARED_DATA = getKeyValueConfiguration().getValue <uint32_t> ("proxy.recorder.dumpshareddata") == 1;

                m_recorder = unique_ptr <Recorder> (new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));
            }

            // Create the camera grabber.
            const string NAME = getKeyValueConfiguration().getValue <string> ("proxy.camera.name");
            string TYPE = getKeyValueConfiguration().getValue <string> ("proxy.camera.type");
            std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
            const uint32_t ID = getKeyValueConfiguration().getValue <uint32_t> ("proxy.camera.id");
            const uint32_t WIDTH = getKeyValueConfiguration().getValue <uint32_t> ("proxy.camera.width");
            const uint32_t HEIGHT = getKeyValueConfiguration().getValue <uint32_t> ("proxy.camera.height");
            const uint32_t BPP = getKeyValueConfiguration().getValue <uint32_t> ("proxy.camera.bpp");

            if (TYPE.compare("opencv") == 0) {
                m_camera = unique_ptr <Camera> (new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP));
            }
            if (TYPE.compare("ueye") == 0) {
#ifdef HAVE_UEYE
                m_camera = unique_ptr <Camera> (new uEyeCamera(NAME, ID, WIDTH, HEIGHT, BPP));
#endif
            }

            if (m_camera.get() == NULL) {
                cerr << "No valid camera type defined." << endl;
            }
        }

        void Proxy::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        void Proxy::distribute(Container c) {
            // Store data to recorder.
            if (m_recorder.get() != NULL) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                m_recorder->store(c);
            }

            // Share data.
            getConference().send(c);
        }


        //  << MODIFIED >>

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {

            uint32_t captureCounter = 0;
            uint32_t sendCounter = 0;

            //Define Serial port connection for Arduino
            const string SERIAL_PORT = "/dev/ttyACM0";
            //Define BAUD_Rate for serial connection
            const uint32_t BAUD_RATE = 9600;
            // Create connection to the serail port
            std::shared_ptr <SerialPort> serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
            // Link connection to the handler
            serial->setStringListener(&receiveHandler);
            // Start listening for the data on the serial port.
            serial->start();


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // Capture frame.
                if (m_camera.get() != NULL) {
                    odcore::data::image::SharedImage si = m_camera->capture();

                    Container c(si);
                    distribute(c);
                    captureCounter++;
                }

                //Get US and IR data
                uint32_t numOfSensors = 5;
                sendCounter++;

                Container containerControlData = getKeyValueDataStore().get(automotive::VehicleControl::ID());
                VehicleControl vc = containerControlData.getData <VehicleControl> ();

                double speedVal = vc.getSpeed();
                double steeringVal = vc.getSteeringWheelAngle();


                //cout << "Value of Speed :" << speedVal << endl;
                //cout << "Value of Steering :" << steeringVal << endl;
                // IF WE WANT TO SEND THE VALUES AS STRING USE THIS
                stringstream steering_str;
                stringstream speed_str;

                speed_str << speedVal;
                steering_str << steeringVal;

                string speedString = speed_str.str();
                string steeringString = steering_str.str();

                if (sendCounter > 200) {
                    serial->send("{" + speedString + "," + steeringString + "}");
                }

                vector <int> usVector = receiveHandler.getUSVector();
                vector <int> irVector = receiveHandler.getIRVector();

                map <uint32_t, double> values;

                for(int i = 0; i < 2; i++) {
                   //cout << "Values of US: " << usVector[i] << endl;
                    values[i+3] = usVector[i];
                }

                for(int i = 0; i < 3; i++) {
                    //cout << "Values of IR: " << irVector[i] << endl;
                    values[i] = irVector[i];
                }

                SensorBoardData sensorBoard(numOfSensors, values);

                Container sensorContainer(sensorBoard);
                distribute(sensorContainer);
    	    }

        // Stop listening to the serial port
        // IF WE WANT TO SEND STOP SIGNALS AFTER THE CONNECTION IS SHUT DOWN
        /*
        for(int i = 0; i < 10; i++){
            serial->send("{" + "0" + "," + "0" + "}");
        }*/

        serial->stop();
        serial->setStringListener(NULL);

        cout << "Proxy: Captured " << captureCounter << " frames." << endl;

        return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

