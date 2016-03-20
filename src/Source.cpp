#include <iostream>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "OpenCVKinect.h"


#include "Aria.h"
#include "ArVideo.h"
#include "KinectArVideoServer/ArVideoOpenCV.h"
#include "ArSystemStatus.h" 


using namespace cv;

#include <opencv2/highgui/highgui.hpp>

ArGripper gripper(NULL);

void OpenGripper() {
	printf("open gripper request");
	gripper.gripOpen();
}

void CloseGripper() {
	printf("close gripper request");
	gripper.gripClose();
}

void StopGripper() {
	printf("close gripper request");
	gripper.gripStop();
}

ArGlobalFunctor openGripper_f(&OpenGripper);
ArGlobalFunctor closeGripper_f(&CloseGripper);
ArGlobalFunctor stopGripper_f(&StopGripper);

int main(int argc, char *argv[])
{

	const std::string face_cascade_file = "C:\\Users\\Gabriel\\Desktop\\RoboHacks\\OpenCV\\opencv\\sources\\data\\haarcascades_GPU\\haarcascade_frontalface_alt.xml";

	OpenCVKinect kinect;
	kinect.setMode(C_MODE_COLOR | C_MODE_DEPTH | C_MODE_ALIGNED);
	
	if (!kinect.init()) {
		return 0;
	}

	Aria::init();
	ArVideo::init();


	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobot robot;
	//Set up the simple connector
	ArRobotConnector robotConnector(&argParser, &robot);

	ArAnalogGyro gyro(&robot);

	if (!robotConnector.connectRobot()) {
		printf("Could not connect to robot... exiting\n");
		Aria::exit(1);
	}

	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed()) {
		Aria::logOptions();
		Aria::exit(-1);
	}

	ArDataLogger dataLogger(&robot, "dataLog.txt");
	dataLogger.addToConfig(Aria::getConfig());


	ArServerBase server;
	ArServerSimpleOpener openServer(&argParser);

	ArClientSwitchManager clientSwitchManager(&server, &argParser);

	/* Set up ArNetworking services */
	ArServerHandlerCommands commandsServer(&server);

	
	//gripper = ArGripper(&robot);
	//commandsServer.addCommand("Open Gripper", "Open the gripper fam", (ArFunctor*)&openGripper_f, "Gripper");
	//commandsServer.addCommand("Close Gripper", "Close the gripper fam", (ArFunctor*)&closeGripper_f, "Gripper");
	//commandsServer.addCommand("Stop Gripper", "Stop the gripper fam", (ArFunctor*)&stopGripper_f, "Gripper");

	char fileDir[1024];
	ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(),
		"ArNetworking/examples");

#ifndef WIN32
	ArServerFileLister fileLister(&server, ".");
	ArServerFileToClient fileToClient(&server, ".");
	ArServerDeleteFileOnServer deleteFileOnServer(&server, ".");
#endif

	ArServerInfoStrings stringInfoServer(&server);

	Aria::getInfoGroup()->addAddStringCallback(stringInfoServer.getAddStringFunctor());
	ArSystemStatus::startPeriodicUpdate();
	Aria::getInfoGroup()->addStringDouble(
		"CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
	//  Aria::getInfoGroup()->addStringUnsignedLong(
	//    "Computer Uptime", 14, ArSystemStatus::getUptimeFunctor());
	//  Aria::getInfoGroup()->addStringUnsignedLong(
	//    "Program Uptime", 14, ArSystemStatus::getProgramUptimeFunctor());
	Aria::getInfoGroup()->addStringInt(
		"Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
	Aria::getInfoGroup()->addStringInt(
		"Wireless Noise", 10, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
	Aria::getInfoGroup()->addStringInt(
		"Wireless Signal", 10, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");
	Aria::getInfoGroup()->addStringInt(
		"Motor Packet Count", 10,
		new ArConstRetFunctorC<int, ArRobot>(&robot,
		&ArRobot::getMotorPacCount));
	ArServerHandlerCommMonitor commMonitorServer(&server);

	ArVideoOpenCV slideshow("projectJacob_Stream");
	ArVideo::createVideoServer(&server, &slideshow, "projectJacob_Stream", "projectJacob_Stream");

	if (!openServer.open(&server))
	{
		std::cout << "error opening ArNetworking server" << std::endl;
		Aria::exit(5);
		return 5;
	}
	server.runAsync();
	std::cout << "ArNetworking server running on port " << server.getTcpPort() << std::endl;

	ArSonarDevice sonarDev;
	robot.addRangeDevice(&sonarDev);

	ArIRs irs;
	robot.addRangeDevice(&irs);

	ArBumpers bumpers;
	robot.addRangeDevice(&bumpers);

	// attach services to the server
	ArServerInfoRobot serverInfoRobot(&server, &robot);
	ArServerInfoSensor serverInfoSensor(&server, &robot);
	ArServerInfoDrawings drawings(&server);


	// modes for controlling robot movement
	ArServerModeStop modeStop(&server, &robot);
	ArServerModeRatioDrive modeRatioDrive(&server, &robot);
	ArServerModeWander modeWander(&server, &robot);
	modeStop.addAsDefaultMode();
	modeStop.activate();

	// set up the simple commands
	//ArServerHandlerCommands commands(&server);
	ArServerSimpleComUC uCCommands(&commandsServer, &robot);  // send commands directly to microcontroller
	ArServerSimpleComMovementLogging loggingCommands(&commandsServer, &robot); // control debug logging
	ArServerSimpleComGyro gyroCommands(&commandsServer, &robot, &gyro); // configure gyro
	ArServerSimpleComLogRobotConfig configCommands(&commandsServer, &robot); // control more debug logging
	ArServerSimpleServerCommands serverCommands(&commandsServer, &server); // control ArNetworking debug logging
	//ArServerSimpleLogRobotDebugPackets logRobotDebugPackets(&commandsServer, &robot, ".");  // debugging tool

	ArServerHandlerConfig serverHandlerConfig(&server, Aria::getConfig()); // make a config handler
	ArLog::addToConfig(Aria::getConfig()); // let people configure logging

	modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings"); // able to configure teleop settings
	modeRatioDrive.addControlCommands(&commandsServer);

	robot.runAsync(true);

	robot.lock();
	robot.enableMotors();
	robot.unlock();

	drawings.addRobotsRangeDevices(&robot);


	//openni::Device dev;

	//openni::OpenNI::initialize();

	//dev.open(openni::ANY_DEVICE);
	//openni::VideoStream vStream;
	//vStream.create(dev, openni::SENSOR_IR);
	//vStream.start();
	//openni::VideoFrameRef vfRef;

	//openni::VideoStream** mstreams = new openni::VideoStream*[1];
	//mstreams[0] = &vStream;
	//int current_stream;

	cv::vector<cv::Mat> Data;

	cvNamedWindow("Capture", CV_WINDOW_AUTOSIZE);

	CascadeClassifier face_cascade, face2_cascade;
	face_cascade.load(face_cascade_file);

	cv::Mat grayscale;
	//cv::Mat bufferImage;
	for (;;) {

		cv::Mat frame;
		Data = kinect.getData();
		cv::Mat full = Data[C_COLOR_STREAM];
		//openni::Status status = openni::OpenNI::waitForAnyStream(mstreams, 1, &current_stream, 2000);

		//if (status != openni::STATUS_OK) continue;

		//vStream.readFrame(&vfRef);
		//cv::Mat full;
		//full.create(vfRef.getHeight(), vfRef.getWidth(), CV_8UC3);

		//full.create(vfRef.getHeight(), vfRef.getWidth(), CV_8UC3);
		//bufferImage.create(vfRef.getHeight(), vfRef.getWidth(), CV_8UC3);
		//bufferImage.data = (uchar*)vfRef.getData();

		//cvtColor(bufferImage, full, CV_BGR2RGB);

		if (full.size.p[0] != 0) {
			resize(full, frame, Size(300, 300));
			cvtColor(frame, grayscale, CV_BGR2GRAY);
			equalizeHist(grayscale, grayscale);

			std::vector<Rect> cascades;
			face_cascade.detectMultiScale(grayscale, cascades, 1.1, 3, CV_HAAR_SCALE_IMAGE);

			//fprintf_s(stdout, "%i faces detected by filter 1 FAM\n", cascades.size());

			for (auto it = cascades.cbegin(); it != cascades.cend(); ++it) {
				Point pt1(it->x + it->width, it->y + it->height);
				Point pt2(it->x, it->y);

				rectangle(frame, pt1, pt2, cvScalar(0, 255, 0, 0), 1, 8, 0);
			}

			slideshow.updateVideoDataCopy(frame, 1, CV_BGR2RGB);

			imshow("Capture", frame);
		}
		
		if (waitKey(30) >= 0) break;

	}

	//vStream.destroy();
	robot.waitForRunExit();

	return 1;
}

/*

#include <iostream>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace openni;
int main(int argc, char** argv)
{
	//YOOOOOOOOOOOO ITS FUCKING3AM MMAMSDMASD NIGGAGAAGAGGAAGAG

	const std::string face_cascade_file = "C:\\Users\\Gabriel\\Desktop\\RoboHacks\\OpenCV\\opencv\\sources\\data\\haarcascades_GPU\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier face_cascade;
	face_cascade.load(face_cascade_file);

	Device device;

	VideoStream ir;       // VIDEO STREAM IR SHIT CLASS OBJ
	VideoFrameRef irf;    //VIDEO FRAME IR SHIT CLAS OBJ
	VideoMode vmode;
	Status rc = STATUS_OK;
	//LOL WHO THE FUCK COMMETNTS
	rc = openni::OpenNI::initialize();    // Gabe you can understand this intialization youre not an idiot
	rc = device.open(openni::ANY_DEVICE); // This openz the deivce
	rc = ir.create(device, SENSOR_IR);    // IR vid stream made here
	rc = ir.start();                      // If you need an expl. here ur benched


	//I got this shit from google
	Mat frame;				// OpenCV Matrix Object, also used to store images
	int h, w;				// Height and Width of the IR VideoFrame
	//End where I got from google

	namedWindow("ir", CV_WINDOW_AUTOSIZE);		// Create a named window

	while (true)
	{
		if (device.getSensorInfo(SENSOR_IR) != NULL)
		{
			rc = ir.readFrame(&irf);		// Reads one IR vidframe at a time
			if (irf.isValid())
			{
				vmode = ir.getVideoMode();
				//resolution, fps, streamformat
				const uint16_t* imgBuf = (const uint16_t*)irf.getData();
				size_t sz=irf.getDataSize();
				// primesense gives the IR stream as 16-bit data output
				h = irf.getHeight();  //this is pretty much your old code
				w = irf.getWidth();
				frame.create(h, w, CV_16U); // create the OpenCV Mat Matrix Class Object 
				// to receive the IR VideoFrames

				//The rest comes from some indian guy
				memcpy_s(frame.data, sz, imgBuf, sz);

				std::vector<Rect> cascades;
				face_cascade.detectMultiScale(frame, cascades, 1.1, 3, CV_HAAR_SCALE_IMAGE, Size(30, 30));

				fprintf_s(stdout, "%i faces detected FAM\n", cascades.size());

				for (auto it = cascades.cbegin(); it != cascades.cend(); ++it) {
					Point pt1(it->x + it->width, it->y + it->height);
					Point pt2(it->x, it->y);

					rectangle(frame, pt1, pt2, cvScalar(0, 255, 0, 0), 1, 8, 0);
				}

				// Copy the ir data from memory imgbuf -> frame.data 
				// using memcpy (a string.h) function
				//frame.convertTo(frame, CV_8U);
				// OpenCV displays 8-bit data (I'm not sure why?) 
				// So, convert from 16-bit to 8-bit
				try {
					imshow("ir", frame);		// Show the IR VideoFrame in this window
				}
				catch (std::exception ex) {
					std::cout << "an error occured while displaying the frame" << std::endl;
				}
				char key = waitKey(10);
				if (key == 27) break;			//keynum escape 

				//End Indian guys code
			}
		}
	}

	ir.stop();								//stop
	ir.destroy();
	device.close();							//close device
}

*/