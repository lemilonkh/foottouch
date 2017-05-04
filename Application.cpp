///////////////////////////////////////////////////////////////////////////
//
// Main class for HCI2 assignments
// Authors:
//		Stefan Neubert (2015)
//		Stephan Richter (2011)
//		Patrick L�hne (2012)
//
///////////////////////////////////////////////////////////////////////////

#include "Application.h"

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "framework/DepthCamera.h"
#include "framework/KinectMotor.h"

using namespace std;
using namespace cv;

const int DEFAULT_MOTOR_ANGLE = 180; // TODO figure out the correct angle to the tripod
const int IMAGE_HEIGHT = 480;
const int IMAGE_WIDTH = 640;
const int CROSSHAIR_SIZE = 50;

void Application::processFrame() {
	// Used textures:
	// * m_bgrImage: The image of the Kinect's color camera
	// * m_depthImage: The image of the Kinects's depth sensor
	// * m_outputImage: The image in which you can draw the touch circles

	// thresholding vars
	Mat withoutGround, thresholdedDepth, src;
	double groundThreshold = 255; // TODO figure out correct threshold for floor
	double legThreshold = 200; // TODO figure out correct threshold for leg / higher objects
	double maxValue = 255;

	// Amplify and convert image from 16bit to 8bit
	m_depthImage *= 2500; // 10
	m_depthImage.convertTo(src, CV_8UC1, 1.0/256.0, 0);

	// first thresholding pass (remove ground)
	threshold( src, withoutGround, groundThreshold, maxValue, 1);

	// second thresholding pass (remove leg etc.)
	threshold( withoutGround, thresholdedDepth, legThreshold, maxValue, 1);

	// find outlines
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresholdedDepth, contours, hierarchy, CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// fit ellipses & determine center points
	vector<RotatedRect> minEllipses(contours.size());
	vector<Point2f> centerPoints;

	for(int i = 0; i < contours.size(); i++) {
		if(contours[i].size() > 10) {
			minEllipses[i] = fitEllipse(Mat(contours[i]));
			centerPoints[i] = minEllipses[i].center;
			cout << "inner if" << "\n";
		}
		cout << "outter if" << "\n";
	}

	// draw touch circles into m_outputImage
	for(int i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(255, 0, 255);
		drawContours(m_outputImage, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		ellipse(m_outputImage, minEllipses[i], color, 2, 8);
		// draw center points (using a crosshair => two lines)
		line(m_outputImage,
			centerPoints[i] - Point2f(CROSSHAIR_SIZE, 0),
			centerPoints[i] + Point2f(CROSSHAIR_SIZE, 0),
			color, 1, 8);
		line(m_outputImage,
			centerPoints[i] - Point2f(0, CROSSHAIR_SIZE),
			centerPoints[i] + Point2f(0, CROSSHAIR_SIZE),
			color, 1, 8);
	}

	// TODO remove
	//m_outputImage = withoutGround;
}

void Application::loop() {
	try {
		int key = cv::waitKey(20);
		switch (key) {
		case 'q': // quit
			m_isFinished = true;
			break;
		case 's': // screenshot
			makeScreenshots();
			break;
		}

		m_depthCamera->getFrame(m_bgrImage, m_depthImage);
		processFrame();

		cv::imshow("bgr", m_bgrImage);
		cv::imshow("depth", m_depthImage);
		cv::imshow("output", m_outputImage);
	} catch ( cv::Exception & e ) {
		cerr << e.msg << endl; // output exception message
	}
}

void Application::makeScreenshots() {
	cv::imwrite("color.png", m_bgrImage);
	cv::imwrite("depth.png", m_depthImage);
	cv::imwrite("output.png", m_outputImage);
}

Application::Application() :
	m_isFinished(false),
	m_depthCamera(nullptr),
	m_kinectMotor(nullptr) {

	// connect to Kinect
	try {
		m_depthCamera = new DepthCamera;

		// open windows
		cv::namedWindow("output", 1);
		cv::namedWindow("depth", 1);
		cv::namedWindow("bgr", 1);

	  // create work buffer
		m_bgrImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
		m_depthImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);
		m_outputImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
	} catch ( cv::Exception & e ) {
		cerr << e.msg << endl; // output exception message
	}
}

Application::~Application() {
	if (m_depthCamera) delete m_depthCamera;
	if (m_kinectMotor) delete m_kinectMotor;
}

bool Application::isFinished() {
	return m_isFinished;
}
