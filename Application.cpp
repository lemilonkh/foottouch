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

using namespace cv;

void Application::processFrame() {
	///////////////////////////////////////////////////////////////////////////
	//
	// To do:
	//
	// This method will be called every frame of the camera. Insert code here in
	// order to fulfill the assignment. These images will help you doing so:
	//
	// * m_bgrImage: The image of the Kinect's color camera
	// * m_depthImage: The image of the Kinects's depth sensor
	// * m_outputImage: The image in which you can draw the touch circles.
	//
	///////////////////////////////////////////////////////////////////////////

	// first thresholding pass (remove ground)
	Mat withoutGround;
	unsigned char thresholdValue = 60; // TODO figure out correct threshold for floor
	threshold( m_depthImage, withoutGround, thresholdValue, 255, THRESH_BINARY);

	// second thresholding pass (remove leg etc.)
	Mat thresholdedDepth;
	unsigned char thresholdValue = 128; // TODO figure out correct threshold for leg / higher objects
	threshold( m_depthImage, thresholdedDepth, thresholdValue, 255, THRESH_BINARY);

	// find outlines
	Mat contours;
	vector<Vec4i> hierarchy;
	findContours(thresholdedDepth, contours, hierarchy, CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// TODO continue with this:
	// http://docs.opencv.org/2.4.10/doc/tutorials/imgproc/shapedescriptors/bounding_rotated_ellipses/bounding_rotated_ellipses.html

	// Sample code brightening up the depth image to make the values visible
	m_depthImage *= 10;
}

void Application::loop() {
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
	// If you want to control the motor / LED
	// m_kinectMotor = new KinectMotor;

	m_depthCamera = new DepthCamera;

	// open windows
	cv::namedWindow("output", 1);
	cv::namedWindow("depth", 1);
	cv::namedWindow("bgr", 1);

    // create work buffer
	m_bgrImage = cv::Mat(480, 640, CV_8UC3);
	m_depthImage = cv::Mat(480, 640, CV_16UC1);
	m_outputImage = cv::Mat(480, 640, CV_8UC1);
}

Application::~Application() {
	if (m_depthCamera) delete m_depthCamera;
	if (m_kinectMotor) delete m_kinectMotor;
}

bool Application::isFinished() {
	return m_isFinished;
}
