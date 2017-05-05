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

const int IMAGE_AMPLIFICATION = 10; // multiplied into the depth texture
const int IMAGE_HEIGHT = 480;
const int IMAGE_WIDTH = 640;
const int CROSSHAIR_SIZE = 50;
const int MIN_CONTOUR_SIZE = 20; // TODO remove
const double GROUND_THRESHOLD = 34; // TODO figure out automatically
const double LEG_THRESHOLD = 28; // TODO figure out automatically

void Application::processFrame() {
	// Used textures:
	// * m_bgrImage: The image of the Kinect's color camera
	// * m_depthImage: The image of the Kinects's depth sensor
	// * m_outputImage: The image in which you can draw the touch circles

	// thresholding vars
	Mat withoutGround, thresholdedDepth, src;

	double maxValue = 255;

	// Amplify and convert image from 16bit to 8bit
	m_depthImage *= IMAGE_AMPLIFICATION;
	m_depthImage.convertTo(src, CV_8UC1, 1.0/256.0, 0);

	// first thresholding pass (remove ground)
	threshold(src, withoutGround, GROUND_THRESHOLD, maxValue, THRESH_TOZERO_INV);

	// second thresholding pass (remove leg etc.)
	threshold(withoutGround, thresholdedDepth, LEG_THRESHOLD, maxValue, THRESH_TRUNC);

	// find outlines
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresholdedDepth, contours, hierarchy, CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// add real color image to output
	m_outputImage = m_bgrImage;

	// fit ellipses & determine center points
	vector<RotatedRect> minEllipses(contours.size());
	vector<Point2f> centerPoints(contours.size());
	RotatedRect currentEllipse;
	Point2f currentCenter;
	float currentSize;
	Scalar drawColor;

	for(int i = 0; i < contours.size(); i++) {
		currentEllipse = fitEllipse(Mat(contours[i]));
		currentCenter = currentEllipse.center;
		currentSize = currentEllipse.size.width * currentEllipse.size.height;

		// filter out too small or too large ellipses
		if(currentSize > MIN_CONTOUR_SIZE) {
			minEllipses[i] = currentEllipse;

			centerPoints.push_back(currentCenter);
			cout << "Center: " << currentCenter.x << "," << currentCenter.y << "\n";

			drawColor = Scalar(0, 255, 50);
		} else {
			drawColor = Scalar(255, 0, 0);
		}

		// draw contours and ellipses
		drawContours(m_outputImage, contours, i, drawColor, 1, 8, vector<Vec4i>(), 0, Point());
		ellipse(m_outputImage, minEllipses[i], drawColor, 2, 8);

		// draw center points (using a crosshair => two lines)
		line(m_outputImage,
			centerPoints[i] - Point2f(CROSSHAIR_SIZE, 0),
			centerPoints[i] + Point2f(CROSSHAIR_SIZE, 0),
			color, 8, 8);
		line(m_outputImage,
			centerPoints[i] - Point2f(0, CROSSHAIR_SIZE),
			centerPoints[i] + Point2f(0, CROSSHAIR_SIZE),
			color, 8, 8);
		}
	}
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
		m_outputImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
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
