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
#include <limits>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "framework/DepthCamera.h"
#include "framework/KinectMotor.h"

#include "knn.h"

using namespace std;
using namespace cv;

// fix for max macro
#ifdef max
#undef max
#endif

#define FOREACH_PATH for(auto i = 0u; i < m_footPathPoints.size(); i++)

#define FOREACH_PATH_WITHOUT_LAST for(auto i = 0u; i < m_footPathPoints.size() - 1; i++)

const int IMAGE_AMPLIFICATION = 20; // multiplied into the depth texture
const int IMAGE_HEIGHT = 480;
const int IMAGE_WIDTH = 640;
const int CROSSHAIR_SIZE = 50;
const int MIN_CONTOUR_POINTS = 10;
const int OVER_SIX_THOUSAND = 6001; //MIN_ELLIPSE_SIZE
const int MIN_CONTOUR_SIZE = 100;
const int MAX_CONTOUR_SIZE = 200;
const double LEG_THRESHOLD = 52; // TODO figure out automatically
const int FRAME_SAMPLING_INTERVAL = 8; // every N frames
const int CLASSIFICATION_POINT_COUNT = 8;
const int K_NEIGHBORHOOD_SIZE = 5; // classification used neighbor count (majority vote)

void Application::processFrame() {
	// Used textures:
	// * m_bgrImage: The image of the Kinect's color camera
	// * m_depthImage: The image of the Kinects's depth sensor
	// * m_outputImage: The image in which you can draw the touch circles

	if(!m_isCalibrated)
		calibrate();

	// thresholding vars
	Mat withoutGround, thresholdedDepth, src, diff;

	double maxValue = 255;

	// Amplify and convert image from 16bit to 8bit
	m_depthImage *= IMAGE_AMPLIFICATION;
	m_depthImage.convertTo(src, CV_8UC1, 1.0/256.0, 0);

	// removes calibration image from depth image
	// so only parts that moved since then are still visible
	absdiff(src, m_calibrationImage, diff);

	// blur to remove artifacts
	medianBlur(diff, diff, 25);

	// amplify to generate a higher contrast image
	diff *= 10;

	// thresholding pass (remove leg etc.)
	threshold(diff, withoutGround, LEG_THRESHOLD, maxValue, THRESH_TOZERO_INV);
	threshold(withoutGround, thresholdedDepth, 20, maxValue, THRESH_TOZERO);

	// find outlines
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresholdedDepth, contours, hierarchy, CV_RETR_TREE,
		CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// add real color image to output
	m_outputImage = m_bgrImage; //thresholdedDepth

	// fit ellipses & determine center points
	vector<RotatedRect> minEllipses(contours.size());
	vector<Point2f> centerPoints(contours.size());
	RotatedRect currentEllipse;
	Point2f currentCenter;
	float currentSize;
	Scalar drawColor;

	// TODO remove debug output
	//cout << "Found " << contours.size() << " contours!" << endl;

	// is there any foot found in this frame?
	bool anyEllipseValid = false;
	double maxEllipseSize = 0.0;
	Point2f maxEllipseCenter(-1.0, -1.0);

	for(auto i = 0u; i < contours.size(); i++) {
		// don't use too small shapes (point count)
		if(contours[i].size() < MIN_CONTOUR_POINTS)
			continue;

		currentEllipse = fitEllipse(Mat(contours[i]));
		currentCenter = currentEllipse.center;
		currentSize = currentEllipse.size.width * currentEllipse.size.height;

		// filter out too small ellipses
		// find ellipse with the maximum size
		anyEllipseValid = true;
		m_footWasDownLastIteration = true;

		// sample every N frames
		if(currentSize > maxEllipseSize) {
			maxEllipseCenter = currentCenter;
			maxEllipseSize = currentSize;
			drawColor = Scalar(0, 255, 255);
		}

		minEllipses[i] = currentEllipse;

		centerPoints.push_back(currentCenter);

		// TODO remove debug output
		//cout << "Center: " << currentCenter.x << "," << currentCenter.y << "\n";

		drawColor = Scalar(255, 255, 255);

		ellipse(thresholdedDepth, currentEllipse, drawColor, 2, 8);

		// draw contours and ellipses
		drawContours(m_outputImage, contours, i, drawColor, 1, 8, vector<Vec4i>(), 0, Point());
		ellipse(m_outputImage, minEllipses[i], drawColor, 2, 8);

		// draw center points (using a crosshair marker)
		// TODO only compatible with OpenCV3
		/*cv::drawMarker(m_outputImage,
			centerPoints[i],
			drawColor,
			cv::MARKER_CROSS,
			20,													// marker size
			1,													// thickness
			LineTypes::LINE_8 // was: 8 // line type
		);*/
	}

	// only write to path log if:
	// * big enough
	// * enough time has passed
	// * valid center point was found (not -1 in coords)
	if(maxEllipseSize > OVER_SIX_THOUSAND && m_frameCounter >= FRAME_SAMPLING_INTERVAL &&
	  maxEllipseCenter.x >= 0.0 && maxEllipseCenter.y >= 0.0) {
		m_frameCounter = 0;
		Point2f lastPoint = m_footPathPoints.back();
	  m_footPathPoints.push_back(maxEllipseCenter);
		cout << "Adding point " << maxEllipseCenter << " to path! #ULTRA" << endl;

		// draw line from last to current point
		Scalar lineColor(128, 64, 255);
		if(m_footPathPoints.size() > 1) {
			arrowedLine(
				m_drawnLines,
				lastPoint,
				maxEllipseCenter,
				lineColor,
				5, // line thickness
				CV_AA, // antialiasing, alternatives: 4 or 8,
				0, // shift (number of fractional parts)
				0.25 // arrow tip length in relation to arrow length
			);
		}

		// if there are enough points, start classifying!
		if(m_footPathPoints.size() >= CLASSIFICATION_POINT_COUNT) {
			classifyFootPathAndReset();
		}
	}

	// draw line image over output
	m_outputImage += m_drawnLines;

	if(!anyEllipseValid && m_footWasDownLastIteration) {
		m_footLiftedLastIteration = true;
		// TODO classify(k, [16 floats (x, y alternating)]);
	}

	cout << "Frame #"  << m_frameCounter   << "\t\t| "
			 << "Center: " << maxEllipseCenter << "\t\t| "
			 << "Size: "   << maxEllipseSize   << endl;

	m_frameCounter++;
}

void Application::classifyFootPathAndReset() {
	assert(m_footPathPoints.size() == CLASSIFICATION_POINT_COUNT);

	normalizePath();
	flipCoordinatesForPath();

	// transform vector to array
	float pathArray[CLASSIFICATION_POINT_COUNT*2];
	FOREACH_PATH {
		pathArray[2*i] = m_footPathPoints[i].x;
		pathArray[2*i+1] = m_footPathPoints[i].y;
	}

	cout << "###############" << endl
	     << "# CLASSIFYING #" << endl
			 << "###############" << endl;
	cout << "Path vector: " << m_footPathPoints << endl;
	cout << "Path array: " << pathArray << endl;

	// run classifier! SCIENCE!
	int result = classify(K_NEIGHBORHOOD_SIZE, pathArray);

	cout << "OUTPUT: " << result << " --- BOMBAKLAAD!1!" << endl;

	// clear foot path vector
	m_footPathPoints.clear();
}

// euclidean distance based discretization of the foot path
void Application::reducePath() {
	float pathLength = 0.0;
	Point2f current, next;
	FOREACH_PATH_WITHOUT_LAST {
		current = m_footPathPoints[i];
		next = m_footPathPoints[i+1]; // can be done because the last element isn't iterated over
		cout << "path reducing in progress" << endl;
		pathLength += norm(next - current); // calculates euclidean distance between points
	}

	assert(pathLength > 0.0);

	float pointDist = pathLength / CLASSIFICATION_POINT_COUNT;
	float travelledDistance = 0.0;
	vector<Point2f> reducedPath(CLASSIFICATION_POINT_COUNT);

	FOREACH_PATH_WITHOUT_LAST {
		current = m_footPathPoints[i];
		next = m_footPathPoints[i+1]; // can be done because the last element isn't iterated over
		cout << "path reducing in progress" << endl;
		travelledDistance += norm(next - current); // calculates euclidean distance between points

		if(travelledDistance >= pointDist) {
			travelledDistance -= pointDist;
			reducedPath.push_back(current);
		}
	}

	// add last point if it isn't already in the path
	if(reducedPath.size() == 7) {
		cout << "ReducedPath is one point short!" << endl;
		cout << "Adding last point " << m_footPathPoints.back() << " to make up for that :8)" << endl;
		reducedPath.push_back(m_footPathPoints.back());
	}
}

void Application::normalizePath() {
	float inf = numeric_limits<float>::max();
	Point2f pMin(inf, inf);
	Point2f pMax(0.0, 0.0);
	Point2f current;

	vector<Point2f> transformedPath;

	FOREACH_PATH {
		current = m_footPathPoints[i];

		if(current.x < pMin.x)
			pMin.x = current.x;
		if(current.y < pMin.y)
			pMin.y = current.y;

		if(current.x > pMax.x)
			pMax.x = current.x;
		if(current.y > pMax.y)
			pMax.y = current.y;
	}

	cout << "Min Point: " << pMin << endl
	     << "Max Point: " << pMax << endl;

	// normalize whole path into [0,1]^2
	FOREACH_PATH {
		current = m_footPathPoints[i];
		current -= pMin;
		Point2f boundingBoxSize = pMax - pMin;
		cout << "Bounding box size: " << boundingBoxSize << endl;
		current.x /= boundingBoxSize.x;
		current.y /= boundingBoxSize.y;

		transformedPath.push_back(current);
	}

	cout << "Transformed path: " << transformedPath << endl;
}

void Application::flipCoordinatesForPath() {
	FOREACH_PATH {
		m_footPathPoints[i].y = 1.0 - m_footPathPoints[i].y;
	}
}

void Application::loop() {
	try {
		m_depthCamera->getFrame(m_bgrImage, m_depthImage);

		int key = cv::waitKey(20);
		switch (key) {
		case 'q': // quit
			m_isFinished = true;
			break;
		case 's': // screenshot
			makeScreenshots();
			break;
		case ' ': // calibrate
			calibrate();
			cout << "Calibrating..." << endl;
			cout << "Ground value: " << m_groundValue << endl;
		}

		processFrame();

		cv::imshow("calibration", m_calibrationImage);
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

void Application::calibrate() {
	// Amplify and convert image from 16bit to 8bit
	m_depthImage *= IMAGE_AMPLIFICATION;
	m_depthImage.convertTo(m_calibrationImage, CV_8UC1, 1.0/256.0, 0);

	m_isCalibrated = true;

	double min, max;
	minMaxLoc(m_calibrationImage, &min, &max);
	m_groundValue = max;
}

void Application::clearDrawnLines() {
	m_drawnLines = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
}

Application::Application() :
	m_isFinished(false),
	m_depthCamera(nullptr),
	m_kinectMotor(nullptr) {

	m_isCalibrated = false;
	m_footLiftedLastIteration = false;
	m_groundValue = 1.0;
	m_frameCounter = 0;

	// connect to Kinect
	try {
		m_depthCamera = new DepthCamera;

		// open windows
		cv::namedWindow("output", 1);
		cv::namedWindow("depth", 1);
		cv::namedWindow("calibration", 1);

	  // create work buffers
		m_bgrImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
		m_depthImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);
		m_outputImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
		m_calibrationImage = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

		clearDrawnLines();
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
