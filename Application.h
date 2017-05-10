#pragma once

#include <opencv2/core/core.hpp>

class DepthCamera;
class KinectMotor;

class Application
{
public:
	Application();
	virtual ~Application();

	void loop();

	void processFrame();

	void makeScreenshots();
	void calibrate();
	void clearOutputImage();

	bool isFinished();

protected:
	DepthCamera *m_depthCamera;
	KinectMotor *m_kinectMotor;

	cv::Mat m_bgrImage;
	cv::Mat m_depthImage;
	cv::Mat m_outputImage;
	cv::Mat m_calibrationImage;

	std::vector<cv::Point2f> m_footPathPoints; // log path that the ellipse center follows

	bool m_isFinished;
	bool m_isCalibrated;

	bool m_footWasDownLastIteration; // for debouncing
	bool m_footLiftedLastIteration; // for debouncing
	unsigned int m_frameCounter; // for time quantization / discretization

	double m_groundValue;
};
