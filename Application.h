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

	bool m_isFinished;
	bool m_isCalibrated;
};
