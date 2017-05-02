#pragma once

#include <opencv2/core/core.hpp>

class DepthCamera;
class KinectMotor;

#define DEFAULT_MOTOR_ANGLE 180 // TODO figure out the correct angle to the tripod

class Application
{
public:
	Application();
	virtual ~Application();

	void loop();

	void processFrame();

	void makeScreenshots();
	void clearOutputImage();

	bool isFinished();

protected:
	DepthCamera *m_depthCamera;
	KinectMotor *m_kinectMotor;

	cv::Mat m_bgrImage;
	cv::Mat m_depthImage;
	cv::Mat m_outputImage;

	bool m_isFinished;
};
