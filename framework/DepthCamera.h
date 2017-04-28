#pragma once

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <XnCppWrapper.h>

class DepthCamera
{
public:
	DepthCamera();
	virtual ~DepthCamera();

	static const XnMapOutputMode DepthCamera::OUTPUT_MODE;

	void getFrame(cv::Mat &bgrImage, cv::Mat &depthImage);

protected:
	xn::Context m_context;
	xn::ScriptNode m_scriptNode;

	xn::DepthGenerator m_depthGenerator;
	xn::ImageGenerator m_imageGenerator;
	xn::DepthMetaData m_depthMetaData;
	xn::ImageMetaData m_imageMetaData;

	cv::VideoCapture m_bgrReader;
	cv::VideoCapture m_depthReader;
};