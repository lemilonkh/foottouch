///////////////////////////////////////////////////////////////////////////
//
// Retreive OpenCV image data from kinect
// Authors:
//		Stefan Neubert (2015)
//		Stephan Richter (2011)
//		Patrick Lühne (2012)
//
///////////////////////////////////////////////////////////////////////////

#include "DepthCamera.h"

#include <iostream>
#include <cstdint>

#include "DepthCameraException.h"

#include <opencv2/imgproc/imgproc.hpp>

const XnMapOutputMode DepthCamera::OUTPUT_MODE = {640, 480, 30};

DepthCamera::DepthCamera()
{
	xn::EnumerationErrors errors;
	XnStatus status = XN_STATUS_OK;

	std::cout << "Initialize OpenNI context...";
	status = m_context.Init();
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not initialize context", status);
	std::cout << " done." << std::endl;

	xn::Query query;
	query.AddSupportedMapOutputMode(OUTPUT_MODE);
	query.AddSupportedCapability(XN_CAPABILITY_MIRROR);
	
	std::cout << "Create image generator...";
	status = m_imageGenerator.Create(m_context, &query);
	if (status != XN_STATUS_OK) throw DepthCameraException("No image generator found", status);
	status = m_imageGenerator.SetMapOutputMode(OUTPUT_MODE);
	status = m_imageGenerator.GetMirrorCap().SetMirror(true);
	std::cout << " done." << std::endl;

	
	query.AddSupportedCapability(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT);
	std::cout << "Create depth generator...";
	status = m_depthGenerator.Create(m_context, &query);
	if (status != XN_STATUS_OK) throw DepthCameraException("No depth generator found", status);
	status = m_depthGenerator.SetMapOutputMode(OUTPUT_MODE);
	status = m_depthGenerator.GetMirrorCap().SetMirror(true);
	m_depthGenerator.GetAlternativeViewPointCap().SetViewPoint(m_imageGenerator);
	std::cout << " done." << std::endl;

	std::cout << "Start generators...";
	status = m_context.StartGeneratingAll();

	if (status != XN_STATUS_OK) throw DepthCameraException("Starting generators failed", status);
	std::cout << " done." << std::endl;
}

DepthCamera::~DepthCamera()
{}

void DepthCamera::getFrame(cv::Mat &bgrImage, cv::Mat &depthImage)
{
	XnStatus status = m_context.WaitAndUpdateAll();
	
	if (status != XN_STATUS_OK) throw DepthCameraException("Could not acquire frame", status);
	
	m_depthGenerator.GetMetaData(m_depthMetaData);
	m_imageGenerator.GetMetaData(m_imageMetaData);

	// depth image
	XnUInt16 depthWidth = m_depthMetaData.XRes();
	XnUInt16 depthHeight = m_depthMetaData.YRes();

	const XnDepthPixel* depth = m_depthMetaData.Data();

	for(XnUInt16 y = 0; y < depthHeight; ++y)
	{
		for(XnUInt16 x = 0; x < depthWidth; ++x, ++depth)
		{
			depthImage.at<uint16_t>(y, x) = *depth;
		}
	}

	// bgr image
	XnUInt16 bgrWidth = m_imageMetaData.XRes();
	XnUInt16 bgrHeight = m_imageMetaData.YRes();

	const XnRGB24Pixel *rgb = m_imageMetaData.RGB24Data();

	for(XnUInt16 y = 0; y < bgrHeight; ++y)
	{
		for(XnUInt16 x = 0; x < bgrWidth; ++x, ++rgb)
		{
			cv::Vec3b& p = bgrImage.at<cv::Vec3b>(y, x);
			p[0] = rgb->nBlue;
			p[1] = rgb->nGreen;
			p[2] = rgb->nRed;
		}
	}
}