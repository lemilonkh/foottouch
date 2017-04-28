#include "DepthCameraException.h"

DepthCameraException::DepthCameraException(std::string message)
	: m_message(message)
{}

DepthCameraException::DepthCameraException(std::string description, XnStatus status)
	: m_message(description + ": " + xnGetStatusString(status))
{}

DepthCameraException::~DepthCameraException()
{}

const char* DepthCameraException::what() const
{
	return m_message.c_str();
}
