#include <string>
#include <exception>

#include <XnStatus.h>

class DepthCameraException : public std::exception
{
public:
	DepthCameraException(std::string message);
	DepthCameraException(std::string description, XnStatus status);
	virtual ~DepthCameraException();

	virtual const char* what() const;

protected:
	std::string m_message;
};