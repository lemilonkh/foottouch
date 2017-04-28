#include "KinectMotor.h"

#include <iostream>

#include "DepthCameraException.h"

#include <XnUSB.h>
#include <XnTypes.h>

#define VID_MICROSOFT 0x045E
#define PID_NUI_MOTOR 0x02B0

KinectMotor::KinectMotor()
{
	XnStatus status = XN_STATUS_OK;
	
	std::cout << "Initialize USB...";
	status = xnUSBInit();
	if(status != XN_STATUS_OK && status != XN_STATUS_USB_ALREADY_INIT)
		throw DepthCameraException("USB could not be initialized", status);
	std::cout << " done." << std::endl;

	std::cout << "Open connection to kinect motor...";
	const XnUSBConnectionString *paths;
	XnUInt32 count;
	status = xnUSBEnumerateDevices(VID_MICROSOFT, PID_NUI_MOTOR, &paths, &count);
	if(status != XN_STATUS_OK) throw DepthCameraException("No kinect motor found", status);
	if(count < 1) throw DepthCameraException("No kinect motor found", XN_STATUS_USB_DEVICE_NOT_FOUND);

	status = xnUSBOpenDeviceByPath(paths[0], &m_motor);
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not open connection to motor", status);

	XnUChar buf;
	status = xnUSBSendControl(m_motor, (XnUSBControlType) 0xc0, 0x10, 0x00, 0x00, &buf, sizeof(buf), 0);
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not initialize kinect motor", status);

	status = xnUSBSendControl(m_motor, XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 0, 0);
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not initialize kinect motor", status);
	std::cout << " done." << std::endl;
}

void KinectMotor::tiltTo(int angle)
{
	XnStatus status = xnUSBSendControl(m_motor, XN_USB_CONTROL_TYPE_VENDOR, 0x31, 2 * angle, 0x00, NULL, 0, 0);
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not move motor", status);
}

void KinectMotor::setLED(LED_STATUS led)
{
	XnStatus status = xnUSBSendControl(m_motor, XN_USB_CONTROL_TYPE_VENDOR, 0x06, led, 0x00, NULL, 0, 0);
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not set LED status", status);
}

int KinectMotor::GetAngle() const
{
	int angle;
	MOTOR_STATUS status;
	XnVector3D acceleration;
	getInformation(angle, status, acceleration);
	return angle;
}
	
KinectMotor::MOTOR_STATUS KinectMotor::GetMotorStatus() const
{
	int angle;
	MOTOR_STATUS status;
	XnVector3D acceleration;
	getInformation(angle, status, acceleration);
	return status;
}

XnVector3D KinectMotor::GetAccelerometer() const
{
	int angle;
	MOTOR_STATUS status;
	XnVector3D acceleration;
	getInformation(angle, status, acceleration);
	return acceleration;
}

void KinectMotor::getInformation(int& rAngle, MOTOR_STATUS& rMotorStatus, XnVector3D& rVec) const
{
	XnUChar aData[10];
	XnUInt32 uSize;
	XnStatus status = xnUSBReceiveControl(m_motor, XN_USB_CONTROL_TYPE_VENDOR, 0x32, 0x00, 0x00, aData, 10, &uSize, 0);
	if(status != XN_STATUS_OK) throw DepthCameraException("Could not retrieve motor information", status);

	std::cout << (int)aData[0] << std::endl;
	std::cout << (int)aData[1] << std::endl;

	rAngle = aData[8];
	if(rAngle > 128)
		rAngle = (int) (-0.5 * (255 - rAngle));
	else
		rAngle /= 2;

	if(aData[9] == MOTOR_STOPPED)
		rMotorStatus = MOTOR_STOPPED;
	else if(aData[9] == MOTOR_LIMIT)
		rMotorStatus = MOTOR_LIMIT;
	else if(aData[9] == MOTOR_MOVING)
		rMotorStatus = MOTOR_MOVING;
	else
		rMotorStatus = MOTOR_UNKNOWN;

	rVec.X = (float)(((XnUInt16)aData[2] << 8) | aData[3]);
	rVec.Y = (float)(((XnUInt16)aData[4] << 8) | aData[5]);
	rVec.Z = (float)(((XnUInt16)aData[6] << 8) | aData[7]);
}

KinectMotor::~KinectMotor()
{
	XnStatus status = xnUSBCloseDevice(m_motor);
	if(status != XN_STATUS_OK && status != XN_STATUS_USB_DEVICE_NOT_VALID && status != XN_STATUS_DEVICE_NOT_CONNECTED)
		throw DepthCameraException("Could not shutdown kinect motor", status);
}
