#pragma once

// Control the Kinect motor and LEDs
// Author: Stefan Neubert (2015)
// Based on Kinect Motor Control by Heresy @ http://kheresy.wordpress.com/
// modify from: https://github.com/manctl/openni/blob/manctl/Samples/Kinect/kinect-motors.cpp
// reference: http://openkinect.org/wiki/Protocol_Documentation#Control_Packet_Structure

struct XnVector3D;
struct XnUSBDeviceHandle;

class KinectMotor
{
public:
	KinectMotor();
	virtual ~KinectMotor();

	enum LED_STATUS
	{
		LED_OFF					= 0,
		LED_GREEN				= 1,
		LED_RED					= 2,
		LED_YELLOW				= 3,
		LED_BLINK_YELLOW		= 4, // does not seem to work
		LED_BLINK_GREEN			= 5,
		LED_BLINK_RED_YELLOW	= 6
	};

	enum MOTOR_STATUS
	{
		MOTOR_STOPPED	= 0x00,
		MOTOR_LIMIT		= 0x01,
		MOTOR_MOVING	= 0x04,
		MOTOR_UNKNOWN	= 0x08
	};

	void tiltTo(int angle);
	void setLED(LED_STATUS status);
	void getInformation(int& rAngle, MOTOR_STATUS& rMotorStatus, XnVector3D& rVec) const;

	int GetAngle() const;
	MOTOR_STATUS GetMotorStatus() const;
	XnVector3D GetAccelerometer() const;

protected:
	XnUSBDeviceHandle* m_motor;
};

