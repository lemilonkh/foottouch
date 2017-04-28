#include "Application.h"

#include <iostream>

#include "framework/DepthCameraException.h"

int main()
{
	try
	{
		Application application;
		
		while (!application.isFinished())
			application.loop();
	}
	catch (DepthCameraException dce)
	{
		std::cerr << std::endl << "[DepthCamera Error] " << dce.what() << std::endl;
		std::cout << std::endl << std::endl << "Press Enter to close the application...";
		std::cin.ignore();
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}