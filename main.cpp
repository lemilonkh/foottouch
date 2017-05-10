#include "Application.h"
//#include "knn.h"
//#include "DataSet.h"

#include <iostream>

#include "framework/DepthCameraException.h"

using namespace std;
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

	// KNN TESTING!!!
	/*cv::Mat data;
	cv::Mat labels;
	string fileName = "pendigits.tra";
	int numberOfRows = 7495;
	float right = 0;
	float wrong = 0;
	readDataSet(fileName, numberOfRows, data, labels);
	for (int i = 4497; i < numberOfRows; i++) {
		cout << "main index: " << i << endl;
		int result = classify(5, &data.at<float>(i*16));
		cout << result << " == " << labels.at<float>(i) << endl;
		if (result == labels.at<float>(i)) {
			right += 1;
		} else {
			wrong += 1;
		}
		cout << "Accuracy: " << right/(right+wrong) << endl;
	}

	return 0;
	*/
}