////////////////////////////////////////////////////////////////////////////////
//
// Reads the pendigits file
//
// Authors: Christoph Sterz, Patrick Lühne (2013)
//
////////////////////////////////////////////////////////////////////////////////

#include "DataSet.h"

#include <iostream>
#include <fstream>
#include <limits>

#include <boost/tokenizer.hpp>

////////////////////////////////////////////////////////////////////////////////

float stringToFloat(std::string string)
{
	float number;
	std::stringstream stringstream(string);
	stringstream >> number;
	return number;
}

////////////////////////////////////////////////////////////////////////////////

void readDataSet(std::string fileName, int numberOfRows, cv::Mat &data, cv::Mat &labels)
{
	data = cv::Mat(numberOfRows, 16, CV_32FC1);
	labels = cv::Mat(numberOfRows, 1, CV_32FC1);

	std::ifstream file(fileName.c_str());

	if (!file.is_open())
	{
		std::cerr << "ERROR: No such file." << std::endl;
		exit(EXIT_FAILURE);
	}

	// Setup boost's CSV parser
	typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;

	std::vector<std::string> tokens;
	std::string currentLine;

	int row = 0;

	// Parse file
	while (getline(file, currentLine) && row < numberOfRows)
	{
		// Skip empty lines
		if (currentLine == "")
			continue;

		boost::char_separator<char> separators(",", "");
		Tokenizer tokenizer(currentLine, separators);
		tokens.assign(tokenizer.begin(), tokenizer.end());

		for (int i = 0; i < 16; i++)
			data.at<float>(row, i) = stringToFloat(tokens[i]) / 100.0f;

		labels.at<float>(row) = stringToFloat(tokens[16]);
		row++;
	}
}

