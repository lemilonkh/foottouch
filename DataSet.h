////////////////////////////////////////////////////////////////////////////////
//
// Reads the pendigits file
//
// Authors: Christoph Sterz, Patrick Lühne (2013)
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __DATASET_H
#define __DATASET_H

#include <string>

#include <opencv2/core/core.hpp>

float stringToFloat(std::string string);

// Reads the pendigits file (the first n rows). Returns a matrix containing the
// data (n x 16) and a matrix with the labels (n x 1).
void readDataSet(std::string fileName, int numberOfRows, cv::Mat &data, cv::Mat &labels);

#endif

