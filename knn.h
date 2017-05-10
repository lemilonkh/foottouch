#ifndef __KNN_H
#define __KNN_H

#include <string>

#include <opencv2/core/core.hpp>

float stringToFloat(std::string string);

int classify(int k, float* input);

#endif