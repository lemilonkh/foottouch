#include "DataSet.h"
#include "knn.h"

#include <opencv2/core/core.hpp>
#include <iostream>
using namespace std;

// k = 1 to 5 are usually good
// accuracy of > 0.95 was achieved with k = 5 and nOR = 1000
int classify(int k, float* input) {
	cv::Mat data;
	cv::Mat labels;
	string fileName = "pendigits.tra";

	// TrainingsSetSize if to slow, lower this number for worse accuracy
	int numberOfRows = 1000;

	readDataSet(fileName, numberOfRows, data, labels);
	vector<pair<float, int>> distances;
	for (int r = 0; r < numberOfRows; r++) {
		float distance = 0;
		for (int i = 0; i < 16; i++) {
			distance += pow(input[i]-data.at<float>(r, i), 2);
		}
		distances.push_back(make_pair(sqrt(distance), labels.at<float>(r)));
	}
	sort(distances.begin(), distances.end());
	vector<int> ranking(10);
	for (int i = 0; i < k; i++) {
		ranking[distances[i].second] += 1;
	}
	int max = 0;
	for (int i = 0; i < 10; i++) {
		if (ranking[max] < ranking[i]) {
			max = i;
		}
	}

	return max;
}
