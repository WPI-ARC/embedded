#include <stdlib.h>
#include <string.h>
#include "PiecewiseFit.h"

PiecewiseFit::PiecewiseFit(int length, float* thresholds, float* slopes, float* offsets) {
	this->length = length;
	this->thresholds = (float*)malloc(length*sizeof(float));
	this->slopes = (float*)malloc(length*sizeof(float));
	this->offsets = (float*)malloc(length*sizeof(float));

	memcpy(this->thresholds, thresholds, length*sizeof(float));
	memcpy(this->slopes, slopes, length*sizeof(float));
	memcpy(this->offsets, offsets, length*sizeof(float));
}

PiecewiseFit::~PiecewiseFit() {
	free(this->thresholds);
	free(this->slopes);
	free(this->offsets);
}

void PiecewiseFit::updateFit(int length, float* thresholds, float* slopes, float* offsets) {
	free(this->thresholds);
	free(this->slopes);
	free(this->offsets);

	this->length = length;
	this->thresholds = (float*)malloc(length*sizeof(float));
	this->slopes = (float*)malloc(length*sizeof(float));
	this->offsets = (float*)malloc(length*sizeof(float));

	memcpy(this->thresholds, thresholds, length*sizeof(float));
	memcpy(this->slopes, slopes, length*sizeof(float));
	memcpy(this->offsets, offsets, length*sizeof(float));
}

float PiecewiseFit::getEstimate(float xval) {
	float estimate = 0;
	for(int i = 0; i < length; ++i) {
        if(xval > thresholds[i]) {
            estimate = (slopes[i] * xval) + offsets[i];
            break;
        }
    }
    return estimate;
}