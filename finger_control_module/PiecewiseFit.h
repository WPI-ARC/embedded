class PiecewiseFit {
	int length;
	float* thresholds;
	float* slopes;
	float* offsets;
public:
	PiecewiseFit(int length, float* thresholds, float* slopes, float* offsets);
	~PiecewiseFit();
	void updateFit(int length, float* thresholds, float* slopes, float* offsets);
	float getEstimate(float xval);
};