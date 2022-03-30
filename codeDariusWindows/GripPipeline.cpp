#include "GripPipeline.h"

namespace grip {
// Sets defaults for HSV
GripPipeline::GripPipeline() {
	hsvThresholdHue[1] = 112.0;
	hsvThresholdHue[0] = 40.0;
	hsvThresholdSaturation[1] = 233.0;
	hsvThresholdSaturation[0] = 60.0;
	hsvThresholdValue[1] = 255.0;
	hsvThresholdValue[0] =  216.0;
	cameraConstY = 24.0;
	cameraConstX = 320.0/14.0;
}
// Standardizes the angles for scoring purposes (not needed for this algorithm, but kept for future use
void CorrectRect(cv::RotatedRect &rect) {
	if (rect.size.width > rect.size.height)
		return;
	double tempHeight = rect.size.height;
	rect.size.height = rect.size.width;
	rect.size.width = tempHeight;

	rect.angle = 90 + rect.angle;
}
// Updates interal values from NT table (runs every second)
void FetchVisionNetworkTable() {
	filterHeight = filterEntries[0].GetDouble(filterHeight);
	deviationThresh = filterEntries[1].GetDouble(deviationThresh);

	hsvThresholdHue[0] = hsvThresholdEntries[0].GetDouble(hsvThresholdHue[0]); 
	hsvThresholdHue[1] = hsvThresholdEntries[1].GetDouble(hsvThresholdHue[1]);
	hsvThresholdSaturation[0] = hsvThresholdEntries[2].GetDouble(hsvThresholdSaturation[0]);
	hsvThresholdSaturation[1] = hsvThresholdEntries[3].GetDouble(hsvThresholdSaturation[1]);
	hsvThresholdValue[0] = hsvThresholdEntries[4].GetDouble(hsvThresholdValue[0]);
	hsvThresholdValue[1] = hsvThresholdEntries[5].GetDouble(hsvThresholdValue[1]);

}
// Outputs values from algorithm (runs every tick)
void UpdateVisionNetworkTable(double avgX, double avgY, double avgWidth, double avgHeight, double stripeCount, double heightDiff, double xOffsetPx) {
	outputEntries[0].SetDouble(avgX);
	outputEntries[1].SetDouble(avgY);
	outputEntries[2].SetDouble(avgWidth);
	outputEntries[3].SetDouble(avgHeight);
	outputEntries[4].SetDouble(123.0); // TODO calc dist
	outputEntries[5].SetDouble(stripeCount);
	outputEntries[6].SetDouble(heightDiff);
	outputEntries[7].SetDouble(xOffsetPx);
	outputEntries[8].SetDouble(watchdog++);
	nt->Flush();
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
bool SortRect(cv::RotatedRect &a, cv::RotatedRect &b) {
	return a.center.y < b.center.y;
}
void GripPipeline::Process(cv::Mat& source0){
	FetchVisionNetworkTable();
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = source0;
	hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, this->hsvThresholdOutput);
	//Step Blur0:
	//input
	cv::Mat blurInput = hsvThresholdOutput;
	BlurType blurType = BlurType::BOX;
	double blurRadius = 3.6036036036036063;  // default Double
	blur(blurInput, blurType, blurRadius, this->blurOutput);
	//Step CV_Threshold0:
	//input
	cv::Mat cvErodeSrc = blurOutput;
	cv::Mat cvErodeOutput;
	cv::Mat cvErodeKernel;
	cv::Point cvErodeAnchor(-1, -1);
	double cvErodeIterations = 1;
	int cvErodeBordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErodeBordervalue(-1);
	cv::erode(cvErodeSrc, cvErodeOutput, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue);	
	cv::Mat cvThresholdSrc = cvErodeOutput;
	double cvThresholdThresh = 30;
	double cvThresholdMaxval = 255.0;  // default Double
    	int cvThresholdType = cv::THRESH_OTSU;
	cvThreshold(cvThresholdSrc, cvThresholdThresh, cvThresholdMaxval, cvThresholdType, this->cvThresholdOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = cvThresholdOutput;
	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	//Step Convex_Hulls0:
	//input
	std::vector<std::vector<cv::Point> > convexHullsContours = findContoursOutput;
	convexHulls(convexHullsContours, this->convexHullsOutput);




	std::string suffix = std::to_string(watchdog) + ".png";
	

	std::vector<cv::RotatedRect> rotatedRectangles;
	
	for (unsigned int i = 0 ; i < convexHullsContours.size() ; i++) {
		rotatedRectangles.push_back(cv::minAreaRect(convexHullsContours[i]));
		CorrectRect(rotatedRectangles[i]);
	}
	sort(rotatedRectangles.begin(), rotatedRectangles.end(), SortRect);
	/*if (watchdog % 100*20 == 0) {
		// Write debug image files
		cv::imwrite("/home/pi/DebugImages/src_" + suffix, source0);
		cv::imwrite("/home/pi/DebugImages/hsv_" + suffix, hsvThresholdOutput);
		cv::imwrite("/home/pi/DebugImages/blur_" + suffix, blurOutput);
		cv::imwrite("/home/pi/DebugImages/erode_" + suffix, cvErodeOutput);
		cv::imwrite("/home/pi/DebugImages/thresh_" + suffix, cvThresholdOutput);
		// Write detected rectangles
		cv::Mat rotatedRectImage = source0.clone();
		for (auto &r : rotatedRectangles)
			cv::rectangle(rotatedRectImage, cv::Point(r.center.x - r.size.width/2, r.center.y - r.size.height/2), cv::Point(r.center.x + r.size.width/2, r.center.y + r.size.height/2), cv::Scalar(0, 255, 0));
		cv::imwrite("/home/pi/DebugImages/rotatedRectImage_" + suffix, rotatedRectImage);
	}*/
	if (rotatedRectangles.size() == 0) {
		watchdog++;
		return;
	}

	int height = blurInput.rows;
	std::vector<int> prefixSum;
	for (int i = 0 ; i < height ; i++) {
		prefixSum.push_back(0);	
	}
	for (auto &r : rotatedRectangles) {
		prefixSum[r.center.y]++;
	}
	int prefix = 0;
	for (int i = 0 ; i < height ; i++) {
		prefix += prefixSum[i];	
		prefixSum[i] = prefix;
	}
	int max = 0;
	int maxIndex = 0;
	int intervalWidth = 0;
	for (int i = filterHeight + 2; i < height ; i++) {
		int intervalSum = prefixSum[i] - prefixSum[i - filterHeight - 1];
		if (intervalSum > max) {
			max = intervalSum;
			maxIndex = i;
			intervalWidth = 0;
		}else if (intervalSum == max)
			intervalWidth++;
	}
	int targetY = maxIndex - (intervalWidth/2);
	int lowerBound = prefixSum[maxIndex - filterHeight];
	int upperBound = prefixSum[maxIndex] - prefixSum[maxIndex - filterHeight];
	// trim vector
	rotatedRectangles.erase(rotatedRectangles.begin(), rotatedRectangles.begin() + lowerBound);
	rotatedRectangles.erase(rotatedRectangles.begin() + upperBound + 1, rotatedRectangles.end() + 1);


	/*if (watchdog % 100*20 == 0) {
		cv::Mat rotatedRectImage = source0.clone();
		std::cout << "Size: " << rotatedRectangles.size();
		for (auto &r : rotatedRectangles)
			cv::rectangle(rotatedRectImage, cv::Point(r.center.x - r.size.width/2, r.center.y - r.size.height/2), cv::Point(r.center.x + r.size.width/2, r.center.y + r.size.height/2), cv::Scalar(0, 255, 0));
		cv::imwrite("/home/pi/DebugImages/filteredRotatedRectImage_" + suffix, rotatedRectImage);
	}*/

	if (rotatedRectangles.size() == 0) {
		watchdog++;
		return;
	}
	double avgWidth = 0;
	double avgHeight = 0;
	for (auto &r : rotatedRectangles) {
		avgWidth += r.size.width;
		avgHeight += r.size.height;
	}
	avgWidth /= (double)rotatedRectangles.size();
	avgHeight /= (double)rotatedRectangles.size();

	std::vector<cv::RotatedRect> stripes;
	for (auto &r : rotatedRectangles) {
		double widthDiff = pow(avgWidth - r.size.width, 2);
		double heightDiff = pow(avgWidth - r.size.width, 2);

		if (widthDiff + heightDiff < deviationThresh)
			stripes.push_back(r);
	}

	double avgX = 0.0;
	double avgY = 0.0;

	for (auto &r : stripes) {
		avgX += r.center.x;
		avgY += r.center.y;
	}

	cv::RotatedRect highest;

	if (stripes.size() > 0) {
		avgX /= (double)stripes.size();
		avgY /= (double)stripes.size();
		
		highest = stripes.back();
	}
	else {
		watchdog++;
		return;
	}



	//std::cout << "Highest: " << highest.center.y << std::endl;
	//std::cout << "Lowest: " << lowest.center.y << std::endl;

	

	/*if (watchdog % 100*20 == 0) {
		cv::Mat stripesImage = source0.clone();
		for (auto &r : stripes)
			cv::rectangle(stripesImage, cv::Point(r.center.x - r.size.width/2, r.center.y - r.size.height/2), cv::Point(r.center.x + r.size.width/2, r.center.y + r.size.height/2), cv::Scalar(0, 255, 0));
		cv::drawMarker(stripesImage, cv::Point(avgX, avgY), cv::Scalar(0, 0, 255));
		cv::imwrite("/home/pi/DebugImages/stipesImage_" + suffix, stripesImage);
	}*/


	UpdateVisionNetworkTable(avgX, avgY, avgWidth, avgHeight, stripes.size(), source0.rows/2 - highest.center.y , avgX - source0.cols/2);
}

/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* GripPipeline::GetHsvThresholdOutput(){
	return &(this->hsvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* GripPipeline::GetBlurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a CV_Threshold.
 * @return Mat output from CV_Threshold.
 */
cv::Mat* GripPipeline::GetCvThresholdOutput(){
	return &(this->cvThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::GetFindContoursOutput(){
	return &(this->findContoursOutput);
}
/**
 * This method is a generated getter for the output of a Convex_Hulls.
 * @return ContoursReport output from Convex_Hulls.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::GetConvexHullsOutput(){
	return &(this->convexHullsOutput);
}
	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void GripPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void GripPipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type) {
			case BOX:
				kernelSize = 2 * radius + 1;
				cv::blur(input,output,cv::Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				cv::medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				cv::bilateralFilter(input, output, -1, radius, radius);
				break;
        }
	}
	/**
	 * Apply a fixed-level threshold to each array element in an image.
	 * @param src Image to threshold.
	 * @param thresh threshold value.
	 * @param maxVal Maximum value for THRES_BINARY and THRES_BINARY_INV.
	 * @param type Type of threshold to apply.
	 * @param dst output Image.
	 */
	void GripPipeline::cvThreshold(cv::Mat &src, double thresh, double maxVal, int type, cv::Mat &dst) {
		cv::threshold(src, dst, thresh, maxVal, type);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void GripPipeline::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Compute the convex hulls of contours.
	 *
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	void GripPipeline::convexHulls(std::vector<std::vector<cv::Point> > &inputContours, std::vector<std::vector<cv::Point> > &outputContours) {
		std::vector<std::vector<cv::Point> > hull (inputContours.size());
		outputContours.clear();
		for (size_t i = 0; i < inputContours.size(); i++ ) {
			cv::convexHull(cv::Mat((inputContours)[i]), hull[i], false);
		}
		outputContours = hull;
	}



} // end grip namespace

