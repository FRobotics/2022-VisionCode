#include "GripPipeline.h"

#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

namespace grip {

// ------- global variables..
static double hsvThresholdHue[2] = { 40.0, 112.0 };
static double hsvThresholdSaturation[2] = { 60.0, 233.0 };
static double hsvThresholdValue[2] = { 216.0, 255.0 };

static double cameraConstX = 320.0 / 14.0;
static double cameraConstY = 24.0;

//hsvThresholdHue[1] = 112.0 ;
//hsvThresholdHue[0] = 40.0;
//hsvThresholdSaturation[1] = 233.0;
//hsvThresholdSaturation[0] = 60.0;
//hsvThresholdValue[1] = 255.0 ;
//hsvThresholdValue[0] =  216.0;
//cameraConstY = 24.0;
//cameraConstX = 320.0/14.0;

static int watchdog = 0;
static int filterHeight = 30;
static double deviationThresh = 400;

static nt::NetworkTableInstance * nt;

static nt::NetworkTableEntry targetFound;
static nt::NetworkTableEntry hsvThresholdEntries[6];
static nt::NetworkTableEntry hsvThresholdEntriesFB[6];
static nt::NetworkTableEntry filterEntries[2];
/*
  Entries are:
0: Vertical Filter Width
1: Deviation Threshold
*/

static nt::NetworkTableEntry outputEntries[9];
/*
  Entries are:
0 : Average X
1 : Average Y
2 : Average Width
3 : Average Height
4 : Distance
5 : Stripe Count
6: Height difference between highest and lowest strip
7: X offset in px from center of highest strip
8: Watchdog
 */


// -------- global functions

// Standardizes the angles for scoring purposes (not needed for this algorithm, but kept for future use
void CorrectRect(cv::RotatedRect &rect) {
	if (rect.size.width > rect.size.height)
		return;
	double tempHeight = rect.size.height;
	rect.size.height = rect.size.width;
	rect.size.width = tempHeight;

	rect.angle = 90 + rect.angle;
}


// get the entry numbers for our network table variables.
void GetVisionNetworkTableEntries( nt::NetworkTableInstance &myNtInst ) {

	grip::nt = &myNtInst;

	grip::hsvThresholdEntries[0] = grip::nt->GetEntry("/vision/HSV/hueLow");      
  	grip::hsvThresholdEntries[1] = grip::nt->GetEntry("/vision/HSV/hueHigh");
  	grip::hsvThresholdEntries[2] = grip::nt->GetEntry("/vision/HSV/satLow");
  	grip::hsvThresholdEntries[3] = grip::nt->GetEntry("/vision/HSV/satHigh");
  	grip::hsvThresholdEntries[4] = grip::nt->GetEntry("/vision/HSV/valLow");
  	grip::hsvThresholdEntries[5] = grip::nt->GetEntry("/vision/HSV/valHigh");

	grip::hsvThresholdEntriesFB[0] = grip::nt->GetEntry("/vision/HSV/FB_hueLow");      
  	grip::hsvThresholdEntriesFB[1] = grip::nt->GetEntry("/vision/HSV/FB_hueHigh");
  	grip::hsvThresholdEntriesFB[2] = grip::nt->GetEntry("/vision/HSV/FB_satLow");
  	grip::hsvThresholdEntriesFB[3] = grip::nt->GetEntry("/vision/HSV/FB_satHigh");
  	grip::hsvThresholdEntriesFB[4] = grip::nt->GetEntry("/vision/HSV/FB_valLow");
  	grip::hsvThresholdEntriesFB[5] = grip::nt->GetEntry("/vision/HSV/FB_valHigh");

  	grip::outputEntries[0] = grip::nt->GetEntry("/vision/averageX");
  	grip::outputEntries[1] = grip::nt->GetEntry("/vision/averageY");
  	grip::outputEntries[2] = grip::nt->GetEntry("/vision/averageWidth");
  	grip::outputEntries[3] = grip::nt->GetEntry("/vision/averageHeight");
  	grip::outputEntries[4] = grip::nt->GetEntry("/vision/distance");
  	grip::outputEntries[5] = grip::nt->GetEntry("/vision/stripeCount");
  	grip::outputEntries[6] = grip::nt->GetEntry("/vision/stripHeightDiff");
  	grip::outputEntries[7] = grip::nt->GetEntry("/vision/stripXOffset");
  	grip::outputEntries[8] = grip::nt->GetEntry("/vision/watchdog");

  	grip::filterEntries[0] = grip::nt->GetEntry("/vision/filterHeight");
  	grip::filterEntries[1] = grip::nt->GetEntry("/vision/deviationThreshold");

  	grip::targetFound = grip::nt->GetEntry("/vision/targetFound");

}

// Updates interal values from NT table (runs every second)
void FetchVisionNetworkTable() {
	try {

		// --------write the feedback values first (This gives a second for stuff to happen.)
		grip::hsvThresholdEntriesFB[0].SetDouble(grip::hsvThresholdHue[0]); 
		grip::hsvThresholdEntriesFB[1].SetDouble(grip::hsvThresholdHue[1]);
		grip::hsvThresholdEntriesFB[2].SetDouble(grip::hsvThresholdSaturation[0]);
		grip::hsvThresholdEntriesFB[3].SetDouble(grip::hsvThresholdSaturation[1]);
		grip::hsvThresholdEntriesFB[4].SetDouble(grip::hsvThresholdValue[0]);
		grip::hsvThresholdEntriesFB[5].SetDouble(grip::hsvThresholdValue[1]);

		// --------read values for tuning
		grip::filterHeight = grip::filterEntries[0].GetDouble(grip::filterHeight);
		grip::deviationThresh = grip::filterEntries[1].GetDouble(grip::deviationThresh);

		grip::hsvThresholdHue[0] = grip::hsvThresholdEntries[0].GetDouble(grip::hsvThresholdHue[0]); 
		grip::hsvThresholdHue[1] = grip::hsvThresholdEntries[1].GetDouble(grip::hsvThresholdHue[1]);
		grip::hsvThresholdSaturation[0] = grip::hsvThresholdEntries[2].GetDouble(grip::hsvThresholdSaturation[0]);
		grip::hsvThresholdSaturation[1] = grip::hsvThresholdEntries[3].GetDouble(grip::hsvThresholdSaturation[1]);
		grip::hsvThresholdValue[0] = grip::hsvThresholdEntries[4].GetDouble(grip::hsvThresholdValue[0]);
		grip::hsvThresholdValue[1] = grip::hsvThresholdEntries[5].GetDouble(grip::hsvThresholdValue[1]);


		// wpi::errs() << "###Read Vars   Hue 0" << grip::hsvThresholdHue[0] << "\n";
		// wpi::errs() << "...Read Vars   Hue 1" << grip::hsvThresholdHue[1] << "\n";
		// wpi::errs() << "...Read Vars   Sat 0" << grip::hsvThresholdSaturation[0] << "\n";
		// wpi::errs() << "...Read Vars   Sat 1" << grip::hsvThresholdSaturation[1] << "\n";
		// wpi::errs() << "...Read Vars   Val 0" << grip::hsvThresholdValue[0] << "\n";
		// wpi::errs() << "...Read Vars   Val 1" << grip::hsvThresholdValue[1] << "\n";
	}
	catch (std::exception& trappedErrorCode){
		wpi::errs() << "FetchVisionNetworkTable -  trapped error" << trappedErrorCode.what() << "\n";
	}
	catch(...) {
		wpi::errs() << "FetchVisionNetworkTable -  trapped error - unknown \n";
	}

}

// Output values to network tables if target is not found.
void UpdateVisionNetworkTableNotFound() {

	targetFound.SetBoolean(false);
	// outputEntries[8].SetDouble(watchdog); // MAYBE NOT THIS ONE IN CASE IT USED AS A FOUND INDICATOR.

}


// Outputs values from algorithm (runs every tick)
void UpdateVisionNetworkTable(double avgX, double avgY, double avgWidth, double avgHeight, double stripeCount, double heightDiff, 
				double xOffsetPx) {

	try {
	outputEntries[0].SetDouble(avgX);
	outputEntries[1].SetDouble(avgY);
	outputEntries[2].SetDouble(avgWidth);
	outputEntries[3].SetDouble(avgHeight);
	outputEntries[4].SetDouble(123.0); // TODO calc dist
	outputEntries[5].SetDouble(stripeCount);
	outputEntries[6].SetDouble(heightDiff);
	outputEntries[7].SetDouble(xOffsetPx);
	outputEntries[8].SetDouble(watchdog++);
	targetFound.SetBoolean(true);
	nt->Flush();
	}
	catch (std::exception& trappedErrorCode){
		wpi::errs() << "UpdateVisionNetworkTable -  trapped error" << trappedErrorCode.what() << "\n";
	}
	catch(...) {
		wpi::errs() << "UpdateVisionNetworkTable -  trapped error - unknown \n";
	}
}
bool SortRect(cv::RotatedRect &a, cv::RotatedRect &b) {
	return a.center.y < b.center.y;
}

/**
* Runs an iteration of the pipeline and updates outputs.
*/

// ======== class functions


// Constructor
GripPipeline::GripPipeline() {

}


void GripPipeline::Process(cv::Mat& source0){
	try {

	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThresholdInput = source0;
	hsvThreshold( hsvThresholdInput, grip::hsvThresholdHue, grip::hsvThresholdSaturation, grip::hsvThresholdValue, 
				this->hsvThresholdOutput);

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

	// --------added just in case.
	if (convexHullsContours.size() == 0) {
		watchdog++;
		UpdateVisionNetworkTableNotFound();
		return;
	}


	std::vector<cv::RotatedRect> rotatedRectangles;
	
	for (unsigned int i = 0 ; i < convexHullsContours.size() ; i++) {
		rotatedRectangles.push_back(cv::minAreaRect(convexHullsContours[i]));
		CorrectRect(rotatedRectangles[i]);
	}


	sort(rotatedRectangles.begin(), rotatedRectangles.end(), SortRect);

	/*
	std::string suffix = std::to_string(watchdog) + ".png";

		if (watchdog % 100*20 == 0) {
		// Write debug image files
		cv::imwrite("/home/pi/DebugImages/src_" + suffix, source0);
		cv::imwrite("/home/pi/DebugImages/hsv_" + suffix, hsvThresholdOutput);
		cv::imwrite("/home/pi/DebugImages/blur_" + suffix, blurOutput);
		cv::imwrite("/home/pi/DebugImages/erode_" + suffix, cvErodeOutput);
		cv::imwrite("/home/pi/DebugImages/thresh_" + suffix, cvThresholdOutput);
		// Write detected rectangles
		cv::Mat rotatedRectImage = source0.clone();
		for (auto &r : rotatedRectangles)
			cv::rectangle(rotatedRectImage, 
				cv::Point(r.center.x - r.size.width/2, r.center.y - r.size.height/2), 
				cv::Point(r.center.x + r.size.width/2, r.center.y + r.size.height/2), 
				cv::Scalar(0, 255, 0));
		cv::imwrite("/home/pi/DebugImages/rotatedRectImage_" + suffix, rotatedRectImage);
	}*/

	if (rotatedRectangles.size() == 0) {
		watchdog++;
		UpdateVisionNetworkTableNotFound();
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
			cv::rectangle(rotatedRectImage, 
				cv::Point(r.center.x - r.size.width/2, r.center.y - r.size.height/2), 
				cv::Point(r.center.x + r.size.width/2, r.center.y + r.size.height/2), 
				cv::Scalar(0, 255, 0));
		cv::imwrite("/home/pi/DebugImages/filteredRotatedRectImage_" + suffix, rotatedRectImage);
	}*/

	if (rotatedRectangles.size() == 0) {
		watchdog++;
		UpdateVisionNetworkTableNotFound();
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
		UpdateVisionNetworkTableNotFound();
		return;
	}



	//std::cout << "Highest: " << highest.center.y << std::endl;
	//std::cout << "Lowest: " << lowest.center.y << std::endl;

	

	/*if (watchdog % 100*20 == 0) {
		cv::Mat stripesImage = source0.clone();
		for (auto &r : stripes)
			cv::rectangle(stripesImage, 
				cv::Point(r.center.x - r.size.width/2, r.center.y - r.size.height/2), 
				cv::Point(r.center.x + r.size.width/2, r.center.y + r.size.height/2), 
				cv::Scalar(0, 255, 0));
		cv::drawMarker(stripesImage, cv::Point(avgX, avgY), cv::Scalar(0, 0, 255));
		cv::imwrite("/home/pi/DebugImages/stipesImage_" + suffix, stripesImage);
	}*/


	UpdateVisionNetworkTable( avgX, avgY, avgWidth, avgHeight, stripes.size(), 
					source0.rows/2 - highest.center.y , avgX - source0.cols/2);

	// -------- force a little breathing room for other threads.
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	} /** end of try */
	catch (std::exception& trappedErrorCode){
		UpdateVisionNetworkTableNotFound();
		wpi::errs() << "Pipeline process trapped error" << trappedErrorCode.what() << "\n";
	}
	catch(...) {
		UpdateVisionNetworkTableNotFound();
		wpi::errs() << "Pipeline process trapped error - unknown \n";
	}
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

