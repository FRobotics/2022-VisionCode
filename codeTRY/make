#include "GripPipeline.h"

namespace grip {

GripPipeline::GripPipeline() {
	hsvThresholdHue[0] = 72.0;
	hsvThresholdHue[1] = 0.0;
	hsvThresholdSaturation[0] = 0.0; 
	hsvThresholdSaturation[1] = 53.0;
	hsvThresholdValue[0] = 91.7266187053599;
	hsvThresholdValue[1] =  255.0;
}
void CorrectRect(cv::RotatedRect &rect) {
	if (rect.size.width > rect.size.height)
		return;
	double tempHeight = rect.size.height;
	rect.size.height = rect.size.width;
	rect.size.width = tempHeight;

	rect.angle = 90 + rect.angle;
}

void SetThreshold() {
	hsvThresholdHue[1] = nt->GetEntry("/SmartDashboard/vision/HSV/hueHigh").GetDouble(180.0); 
	hsvThresholdHue[0] = nt->GetEntry("/SmartDashboard/vision/HSV/hueLow").GetDouble(0.0);
	hsvThresholdSaturation[1] = nt->GetEntry("/SmartDashboard/vision/HSV/satHigh").GetDouble(53.0);
	hsvThresholdSaturation[0] = nt->GetEntry("/SmartDashboard/vision/HSV/satLow").GetDouble(0.0);
	hsvThresholdValue[1] = nt->GetEntry("/SmartDashboard/vision/HSV/valHigh").GetDouble(255.0);
	hsvThresholdValue[0] = nt->GetEntry("/SmartDashboard/vision/HSV/valLow").GetDouble(91.7266187053600);
}
void UpdateVisionNetworkTable(cv::RotatedRect rect) {
	std::cout << "Center: " << rect.center.x << ", " << rect.center.y << "\n";
	nt->GetEntry("/SmartDashboard/vision/data/RotatedRectX").SetDouble(rect.center.x);
	nt->GetEntry("/SmartDashboard/vision/data/RotatedRectY").SetDouble(rect.center.y);      
	nt->GetEntry("/SmartDashboard/vision/data/RotatedRectWidth").SetDouble(rect.size.width);
	nt->GetEntry("/SmartDashboard/vision/data/RotatedRectHeight").SetDouble(rect.size.height);
	nt->GetEntry("/SmartDashboard/vision/data/RotatedRectAngle").SetDouble(rect.angle);
	nt->Flush();
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void GripPipeline::Process(cv::Mat& source0){
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

	std::vector<cv::RotatedRect> rotatedRectangles;
	std::cout << "Size of array: " << convexHullsContours.size() << "\n";
	
	double maxScore[2];
	int maxIndex = -1;
	for (unsigned int i = 0 ; i < convexHullsContours.size() ; i++) {
		rotatedRectangles.push_back(cv::minAreaRect(convexHullsContours[i]));
		//CorrectRect(rotatedRectangles[i]);
		if (rotatedRectangles[i].size.width < 20 || rotatedRectangles[i].size.height < 7) {
			continue;
		}
		double score[3];
		std::cout << "Angle: " << rotatedRectangles[i].angle << "\n";
		score[0] = (-1/81)*(rotatedRectangles[i].angle-90)*(rotatedRectangles[i].angle+90);
		score[1] = 100-abs(1-(8*rotatedRectangles[i].angle/19));
		score[2] = score[0] + score[1];
		std::cout << "Score 0: " << score[0] << " Score 1: " << score[1] << " Score 2: " << score[2] << "\n";
		
		if (score[2] > maxScore[2]) {
			maxScore[0] = score[0];
			maxScore[1] = score[1];
			maxScore[2] = score[2];
			maxIndex = i;
		}	
	}
	nt->GetEntry("/SmartDashboard/vision/angleScore").SetDouble(maxScore[0]);
	nt->GetEntry("/SmartDashboard/vision/ratioScore").SetDouble(maxScore[1]);
	nt->GetEntry("/SmartDashboard/vision/score").SetDouble(maxScore[2]);
	
	if (rotatedRectangles.size() > 0 && maxScore[2] > 60) {
		UpdateVisionNetworkTable(rotatedRectangles[maxIndex]);
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

