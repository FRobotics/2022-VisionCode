#pragma once
#include "vision/VisionPipeline.h"

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>
#include <cstdint>
#include <algorithm>
// NEW CODE
#include <wpi/raw_ostream.h>

double hsvThresholdHue[2];
double hsvThresholdSaturation[2];
double hsvThresholdValue[2];

double cameraConstX, cameraConstY;

namespace grip {
int watchdog = 0;
int filterHeight = 100;
double deviationThresh = 400;

nt::NetworkTableInstance * nt;

nt::NetworkTableEntry hsvThresholdEntries[6];
nt::NetworkTableEntry filterEntries[2];
/*
  Entries are:
0: Vertical Filter Width
1: Deviation Threshold
*/

nt::NetworkTableEntry outputEntries[9];
/*
  Entries are:
0 : Average X
1 : Average Y
2 : Average Width
3 : Average Height
4 : Distance --- changed to Y offset in pixels from center of screen.
5 : Stripe Count
6: Height difference between highest and lowest strip
7: X offset in px from center of highest strip
8: Watchdog
 */
nt::NetworkTableEntry outputFound;


void CorrectRect(cv::RotatedRect &rect);
void SetThreshold();
void UpdateVisionNetworkTable(double avgX, double avgY, double avgWidth, double avgHeight, double stripeCount);
bool SortRect(cv::RotatedRect &a, cv::RotatedRect &b);



/**
* A representation of the different types of blurs that can be used.
*
*/
enum BlurType {
	BOX, GAUSSIAN, MEDIAN, BILATERAL
};
/**
* GripPipeline class.
* 
* An OpenCV pipeline generated by GRIP.
*/
class GripPipeline : public frc::VisionPipeline {
	private:
		cv::Mat hsvThresholdOutput;
		cv::Mat blurOutput;
		cv::Mat cvThresholdOutput;
		std::vector<std::vector<cv::Point> > findContoursOutput;
		std::vector<std::vector<cv::Point> > convexHullsOutput;
		void hsvThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void blur(cv::Mat &, BlurType &, double , cv::Mat &);
		void cvThreshold(cv::Mat &, double , double , int , cv::Mat &);
		void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
		void convexHulls(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &);

	public:
		GripPipeline();
		void Process(cv::Mat& source0) override;
		void SetThreshold(nt::NetworkTableInstance * nt);
		cv::Mat* GetHsvThresholdOutput();
		cv::Mat* GetBlurOutput();
		cv::Mat* GetCvThresholdOutput();
		std::vector<std::vector<cv::Point> >* GetFindContoursOutput();
		std::vector<std::vector<cv::Point> >* GetConvexHullsOutput();
};


} // end namespace grip


