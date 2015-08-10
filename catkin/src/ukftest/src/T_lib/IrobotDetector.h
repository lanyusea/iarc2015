/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	irobot 探测
* Brief: 探测4次，分别canny，threshold加轮廓匹配。
* Version: 1.1
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/4/18 21:00
* History:
************************************************************************/
#pragma once
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;


// THRESHOLD 55 good for T-box
#define BINTHRESHOLD 40
#define SCALE 2
#define IMAGE_PATH "/home/aqua/iarc/obw.jpg"
#define DEBUG_ON 1

class IrobotDetector
{
public:
	Mat camMatrix,distCoeff;
	Mat raux,taux;
	Vec3f Tvec;
	Mat Rvec;
	Vec3f drawT;
	Matx33f drawR;
	vector<Point2f> marker2dPt;
	vector<Point3f> marker3dPt;
	bool guess;
	//读取外部轮廓
	Mat imC,imCbw,imEdge;
	vector<Point> contoursOO;
	vector<vector<Point> > contoursO;
	vector<Vec4i> hierarchyO;
	bool isTracking;

	IrobotDetector();
	~IrobotDetector();
	void processFrame(Mat frame);
	Point2f pCenter;

private:
	RNG rng;
	vector<Point2f> markCorners2d;
	vector<cv::Point> goodKeyPt;
	int goodKeyPtTime;
	Size m_markerSize;
	double matchMax;
	double matchN;
	int goodContour;
	Mat imgray,imgrayTemplate,imbw,imcontour,imgrayPreciseCorners;
	Mat imTeplate[8];
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	bool tempFlag;
	vector<Point2f> harrisCorners;
	vector<cv::Point> matchContour;

	void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour);
	void trackKeyPt(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect);
	void estimatePosition(vector<cv::Point3f> Pt3d,vector<cv::Point2f> Pt2d);
	void drawPlain(Mat& _frame);
	void findImageContours();
	void findIrobot();

	void ContourBasedDetector();
	void BinBasedDetector();
	void HarrisBasedDetector();
	void MatchTemplateBaseDetector();
	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	void drawGoodpt(Mat image,vector<cv::Point> goodpt,Scalar a);

};
