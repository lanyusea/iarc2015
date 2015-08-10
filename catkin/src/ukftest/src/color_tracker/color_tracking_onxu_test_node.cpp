/*
*    H 0-9 S 40-255 V  49-252
*
*/
// opencv lib
#include <opencv2/opencv.hpp>
// ros 
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// msg
#include "ukftest/markerInfo.h"

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
Rect colorDetector(Mat img, Mat &dst);

// ros pub
ros::Publisher pub;
// color detector
vector< vector<Point> > contours;	
vector< vector<Point> > filterContours;	
vector< Vec4i > hierarchy;	
vector< Point > hull;
// bar window
int H_min = 0,
	H_max = 13,
	S_min = 144,
	S_max = 187,
	V_min = 0,
	V_max = 255;

int 	H_min_b = 0,
	H_max_b = 94,
	S_min_b = 85,
	S_max_b = 97,
	V_min_b = 0,
	V_max_b = 255;

int main(int argc,char **argv)
{

	namedWindow( "Bar", 1 );
	createTrackbar( "H_min", "Bar", &H_min, 255 );
	createTrackbar( "H_max", "Bar", &H_max, 255 );
	createTrackbar( "S_min", "Bar", &S_min, 255 );
	createTrackbar( "S_max", "Bar", &S_max, 255 );
	createTrackbar( "V_min", "Bar", &V_min, 255 );
	createTrackbar( "V_max", "Bar", &V_max, 255 );

	// ros
	ros::init(argc, argv, "color_test_onxu_node");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub;

	imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
	pub = nh.advertise<ukftest::markerInfo>("marker_pose",200);
	ros::spin();
	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ukftest::markerInfo outMsg;
	Mat frame;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	frame = cv_ptr->image;
	
	imshow("debug",frame);
	///////////////////////////////////////////////
	Mat dstimage;
	Rect boundBox;
	boundBox = colorDetector( frame,dstimage);

	imshow("dst",dstimage);
	////////////////////////////////////////////////
	waitKey(1);
	if(boundBox.x == -1)
	{
		outMsg.header.stamp =ros::Time::now();
		outMsg.header.frame_id = "camera";
		outMsg.isTracking = 0;
		pub.publish(outMsg);
		return;
	}
	else
	{
		outMsg.header.stamp =ros::Time::now();
		outMsg.header.frame_id = "camera";

		
		float x=(boundBox.x + boundBox.x + boundBox.width )/2.0f;
		float y=(boundBox.y + boundBox.y + boundBox.height)/2.0f;


		// debug
		line(frame,Point2f(160,120),Point2f(x,y),Scalar(255,255,0),1);
		circle(frame,Point2f(x,y),3,Scalar(0,255,255),4,8);
		imshow("debug",frame);
		waitKey(1);
		
		// undistort
		Mat K=(Mat_<float>(3,3)<<
			180.2346315,0,159.5,
			0,180.2346315,119.5,
			0,0,1);

		Mat distCoeff = (Mat_<float>(5,1) << -0.276089956,0.1087447314,0.0,0.0,-0.024036442);

		Mat src(1,1,CV_32FC2);
		Mat dst(1,1,CV_32FC2);
		src.ptr<float>(0)[0] = x;
		src.ptr<float>(0)[1] = y;
	
		undistortPoints(src,dst,K,distCoeff);
	
		x = dst.ptr<float>(0)[0] * K.at<float>(0,0) + K.at<float>(0,2);
		y = dst.ptr<float>(0)[1] * K.at<float>(1,1) + K.at<float>(1,2);

		outMsg.uv1.x = x;
		outMsg.uv1.y = y;
		outMsg.uv1.z = 1.0f;
		outMsg.isTracking = 1;
		pub.publish(outMsg);
	}
}

Rect colorDetector(Mat img, Mat &dst)
{
	Mat mask = Mat::zeros(img.rows, img.cols, CV_8UC1);
	Mat mask_r = Mat::zeros(img.rows, img.cols, CV_8UC1);
	Mat mask_b = Mat::zeros(img.rows, img.cols, CV_8UC1);

	Mat frameHSV;
	GaussianBlur( img, frameHSV, Size(9, 9), 3, 3 );

	cvtColor( frameHSV, frameHSV, CV_BGR2HSV );

	//inRange(frameHSV, Scalar(0,40,49),Scalar(11,255,255), mask);
	inRange(frameHSV, Scalar(MIN(H_min,H_max),MIN(S_min,S_max),MIN(V_min,V_max)),
			Scalar(MAX(H_min,H_max),MAX(S_min,S_max),MAX(V_min,V_max)), mask_r);

	inRange(frameHSV, Scalar(H_min_b,S_min_b,V_min_b),Scalar(H_max_b,S_max_b,V_max_b), mask_b);
	bitwise_or(mask_r, mask_b, mask);

	Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
	erode(mask, mask, element);
	morphologyEx(mask, mask, MORPH_OPEN, element);
	Mat element2 = getStructuringElement(MORPH_RECT, Size(30,30));
	dilate(mask, mask, element2);
	morphologyEx(mask, mask, MORPH_CLOSE, element2);
	Mat showMask;
	img.copyTo(showMask,mask);
	img.copyTo(dst);
	imshow("Bar",showMask);


	contours.clear();
	filterContours.clear();
	hierarchy.clear();
	hull.clear();


	//findContours( mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++)
	{
		if (fabs(contourArea(Mat(contours[i]))) > 200)
		{
			filterContours.push_back(contours[i]);
		}
	}
	double maxArea = 0;
	int maxNo = -1;
	for (int i = 0;i < contours.size(); i++)
	{
		double cArea = contourArea(filterContours[i]);
		if(cArea > maxArea)
		{
			maxArea = cArea;
			maxNo = i;
		}
	}
	
	if(maxNo != -1)
	{
		convexHull(Mat(filterContours[maxNo]), hull, true);
		int hullcount = (int)hull.size();

		for (int i=0; i<hullcount-1; i++)
		{
			line(dst, hull[i+1], hull[i], Scalar(255,0,0), 2, CV_AA);
		}
		line(dst, hull[hullcount-1], hull[0], Scalar(255,0,0), 2, CV_AA);
		return boundingRect(hull);
		
	}
	/*
	for (int j=0; j<filterContours.size(); j++)
	{
		convexHull(Mat(filterContours[j]), hull, true);
		int hullcount = (int)hull.size();

		for (int i=0; i<hullcount-1; i++)
		{
			line(dst, hull[i+1], hull[i], Scalar(255,0,0), 2, CV_AA);
		}
		line(dst, hull[hullcount-1], hull[0], Scalar(255,0,0), 2, CV_AA);
	}

	if(hull.size()>0)
	{
		return boundingRect(hull);
	}
	*/
	else
	{ 
		return Rect(-1,-1,0,0);
	}
	
}
