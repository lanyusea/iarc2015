/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	neg-marker
* Brief:
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/4 21:00
* History:
************************************************************************/
// opencv lib
#include <opencv2/opencv.hpp>

// CT lib
#include "CompressiveTracker.h"
// ros 
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// msg
#include "ukftest/markerInfo.h"

using namespace cv;
using namespace std;

cv_bridge::CvImagePtr cv_ptr;
Mat InImage;

ukftest::markerInfo outMsg;

ros::Publisher pub;


//// tracker
CompressiveTracker ct;
Rect box; // tracking object
bool gotBB = false;
bool isInit = false;
bool drawing_box = false;


void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void mouseHandler(int event, int x, int y, int flags, void *param);

int main(int argc,char **argv)
{

	// !!!!Register mouse callback to draw the tracking box
	namedWindow("CT", CV_WINDOW_AUTOSIZE);
	setMouseCallback("CT", mouseHandler, NULL);


	ros::init(argc, argv, "neg_marker_test_node");
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
	Mat frame;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	frame = cv_ptr->image;
	resize(frame,frame,Size(320,240));
	//cvtColor(frame, frame, CV_GRAY2RGB);

	/////////////////////////////////////////////////////////////////////////////////
	//mine 

	if(gotBB && !isInit)
	{
		// Remove callback
		setMouseCallback("CT", NULL, NULL);
		printf("Initial Tracking Box = x:%d y:%d h:%d w:%d\n", box.x, box.y, box.width, box.height);
		// CT initialization
		ct.init(frame, box);
		isInit = !isInit;
	}
	if(gotBB)
	{
		ct.processFrame(frame, box);
		// Draw Points
		rectangle(frame, box, Scalar(0,0,255));
	}
	imshow("CT", frame);
	waitKey(1);
	/////////////////////////////////////////////////////////////////////
	// get board position
	if (!gotBB)
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

		
		float x=(box.x + box.x + box.width )/2.0f;
		float y=(box.y + box.y + box.height)/2.0f;

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

void mouseHandler(int event, int x, int y, int flags, void *param){
  switch( event ){
  case CV_EVENT_MOUSEMOVE:
    if (drawing_box){
        box.width = x-box.x;
        box.height = y-box.y;
    }
    break;
  case CV_EVENT_LBUTTONDOWN:
    drawing_box = true;
    box = Rect( x, y, 0, 0 );
    break;
  case CV_EVENT_LBUTTONUP:
    drawing_box = false;
    if( box.width < 0 ){
        box.x += box.width;
        box.width *= -1;
    }
    if( box.height < 0 ){
        box.y += box.height;
        box.height *= -1;
    }
    gotBB = true;
    break;
  }
}
