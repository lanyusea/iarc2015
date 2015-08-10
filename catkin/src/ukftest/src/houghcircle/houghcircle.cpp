/*
*	Hough Circle detector 
*	by YINGJiahang mailto  yingjiahang@gmail.com
*
	Parameters:	
	image 每 8-bit, single-channel, grayscale input image.
	circles 每 Output vector of found circles. Each vector is encoded as a 3-element floating-point vector (x, y, radius) .
	circle_storage 每 In C function this is a memory storage that will contain the output sequence of found circles.
	method 每 Detection method to use. Currently, the only implemented method is CV_HOUGH_GRADIENT , which is basically 21HT , described in [Yuen90].
	dp 每 Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
	minDist 每 Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
	param1 每 First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
	param2 每 Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
	minRadius 每 Minimum circle radius.
	maxRadius 每 Maximum circle radius.
*/
// opencv lib
#include <opencv2/opencv.hpp>
// ros 
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
int houghcircledetector(Mat src,Mat &dst);

int main(int argc,char **argv)
{
	ros::init(argc, argv, "houghcircle_test_node");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub;

	imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
	ros::spin();
	return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	Mat frame,image;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
	frame = cv_ptr->image;
	resize(frame,frame,Size(320,240));
	cvtColor(frame, frame, CV_GRAY2RGB);

	cout<<"No. of circle: "<<houghcircledetector(frame,image)<<endl;

	imshow("debug",image);
	waitKey(1);
}


int houghcircledetector(Mat src,Mat &dst)
{
	Mat src_gray;
	resize(src,src,Size(src.cols/1,src.rows/1));
	if( !src.data )
	{ return -1; }

	/// Convert it to gray
	cvtColor( src, src_gray, CV_BGR2GRAY );

	/// Reduce the noise so we avoid false circle detection
	GaussianBlur( src_gray, src_gray, Size(7, 7), 2, 2 );
	//imshow("GaussianBlur",src_gray);
	vector<Vec3f> circles;

	/// Apply the Hough Transform to find the circles
	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, 10,
					100, 30, 
					50, src_gray.rows/2);

	/// Draw the circles detected
	for( size_t i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle( src, center, 3, Scalar(255,0,0), -1, 8, 0 );
		// circle outline
		circle( src, center, radius, Scalar(0,255,0), 3, 8, 0 );
	}

	src.copyTo(dst);
	return circles.size();

}

