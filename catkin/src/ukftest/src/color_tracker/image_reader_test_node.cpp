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
	ros::init(argc, argv, "image_reader_test_node");
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
	cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	frame = cv_ptr->image;
	imshow("debug",frame);
	waitKey(1);
}
