#ifndef OPENRADAR_H
#define OPENRADAR_H

#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ukftest/Steer.h>
#include <ukftest/ukfData.h>
#include <ukftest/laserPoint.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
class OpenRadar
{
	public:
		OpenRadar(void);	
    ~OpenRadar(void);

		// this is the class entry function, msg come from ROS node
		void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
		// plot radar data to a OpenCV window
		void DrawRadarData();
		
		//this function is called once per several laser_callback call
		void laser_proc();

		// VFH use Vector Field Histogram based method to calculate safe direction and speeds
		void VFH(); 

		// This function breaks radar data to be segments
		// returns number of segments
		int RadarBreak();

		// this is a utility function used in laser_proc to pre-process the noisy data in ranges[][]
		float denoise_across_frame(int idx);


	private:
		// display related varibles
		static const int RadarImageWdith  = 720;
		static const int RadarImageHeight = 400;
		static const int x_ratio = 3;
		static const int y_ratio = 2;
		static const int DisplayRatio = 30;
		
		// this number defines how many frames of laser data to be stored
		static const int MAX_frame = 3;
	
		// how to divide laser data into bins
		static const int polarH_resolution = 5;
		static const int polarH_length = 240/polarH_resolution;
		
		// varibles to convert laser measurement to obstacle strenth
		// obstacle = constA - constB*distance_measure 
		static const float constA = 40;
		static const float constB = 0.007;
		
		// smooth window size of histogram bins
		static const int sw_size = 7;
		static const float H_m = 450;
	
		// skip first several and last several histogram bins (since they are close to boundary of sensor)
		static const int skip_bin_idx = 5; 

		// this is a varible used in display image function
		static const int line_length = 50;


		// some thresholds used by VFH
		static const int equal_block_thres = 5;
		static const int clear_thres = 320;	//100
		static const int block_thres = 320;	//300

		// thresholds used by radarbreak function
		static const int break_thres = 120;

		// assist laser measurement storage
		int receive_state;

		// laser measurement temporary storage
		float ranges[MAX_frame][1000];
		float range_tmp[MAX_frame];

		// denoised data is stored here
		int RadarRho[1082];
		float RadarTheta[1082];
		int RadarDataCnt;

		// VFH histogram bins & assistant storage
		float *H;
		int *H_c;
		int *H_c2;

		// these values are read from laser topic
		int num_data;
		float angle_max, angle_min, angle_increment;
		float range_max, range_min;
		
		// a shared varible used by denoise_across_frame function
		float variance;
		static const int smooth_window = 5;

		// ROS varibles
		ros::NodeHandle n2;
		ros::Publisher pub;
		ros::Publisher pub_xy;
		// display image
		int DisplayDx;
		int DisplayDy;
		IplImage* RadarImage;

		// output direction
		float v_angle;
		float v_scale;
		
		// time freq
		ros::Time time_old;
		bool timeInit;

		
};

#endif
