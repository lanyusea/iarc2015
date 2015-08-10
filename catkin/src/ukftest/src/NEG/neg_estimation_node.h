/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	neg_estimation_node.h
* Brief: neg estimation node for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/8 14:00
* History: modified from ROS my ukf
************************************************************************/

#pragma once

//ros 
#include <ros/ros.h>

//message
#include "serial_to_uav/UAV.h"
#include "ukftest/markerInfo.h"
#include <ukftest/ukfData.h>

#include "neg_estimation.h"
#include <boost/thread/mutex.hpp>

namespace estimation
{
	typedef boost::shared_ptr<ukftest::markerInfo const> VoConstPtr;
	typedef boost::shared_ptr<serial_to_uav::UAV const> ImuConstPtr;

	class OdomEstimationNode
	{
	public:
		/// constructor
		OdomEstimationNode();

		/// destructor
		~OdomEstimationNode();
	private:
		/// the mail filter loop that will be called periodically
		void spin(const ros::TimerEvent& e);

		/// callback function for vo data
		void voCallback(const VoConstPtr& vo);

		/// callback function for imu data
		void imuCallback(const ImuConstPtr& imu);

		ros::NodeHandle node_;
		ros::Timer timer_;
		ros::Publisher pose_pub_;
		ros::Subscriber imu_sub_, vo_sub_;

		// ukf filter
		OdomEstimation my_filter_;

		// output data
		VectorXf output_tmp;
		ukftest::ukfData output_;
		ros::Time imu_time_, vo_time_;
		ros::Time imu_stamp_, vo_stamp_, filter_stamp_;
		ros::Time imu_init_stamp_, vo_init_stamp_;

		bool imu_active_, vo_active_;
		bool imu_used_, vo_used_;
		bool imu_initializing_, vo_initializing_;

		double timeout_;
		double freq;
		double imu_covariance_,vo_covariance_;
		std::string output_frame_;
		bool debug_;
		float imu_height;
		// counters
		unsigned int imu_callback_counter_, vo_callback_counter_,ukf_sent_counter_;

	}; //class
}; //namespace

