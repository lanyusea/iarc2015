/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	neg_estimation_node.cpp
* Brief: neg estimation node for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/8 14:27
* History: modified from ROS my ukf
************************************************************************/

#include "laser_neg_estimation_node.h"

using namespace ros;
using namespace std;

namespace estimation
{
	OdomEstimationNode::OdomEstimationNode(): 
		imu_active_(false),
		vo_active_(false),
		laser_active_(false),
		imu_initializing_(false),
		vo_initializing_(false),
		laser_initializing_(false),
		imu_callback_counter_(0),
		vo_callback_counter_(0),
		laser_callback_counter_(0),
		ukf_sent_counter_(0)
	{
		ros::NodeHandle nh_private("~");
		ros::NodeHandle nh;

		// paramters
		nh_private.param("output_frame", output_frame_, std::string("odom_combined"));
		nh_private.param("sensor_timeout", timeout_, 2.0);
		nh_private.param("imu_used",  imu_used_, true);
		nh_private.param("laser_used", laser_used_, true);
		nh_private.param("vo_used", vo_used_, false);
		nh_private.param("debug",   debug_, false);

		
		nh_private.param("freq", freq, 10.0); 

		timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

		// advertise our estimation
		pose_pub_ = nh_private.advertise<ukftest::ukfData> (output_frame_, 10);

		// initialize
		filter_stamp_ = Time::now();

		// subscribe to imu messages
		if (imu_used_){
			ROS_DEBUG("Imu sensor can be used");
			imu_sub_ = nh.subscribe("uav_imu", 10,  &OdomEstimationNode::imuCallback, this);
		}
		else ROS_DEBUG("Imu sensor will NOT be used");

		// subscribe to vo messages
		if (vo_used_){
			ROS_DEBUG("VO sensor can be used");
			vo_sub_ = nh.subscribe("marker_pose", 10, &OdomEstimationNode::voCallback, this);
		}
		else ROS_DEBUG("VO sensor will NOT be used");
		// subscribe to laser messages
		if (laser_used_){
			ROS_DEBUG("Laser sensor can be used");
			laser_sub_ = nh.subscribe("laser_point", 10, &OdomEstimationNode::laserCallback, this);
		}
		else ROS_DEBUG("Laser sensor will NOT be used");
	};
	// destructor
	OdomEstimationNode::~OdomEstimationNode(){};

	// callback function for imu data
	void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
	{
		assert(imu_used_);
		
		imu_stamp_  = Time::now();
		
		if(imu_callback_counter_ >1) 
		{
			float freq_imu = 1.0 / (imu_stamp_ - imu_time_).toSec();
			//std::cout << "IMU Freq: "<< freq_imu <<" Hz \n";
		}
		imu_time_ =imu_stamp_;
		
		// receive data 
		VectorXf imu_data;
		imu_data.resize(10);
		// q_eb
		imu_data(0) = imu -> orientation.w; 
		imu_data(1) = imu -> orientation.x;
		imu_data(2) = imu -> orientation.y;
		imu_data(3) = imu -> orientation.z;

		//debug 
		/*
		vector4f q_debug;
		q_debug[0] = imu_data(0);
		q_debug[1] = imu_data(1);
		q_debug[2] = imu_data(2);
		q_debug[3] = imu_data(3);
		float yaw, pitch, roll;
		quat_to_eular(&yaw, &pitch, &roll, q_debug);
		cout<< "yaw: "<< yaw/ M_PI * 180.0f<< "| pitch: "<< pitch/ M_PI * 180.0f<< "| roll: "<< roll/ M_PI * 180.0f<< endl;
		*/
		// a_eb or a_ec  where we set the unit of a to cm/s^2
		imu_data(4) = imu -> linear_a.x ;   
		imu_data(5) = imu -> linear_a.y ;  
		imu_data(6) = imu -> linear_a.z ;  

		// w_bb or w_bc
		imu_data(7) = imu -> angular_v.x/180.0f*M_PI; 
		imu_data(8) = imu -> angular_v.y/180.0f*M_PI; 
		imu_data(9) = imu -> angular_v.z/180.0f*M_PI; 
		/*
		//show debug
		std::cout<< "o.w: "<< imu_data(0)<<endl
			 << "o.x: "<< imu_data(1)<<endl
			 << "o.y: "<< imu_data(2)<<endl
			 << "o.z: "<< imu_data(3)<<endl<<endl
			 << "a.x: "<< imu_data(4)<<endl
			 << "a.y: "<< imu_data(5)<<endl
			 << "a.z: "<< imu_data(6)<<endl<<endl
			 << "w.x: "<< imu_data(7)*180.0f/M_PI<<endl
			 << "w.y: "<< imu_data(8)*180.0f/M_PI<<endl
			 << "w.z: "<< imu_data(9)*180.0f/M_PI<<endl<<endl;

		*/
		// add meas to filter
		my_filter_.addMeasurement_imu(imu_data);
    
		//for (int i = 0 ; i < imu_data.size() ; i++) std::cout<< imu_data(i)<<std::endl;
		// activate imu
		if (!imu_active_) 
		{
			imu_active_ = true;
		}

		imu_initializing_ = true;

		imu_callback_counter_++;

	};

	void OdomEstimationNode::voCallback(const VoConstPtr& vo)
	{
		assert(vo_used_);
		vo_stamp_ = Time::now();
		if(vo_callback_counter_ >1) 
		{
			float freq_vo = 1.0 / (vo_stamp_ - vo_time_).toSec();
			std::cout << "VO Freq: "<< freq_vo <<" Hz \n";
		}
		vo_time_ =vo_stamp_;
		

		// receive data 
		VectorXf vo_data;
		vo_data.resize(3);

		// uv1
		vo_data(0) = vo -> uv1.x;
		vo_data(1) = vo -> uv1.y;
		vo_data(2) = 1.0f;

		// activate vo
		if (!vo_active_ && vo -> isTracking == 1)
		{
			// add meas to filter
			

			my_filter_.addMeasurement_vo(vo_data);
			vo_active_ = true;

		}

		vo_initializing_ = true;

		vo_callback_counter_++;
	};

	void OdomEstimationNode::laserCallback(const LaserConstPtr& laser)
	{
		//cout<< "call laser!"<<endl;
		assert(laser_used_);
		laser_stamp_ = Time::now();
		//cout<<laser_callback_counter_<<endl;
		if(laser_callback_counter_ >1) 
		{
			float freq_laser = 1.0 / (laser_stamp_ - laser_time_).toSec();
			std::cout << "Laser Freq: "<< freq_laser <<" Hz \n";
		}
		laser_time_ =laser_stamp_;
		
		// activate laser
		if (!laser_active_ && laser -> isBlocking == 1)
		{
			// add meas to filter
			// receive data 
			VectorXf laser_data;
			laser_data.resize(3);

			// laser []
			laser_data(0) = laser -> x[0];
			laser_data(1) = laser -> y[0];
			laser_data(2) = 0.0f;
			cout<< laser_data <<endl;
			my_filter_.addMeasurement_laser(laser_data);
			laser_active_ = true;

		}

		laser_initializing_ = true;

		laser_callback_counter_++;
	};

	// filter loop
	void OdomEstimationNode::spin(const ros::TimerEvent& e)
	{
		ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

		// check for timing problems
		if ( (vo_active_ || vo_initializing_ ) && (imu_active_ || imu_initializing_ ) )
		{
			//ROS_INFO("vo time: %f \n imu time: %f", vo_stamp_.toSec(), imu_stamp_.toSec());
			double diff = fabs( (vo_stamp_ - imu_stamp_).toSec() );
			if (diff > 1.0) ROS_ERROR("Timestamps of vo and imu are %f seconds apart.", diff);
		}

		// check for laser timing problems
		if ( (laser_active_ || laser_initializing_ ) && (imu_active_ || imu_initializing_ ) )
		{
			//ROS_INFO("vo time: %f \n imu time: %f", vo_stamp_.toSec(), imu_stamp_.toSec());
			double diff = fabs( (laser_stamp_ - imu_stamp_).toSec() );
			if (diff > 1.0) ROS_ERROR("Timestamps of vo and imu are %f seconds apart.", diff);
		}
    
		// initial value for filter stamp; keep this stamp when no sensors are active
		filter_stamp_ = Time::now();
    
		// check which sensors are still active
		//ROS_INFO("active time: %f ",Duration(Time::now() - imu_stamp_).toSec());
		if (imu_initializing_ && Duration(Time::now() - imu_stamp_).toSec() > timeout_)
		{
			imu_active_ = false; imu_initializing_ = false;
			ROS_ERROR("Imu sensor not active any more");
		}
		if (vo_initializing_ && Duration(Time::now() - vo_stamp_).toSec() > timeout_)
		{
			vo_active_ = false; vo_initializing_ = false;
			ROS_ERROR("VO sensor not active any more");
		}
		if (laser_initializing_ && Duration(Time::now() - laser_stamp_).toSec() > timeout_)
		{
			laser_active_ = false; laser_initializing_ = false;
			ROS_ERROR("Laser sensor not active any more");
		}
		// only update filter when one of the sensors is active
		if (imu_active_ || vo_active_ || laser_active_)
		{
			// update filter at time where all sensor measurements are available
			if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
			if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
			if (laser_active_) filter_stamp_ = min(filter_stamp_, laser_stamp_);
			//ROS_INFO("filter_stamp_ time: %f ", filter_stamp_.toSec());
		}

		// initialize filer with odometry frame
		if (imu_active_ && vo_active_ && !my_filter_.isInitialized())
		{
			my_filter_.initialize(filter_stamp_);
			if (my_filter_.isInitialized())
			{
				ROS_INFO("Kalman filter initialized! \n Frequency is : %f  Hz\n", freq);
			}			
			imu_active_ = false;
			vo_active_  = false;
		}

		if (imu_active_ && laser_active_ && !my_filter_.isInitialized())
		{
			my_filter_.initialize(filter_stamp_);
			if (my_filter_.isInitialized())
			{
				ROS_INFO("Kalman filter initialized! \n Frequency is : %f  Hz\n", freq);
			}			
			imu_active_ = false;
			laser_active_  = false;
		}
      
		// update filter
		if( !my_filter_.isInitialized() )
		{
			output_.header.stamp = ros::Time::now();
			output_.isctrl		 = 0;

			pose_pub_.publish(output_);
		}

		else
		{
			if (my_filter_.update(imu_active_, vo_active_, laser_active_, filter_stamp_ ))
			{
				imu_active_ = false;
				vo_active_  = false;
				laser_active_  = false;
				// output most recent estimate and relative covariance
				if(my_filter_.getEstimate(output_tmp))
				{
					// tmp -> msg
					output_.header.stamp = ros::Time::now();
					output_.xyz.x        = output_tmp(0);
					output_.xyz.y        = output_tmp(1);
					output_.xyz.z        = output_tmp(2);
					output_.q.x          = output_tmp(4);
					output_.q.y			 = output_tmp(5);
					output_.q.z			 = output_tmp(6);
					output_.q.w			 = output_tmp(3);
					output_.v.x          = output_tmp(7);
					output_.v.y          = output_tmp(8);
					output_.v.z          = output_tmp(9);
					output_.w.x          = output_tmp(10);
					output_.w.y          = output_tmp(11);
					output_.w.z          = output_tmp(12);
					output_.a.x          = output_tmp(13);
					output_.a.y          = output_tmp(14);
					output_.a.z          = output_tmp(15);
					output_.dt           = output_tmp(16);
					output_.isctrl		 = 1;
					//std::cout<<"ukf_sent_counter_:"<<ukf_sent_counter_<<std::endl;
					
					/*
					cout << "w_b_x: "<< output_.w.x/M_PI*180.0f<<endl
					     << "w_b_y: "<< output_.w.y/M_PI*180.0f<<endl		
					     << "w_b_z: "<< output_.w.z/M_PI*180.0f<<endl<<endl;
					*/
					//debug yaw roll pitch

					/*
					vector4f q_debug;
					q_debug[0] = output_.q.w;
					q_debug[1] = output_.q.x;
					q_debug[2] = output_.q.y;
					q_debug[3] = output_.q.z;
					float yaw,pitch,roll;
                            		quat_to_eular(&yaw, &pitch, &roll, q_debug);
					cout<< "yaw  : "<<yaw<<endl
			    		    << "pitch: "<<pitch<<endl
			    		    << "roll : "<<roll<<endl<<endl;
					*/
					pose_pub_.publish(output_);
					ukf_sent_counter_++;
				}
	
			}
		}
	};
};

using namespace estimation;

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "robot_pose_ukf");
	std::cout<< "odom node start!\n";
	// create filter class
	OdomEstimationNode my_filter_node;

	ros::spin();
	std::cout<< "odom node stop!\n";
	return 0;
}
