/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	neg_estimation.h
* Brief: neg estimation for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/8 14:00
* History: modified from ROS my ukf
************************************************************************/

#pragma once

//filter
#include "SRUKF.h"

// ros
#include <ros/ros.h>
#include <ros/console.h>
// eigen lib
#include <eigen3/Eigen/Dense>
// opencv lib
#include <opencv2/opencv.hpp>
// 30cm
#define FIXED_HEIGHT 370.0f

using namespace ros;
using namespace cv;
namespace estimation
{

	void process_func(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt);
	void measure_func_imu(VectorXf& z_t, const VectorXf& x_t);
	void measure_func_voimu(VectorXf& z_t, const VectorXf& x_t);
	void measure_func_vo(VectorXf& z_t, const VectorXf& x_t);
	void measure_func_laser(VectorXf& z_t, const VectorXf& x_t);
	void measure_func_laserimu(VectorXf& z_t, const VectorXf& x_t);

	class OdomEstimation
	{
	public:
		OdomEstimation();
		~OdomEstimation();

		SRUKF* ukf;

		// create VO MODEL
		MatrixXf voR;
		MatrixXf voQ;
		// create VO&IMU MODEL
		MatrixXf voImuR;
		MatrixXf voImuQ;
		// create IMU MODEL
		MatrixXf imuR;
		MatrixXf imuQ;
		// create LASER MODEL
		MatrixXf laserR;
		MatrixXf laserQ;
		// create LASER&IMU MODEL
		MatrixXf laserImuR;
		MatrixXf laserImuQ;

		bool update(bool imu_active, bool vo_active, bool laser_active,const Time& filter_time);

		void initialize(const Time& time);

		bool isInitialized() {return filter_initialized_;};

		bool isValidFdata();
		
		bool getEstimate(VectorXf& estimate);

		void addMeasurement_vo(const VectorXf& meas_vo);

		void addMeasurement_imu(const VectorXf& meas_imu);

		void addMeasurement_laser(const VectorXf& meas_laser);

	private:

		double q_; //Std of process 
		double r_; //Std of measurement
		double n_; //n
		double m_; //m
		float dt;
		bool filter_initialized_, imu_initialized_, vo_initialized_, laser_initialized_;
		float a_eb[3];
		ros::Time filter_time_old_;
		// SRUKF related variables
		void (*f_func)(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt);   // function pointer to process function
		void (*h_func)(VectorXf& z_t, const VectorXf& x_t);    

		//tilt and torsion to get a new T_tc
		void getCoVoImu(VectorXf& filter_compose, const VectorXf& meas_vo, const VectorXf& meas_imu);

		void getXYZ(VectorXf& T_et, const VectorXf& uv1, const VectorXf& meas_imu, const float heigt);
		
		void getLaserXYZ(VectorXf& T_et, const VectorXf& T_bt, const VectorXf& meas_imu);

		bool preInitialize(const ros::Time& time);

		VectorXf s_init, s_meas;
		VectorXf vo_meas, vo_init_meas;
		VectorXf imu_meas, imu_init_meas;
		VectorXf laser_meas, laser_init_meas;
		VectorXf init_compose, now_compose;
		VectorXf filter_data;

		VectorXf T_et_init, T_et_now;
	};  // class
}; // namespace
