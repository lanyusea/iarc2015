/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	neg_estimation.cpp
* Brief: neg estimation for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/8 14:00
* History:
************************************************************************/

#include "laser_neg_estimation.h"

namespace estimation
{
	void process_func(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt)
	{
		x_t(0) = x_t_1(0) + x_t_1(3) * dt;
		x_t(1) = x_t_1(1) + x_t_1(4) * dt;
		x_t(2) = x_t_1(2) + x_t_1(5) * dt;

		x_t(3) = x_t_1(3) - a[0] * dt;
		x_t(4) = x_t_1(4) - a[1] * dt;
		x_t(5) = x_t_1(5) - a[2] * dt;
	};

	void measure_func_voimu(VectorXf& z_t, const VectorXf& x_t)
	{
		int state_size = z_t.size();
		for (int i = 0; i < state_size; i++)
			z_t(i) = x_t(i);
	};

	void measure_func_imu(VectorXf& z_t, const VectorXf& x_t)
	{
		int state_size = z_t.size();
		z_t(0) = 0;
		z_t(1) = 0;
		z_t(2) = 0;
		for (int i = 3; i < state_size; i++)
			z_t(i) = x_t(i);
	};

	void measure_func_vo(VectorXf& z_t, const VectorXf& x_t)
	{
		int state_size = z_t.size();
		for (int i = 0; i < z_t.size(); i++)
			z_t(i) = x_t(i);
	};

	void measure_func_laser(VectorXf& z_t, const VectorXf& x_t)
	{
		int state_size = z_t.size();
		for (int i = 0; i < z_t.size(); i++)
			z_t(i) = x_t(i);
	};

	void measure_func_laserimu(VectorXf& z_t, const VectorXf& x_t)
	{
		int state_size = z_t.size();
		for (int i = 0; i < state_size; i++)
			z_t(i) = x_t(i);
	};

	OdomEstimation::OdomEstimation():
		filter_initialized_(false),
		imu_initialized_(false),
		vo_initialized_(false),
		laser_initialized_(false)
	{
		ros::NodeHandle nh_private("~");

		// create SYSTEM MODEL
		nh_private.param("q", q_, 0.01);
		nh_private.param("r", r_, 2.0);
		nh_private.param("n", n_, 6.0);
		nh_private.param("m", m_, 6.0);

		f_func = &process_func;
		
		s_init.resize(6);
		s_meas.resize(6);

		// create VO MODEL
		voR = MatrixXf::Identity(6,6);
		voQ = MatrixXf::Identity(6,6);
		vo_meas.resize(3);


		/////////////////////////////////////////////
		// create VO&IMU MODEL
		voImuR = MatrixXf::Identity(6,6);
		voImuQ = MatrixXf::Identity(6,6);

		// measurement Std
		// T_et x,y,z
		voImuR(0,0) = 1.0; voImuR(1,1) = 1.0; voImuR(2,2) = 1.0;  
		// v_et
		voImuR(3,3) = 0.1; voImuR(4,4) = 0.1; voImuR(5,5) = 0.1;

		// process Std
		// T_et x,y,z
		voImuQ(0,0) = 10.0; voImuQ(1,1) = 10.0; voImuQ(2,2) = 2.0;  
		// v_et
		voImuQ(3,3) = 10.0; voImuQ(4,4) = 10.0; voImuQ(5,5) = 10.0;

		////////////////////////////////////////////////////
		// create IMU MODEL
		imuR = MatrixXf::Identity(6,6);
		imuQ = MatrixXf::Identity(6,6);
		imu_meas.resize(10);

		// measurement Std
		// T_et x,y,z
		imuR(0,0) = 99.0; imuR(1,1) = 99.0; imuR(2,2) = 99.0;  
		// v_et
		imuR(3,3) = 0.01; imuR(4,4) = 0.01; imuR(5,5) = 0.01;

		// process Std
		// T_et x,y,z
		imuQ(0,0) = 6.0; imuQ(1,1) = 6.0; imuQ(2,2) = 6.0;  
		// v_et
		imuQ(3,3) = 1.0; imuQ(4,4) = 1.0; imuQ(5,5) =1.0;

		/////////////////////////////////////////////
		// create LASER MODEL
		laserR = MatrixXf::Identity(6,6);
		laserQ = MatrixXf::Identity(6,6);
		laser_meas.resize(3);





		//////////////////////////////////////////
		// create LASER&IMU MODEL
		laserImuR = MatrixXf::Identity(6,6);
		laserImuQ = MatrixXf::Identity(6,6);

		// measurement Std
		// T_et x,y,z
		laserImuR(0,0) = 10.0; laserImuR(1,1) = 10.0; laserImuR(2,2) = 10.0;  
		// v_et
		laserImuR(3,3) = 10.0; laserImuR(4,4) = 10.0; laserImuR(5,5) = 10.0;

		// process Std
		// T_et x,y,z
		laserImuQ(0,0) = 10.0; laserImuQ(1,1) = 10.0; laserImuQ(2,2) = 10.0;  
		// v_et
		laserImuQ(3,3) = 10.0; laserImuQ(4,4) = 10.0; laserImuQ(5,5) = 10.0;
		//cout<< "here1" <<endl;
	};

	OdomEstimation::~OdomEstimation(){};

	void OdomEstimation::addMeasurement_vo(const VectorXf& meas_vo)
	{
		vo_meas = meas_vo;
	};

	void OdomEstimation::addMeasurement_imu(const VectorXf& meas_imu)
	{
		imu_meas = meas_imu;
	};

	void OdomEstimation::addMeasurement_laser(const VectorXf& meas_laser)
	{
		laser_meas = meas_laser;
	};

	bool OdomEstimation::preInitialize(const ros::Time& time)
	{
		if(!imu_initialized_ && !laser_initialized_)
		{
			// init
			filter_time_old_ = time;
			imu_initialized_ = true;
			laser_initialized_  = true;

			// save the initalized data
			imu_init_meas = imu_meas;
			laser_init_meas  = laser_meas;
			//cout<< "here20" <<endl;
			return false;
		}
		else
		{
			//cout<< "here21" <<endl;
			// here, got the init data and compute the S_pre and S_post 
			dt = (time - filter_time_old_ ).toSec();
			filter_time_old_ = time;

			T_et_init.resize(3);
			T_et_now.resize(3);
			//cout<< "here22" <<endl;
			getLaserXYZ(T_et_init, laser_init_meas, imu_init_meas);
			getLaserXYZ(T_et_now, laser_meas, imu_meas);
			//cout<< "here23" <<endl;
			//getCoVoImu(init_compose, vo_init_meas, imu_init_meas);
			//getCoVoImu(now_compose , vo_meas, imu_meas);

			float diff_x = T_et_now(0) - T_et_init(0);
			float diff_y = T_et_now(1) - T_et_init(1);
			float diff_z = T_et_now(2) - T_et_init(2);
				
			// init S x_tc, y_tc, z_tc
			s_init(0) = T_et_now(0);
			s_init(1) = T_et_now(0);
			s_init(2) = T_et_now(0);
			/*
			// init q_eb
			s_init(3) = imu_meas(0);
			s_init(4) = imu_meas(1);
			s_init(5) = imu_meas(2);
			s_init(6) = imu_meas(3);
			*/
			// inti v_et
			s_init(3) = diff_x / dt;
			s_init(4) = diff_y / dt;
			s_init(5) = diff_z / dt;
			/*
			// init w_tb or w_tc
			s_init(10) = imu_meas(7);
			s_init(11) = imu_meas(8);
			s_init(12) = imu_meas(9);
			*/
			//cout<< "here2" <<endl;
			return true;
		}
	};
	void OdomEstimation::initialize(const Time& time)
	{
		if(preInitialize(time) )
		{
			/*
			// init with Vo and IMU
			h_func = &measure_func_voimu;
			// init the ukf 
			ukf = new SRUKF(n_, m_, q_*q_, r_*r_, f_func, h_func); 
			
			ukf-> setR(voImuR);
			ukf-> setQ(voImuQ);

			ukf-> state_pre = s_init ;
			ukf-> state_post = s_init ;

			ukf->S_pre = MatrixXf::Identity(n_,m_);
			ukf->S_post = MatrixXf::Identity(n_,m_);

			filter_time_old_ = time;
			filter_data = ukf-> state_post;
			filter_initialized_ = true;

			// a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);
			*/
			h_func = &measure_func_laserimu;
			ukf = new SRUKF(n_, m_, q_*q_, r_*r_, f_func, h_func); 
			ukf-> setR(laserImuR);
			ukf-> setQ(laserImuQ);

			ukf-> state_pre = s_init ;
			ukf-> state_post = s_init ;

			ukf->S_pre = MatrixXf::Identity(n_,m_);
			ukf->S_post = MatrixXf::Identity(n_,m_);

			filter_time_old_ = time;
			filter_data = ukf-> state_post;
			filter_initialized_ = true;

			// a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);
			//cout<< "here3" <<endl;
		}
	};



	void OdomEstimation::getCoVoImu(VectorXf& filter_compose, const VectorXf& meas_vo, const VectorXf& meas_imu)
	{
		// rotation related variables
		matrix3f R_bc, R_bb_f;
		vector4f q_ce, q_cf, q_eb, q_ct, q_ec, q_fc, q_tilt_ce, q_torsion_ce, q_tilt_cf, q_torsion_cf, q_tc_new, q_tc_filter;
		vector4f q_tmp_torsion, q_tmp_tilt, q_tmp;
		matrix3f R_eb, R_ct, R_tc, R_ec, R_cf, R_fc, R_tc_new, R_tc_filter;
		matrix3f R_roll_ec, R_yaw_ec, R_roll_fc, R_yaw_fc, R_tmp;

		vector3f T_tc, T_ct, a_e, a_c, a_t, w_b, w_c, w_t, T_tc_new;
		vector3f T_tmp;
		vector3f old_T_tc;

		// init some rotation related variables 
		matrix3f_set_value(R_bc, 
							0, -1, 0,
                            1,  0, 0,
                            0,  0, 1);
		matrix3f_set_value(R_bb_f, 
						   -1,  0,  0,
                            0,  1,  0,
                            0,  0, -1);
		// here we have imu data and vision data available:
		// got q_eb, a_e, w_b, T_tc, q_ct 
		// Then get R_tc_new and T_tc_new

		//imu data 
		q_eb[0] = meas_imu(0);
		q_eb[1] = meas_imu(1);
		q_eb[2] = meas_imu(2);
		q_eb[3] = meas_imu(3);

		a_e[0]  = meas_imu(4);
		a_e[1]  = meas_imu(5);
		a_e[2]  = meas_imu(6);

		w_b[0] = meas_imu(7); 
		w_b[1] = meas_imu(8); 
		w_b[2] = meas_imu(9); 
		/*
		//debug
		cout << "w_b_x: "<< w_b[0]<<endl
		     << "w_b_y: "<< w_b[1]<<endl		
		     << "w_b_z: "<< w_b[2]<<endl<<endl;
		cout << "w_b_x: "<< w_b[0]/M_PI*180.0f<<endl
		     << "w_b_y: "<< w_b[1]/M_PI*180.0f<<endl		
		     << "w_b_z: "<< w_b[2]/M_PI*180.0f<<endl<<endl;
		*/
		//vo data
		q_ct[0] = meas_vo(0);
		q_ct[1] = meas_vo(1);
		q_ct[2] = meas_vo(2);
		q_ct[3] = meas_vo(3);

		T_tc[0] = meas_vo(4);
		T_tc[1] = meas_vo(5);
		T_tc[2] = meas_vo(6);

		//tilt and torsion compose
		quat_to_DCM(R_eb, q_eb);
		quat_to_DCM(R_ct, q_ct);

		T_tmp[0] = -T_tc[0];
		T_tmp[1] = -T_tc[1];
		T_tmp[2] = -T_tc[2];

		matrix3f_multi_vector3f(T_ct, R_ct, T_tmp); //T_ct = -R_ct*T_tc;

		matrix3f_multi_matrix3f(R_ec, R_eb, R_bc);   // R_ec = R_eb * R_bc

		get_matrix3f_transpose(R_tmp, R_ec);
		matrix3f_multi_vector3f(a_c, R_tmp, a_e); //a_c = R_ce * a_e;

		get_matrix3f_transpose(R_tc, R_ct);
		matrix3f_multi_matrix3f(R_fc, R_bb_f, R_tc); // R_fc = R_bb_f * R_tc

		DCM_to_quat(q_ec, R_ec);
		DCM_to_quat(q_fc, R_fc);

		get_quat_conjugate(q_ce, q_ec);
		get_quat_conjugate(q_cf, q_fc);

		quat_decomposition_tilt_n_torsion(q_tilt_ce, q_torsion_ce, q_ce);
		quat_decomposition_tilt_n_torsion(q_tilt_cf, q_torsion_cf, q_cf);

		get_quat_conjugate(q_tmp_torsion, q_torsion_cf);
		get_quat_conjugate(q_tmp_tilt, q_tilt_ce);
		unit_quat_multi(q_tmp, q_tmp_torsion, q_tmp_tilt);

		quat_to_DCM(R_tmp, q_tmp);
		matrix3f_multi_matrix3f(R_tc_new, R_bb_f, R_tmp);
		DCM_to_quat(q_tc_new, R_tc_new);

		get_matrix3f_transpose(R_tmp, R_bc);
		matrix3f_multi_vector3f(w_c, R_tmp, w_b); //w_c = R_cb*w_b;
		//////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		matrix3f_multi_vector3f(w_t, R_tc_new, w_c); //w_t = R_tc*w_c;

		T_tmp[0] = -T_ct[0]; 
		T_tmp[1] = -T_ct[1]; 
		T_tmp[2] = -T_ct[2];

		matrix3f_multi_vector3f(T_tc_new, R_tc_new, T_tmp); //T_tc_new = -R_tc_new*T_ct;
		matrix3f_multi_vector3f(a_t, R_tc_new, a_c); //a_t = R_tc* a_c;


		// here we get T_tc_new, q_tc_new , w_c, a_t;
		filter_compose(0) = T_tc_new[0];
		filter_compose(1) = T_tc_new[1];
		filter_compose(2) = T_tc_new[2];

		filter_compose(3) = q_tc_new[0];
		filter_compose(4) = q_tc_new[1];
		filter_compose(5) = q_tc_new[2];
		filter_compose(6) = q_tc_new[3];
		
		/*
		float yaw2,pitch2,roll2;
    		quat_to_eular(&yaw2, &pitch2, &roll2, q_tc_new);
		cout<< "yaw  : "<<yaw2 / M_PI * 180.0f<<endl
    		    << "pitch: "<<pitch2 / M_PI * 180.0f<<endl
    		    << "roll : "<<roll2 /  M_PI * 180.0f<<endl<<endl;
		*/	
		filter_compose(7) = w_c[0];
		filter_compose(8) = w_c[1];
		filter_compose(9) = w_c[2];

		filter_compose(10) = a_t[0];
		filter_compose(11) = a_t[1];
		filter_compose(12) = a_t[2];
		//std::cout<<"NOW \n"<<filter_compose.size()<<std::endl;
		//for (int i = 0 ; i < filter_compose.size() ; i++) std::cout<< filter_compose(i)<<std::endl;
	};

	void OdomEstimation::getXYZ(VectorXf& T_et, const VectorXf& uv1, const VectorXf& meas_imu, const float heigt)
	{
		vector4f q_eb;

		matrix3f R_cb, R_eb, R_be, R_cetmp;
		matrix3f_set_value(R_cb, 
				 0,  1, 0,
				-1,  0, 0,
				 0,  0, 1);

		q_eb[0] = meas_imu(0);
		q_eb[1] = meas_imu(1);
		q_eb[2] = meas_imu(2);
		q_eb[3] = meas_imu(3);

		quat_to_DCM(R_eb, q_eb);
		get_matrix3f_transpose(R_be, R_eb);

		matrix3f_multi_matrix3f(R_cetmp, R_cb, R_be);
		// opencv lib
		//R_cNEG
		Mat R_ce = (Mat_< float >(3,3)<<
					R_cetmp[0][0],R_cetmp[0][1],R_cetmp[0][2],
					R_cetmp[1][0],R_cetmp[1][1],R_cetmp[1][2],
					R_cetmp[2][0],R_cetmp[2][1],R_cetmp[2][2]);
	
		Mat XYZ = (Mat_< float >(3,1)<<	0.0f,
						0.0f,
						1.0f);

		Mat uv = (Mat_< float >(3,1)<<	uv1(0),
						uv1(1),
						uv1(2));
		Mat offset = (Mat_< float >(3,1)<<	0.0f,
							88.0f,
							-10.0f);
		Mat K=(Mat_<float>(3,3)<<
					180.2346315,0,159.5,
					0,180.2346315,119.5,
					0,0,1);

		XYZ = (K * R_ce).inv() * uv;
		Mat RoffSet = R_ce.inv() * offset;
		XYZ = ((heigt + RoffSet.at<float>(2,0))/ XYZ.at<float>(2,0)) * XYZ - RoffSet; 
		// end opencv lib 
		// mm to cm
		T_et(0) = XYZ.at<float>(0,0) / 10.0f;
		T_et(1) = XYZ.at<float>(1,0) / 10.0f;
		T_et(2) = XYZ.at<float>(2,0) / 10.0f;
	};

	void OdomEstimation::getLaserXYZ(VectorXf& T_et, const VectorXf& T_lt, const VectorXf& meas_imu)
	{
		//cout<<"here220"<<endl;
		vector4f q_eb;
		vector3f T_bt, T_bl, T_lttmp, T_ettmp;
		T_bl[0] = 10; //cm
		T_bl[1] = 0;
		T_bl[2] = -9; //cm
		T_lttmp[0] = T_lt(0);
		T_lttmp[1] = T_lt(1);
		T_lttmp[2] = T_lt(2);
		//cout<<"here221"<<endl;
		matrix3f R_eb, R_be;

		q_eb[0] = meas_imu(0);
		q_eb[1] = meas_imu(1);
		q_eb[2] = meas_imu(2);
		q_eb[3] = meas_imu(3);

		quat_to_DCM(R_eb, q_eb);
		//get_matrix3f_transpose(R_be, R_eb);

		vector3f_add(T_bt, T_lttmp, T_bl);//T_bt = T_lt + T_bl;
		matrix3f_multi_vector3f(T_ettmp, R_eb, T_bt);
		// opencv lib
		
		// end opencv lib 
		// mm to cm
		T_et(0) = T_ettmp[0];
		T_et(1) = T_ettmp[1];
		T_et(2) = T_ettmp[2];
	};

	bool OdomEstimation::update(bool imu_active, bool vo_active, bool laser_active, const ros::Time& filter_time)
	{
		dt = (filter_time - filter_time_old_ ).toSec();
		//ROS_INFO("dt is : %f", dt);
		filter_time_old_ = filter_time;
		// only get imu data
		//if(imu_active || vo_active) //test active
		/*
		if(imu_active && !vo_active)
		{
			// for this setp, x,y,z can be any value as the measurement fanction changed
			s_meas(0) = 0;
			s_meas(1) = 0;
			s_meas(2) = 0;
			// v_et !!!!!! velocoties update upon the imu data
			s_meas(3) = filter_data(3) - a_eb[0]* dt;
			s_meas(4) = filter_data(4) - a_eb[0]* dt;
			s_meas(5) = filter_data(5) - a_eb[0]* dt;
	
			ukf->predict(a_eb, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_imu);
				// change the measurement noise covariance
				ukf-> setR(imuR);
				ukf-> setQ(imuQ);
				// correct
				ukf->correct(s_meas);
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;

			// update a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);

			ROS_DEBUG("Imu  sensor active!");
			//std::cout<<"Imu  sensor  active!"<<std::endl;


			return true;
		}
		*/
		if(imu_active && !laser_active)
		{
			// for this setp, x,y,z can be any value as the measurement fanction changed
			s_meas(0) = 0;
			s_meas(1) = 0;
			s_meas(2) = 0;
			// v_et !!!!!! velocoties update upon the imu data
			s_meas(3) = filter_data(3) - a_eb[0]* dt;
			s_meas(4) = filter_data(4) - a_eb[0]* dt;
			s_meas(5) = filter_data(5) - a_eb[0]* dt;
	
			ukf->predict(a_eb, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_imu);
				// change the measurement noise covariance
				ukf-> setR(imuR);
				ukf-> setQ(imuQ);
				// correct
				ukf->correct(s_meas);
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;

			// update a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);

			ROS_DEBUG("Imu  sensor active!");
			std::cout<<"Imu  sensor  active!"<<std::endl;


			return true;
		}
		// both imu and vo
		if(imu_active && vo_active)
		{
			getXYZ(T_et_now, vo_meas, imu_meas,  FIXED_HEIGHT);

			float diff_x = T_et_now(0) - filter_data(0);
			float diff_y = T_et_now(1) - filter_data(1);
			float diff_z = T_et_now(2) - filter_data(2);

			//  S x_et, y_et, z_et
			s_meas(0) = T_et_now(0);
			s_meas(1) = T_et_now(1);
			s_meas(2) = T_et_now(2);

			// ! v_et  = (Differential + Integral) /2
			s_meas(3) = (diff_x/dt + filter_data(3) - a_eb[0]*dt) / 2;
			s_meas(4) = (diff_y/dt + filter_data(4) - a_eb[1]*dt) / 2;
			s_meas(5) = (diff_z/dt + filter_data(5) - a_eb[2]*dt) / 2;

			ukf->predict(a_eb, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_voimu);

				// change the measurement noise covariance

				ukf-> setR(voImuR);
				ukf-> setQ(voImuQ);

				// correct
				ukf->correct(s_meas);
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;
			// update a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);

			ROS_DEBUG("Both sensors active!");
			std::cout<<"Both sensors active!"<<std::endl;
			return true;
		}

		// only get vo data
		//if(vo_active || imu_active)
		if(vo_active && !imu_active)
		{
			getXYZ(T_et_now, vo_meas, imu_meas,  FIXED_HEIGHT);

			float diff_x = T_et_now(0) - filter_data(0);
			float diff_y = T_et_now(1) - filter_data(1);
			float diff_z = T_et_now(2) - filter_data(2);

			//  S x_et, y_et, z_et
			s_meas(0) = T_et_now(0);
			s_meas(1) = T_et_now(1);
			s_meas(2) = T_et_now(2);

			// ! v_et  = (Differential + Integral) /2
			s_meas(3) = diff_x/dt;
			s_meas(4) = diff_y/dt;
			s_meas(5) = diff_z/dt;

			ukf->predict(a_eb, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_vo);

				// change the measurement noise covariance

				ukf-> setR(voR);
				ukf-> setQ(voQ);

				// correct
				ukf->correct(s_meas);
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;
			// update a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);

			ROS_DEBUG("Vo   sensor  active!");
			std::cout<<"Vo   sensor  active!"<<std::endl;
			return true;
		}
		// both imu and vo
		if(imu_active && laser_active)
		{
			getLaserXYZ(T_et_now, laser_meas, imu_meas);

			float diff_x = T_et_now(0) - filter_data(0);
			float diff_y = T_et_now(1) - filter_data(1);
			float diff_z = T_et_now(2) - filter_data(2);

			//  S x_et, y_et, z_et
			s_meas(0) = T_et_now(0);
			s_meas(1) = T_et_now(1);
			s_meas(2) = T_et_now(2);

			// ! v_et  = (Differential + Integral) /2
			s_meas(3) = (diff_x/dt + filter_data(3) - a_eb[0]*dt) / 2.0f;
			s_meas(4) = (diff_y/dt + filter_data(4) - a_eb[1]*dt) / 2.0f;
			s_meas(5) = (diff_z/dt + filter_data(5) - a_eb[2]*dt) / 2.0f;

			ukf->predict(a_eb, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_laserimu);

				// change the measurement noise covariance

				ukf-> setR(laserImuR);
				ukf-> setQ(laserImuQ);

				// correct
				ukf->correct(s_meas);
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;
			// update a_eb
			a_eb[0] = imu_meas(4);
			a_eb[1] = imu_meas(5);
			a_eb[2] = imu_meas(6);

			ROS_DEBUG("Both sensors active!");
			std::cout<<"Both sensors active!"<<std::endl;
			return true;
		}

		std::cout<<"No sensor active!"<<std::endl;
		return false;
	};

	bool OdomEstimation::getEstimate(VectorXf& estimate)
	{
		if(isValidFdata())
		{
			
			VectorXf all_data;
			all_data.resize(17);
			// T_et
			all_data (0)  = filter_data(0);		
			all_data (1)  = filter_data(1);	
			all_data (2)  = filter_data(2);	
			// q_eb
			all_data (3)  = imu_meas(0);		
			all_data (4)  = imu_meas(1);
			all_data (5)  = imu_meas(2);
			all_data (6)  = imu_meas(3);
			// v_et
			all_data (7)  = filter_data(3);		
			all_data (8)  = filter_data(4);	
			all_data (9)  = filter_data(5);	
			// w_bb
			all_data (10) = imu_meas(7);		
			all_data (11) = imu_meas(8);
			all_data (12) = imu_meas(9);
			// a_eb
			all_data (13) = a_eb[0];
			all_data (14) = a_eb[1];
			all_data (15) = a_eb[2];
			// dt
			all_data (16) = dt;
			
			estimate = all_data;
			
			return true;
		}
		else
			return false;
	};

	bool OdomEstimation::isValidFdata()
	{
		int invalid = 0;
		int n = ukf -> state_post.size();
		for (int i = 0; i < n; i++)
			if (ukf -> state_post(i) != ukf -> state_post(i))
				invalid ++;
		for (int i = 0; i < n; i++)
			if (ukf -> state_post(i) > 2000.0)
				invalid ++;
		if (invalid >= 2)
		{
			filter_initialized_ = false;
			imu_initialized_    = false;
			vo_initialized_     = false;

			ROS_ERROR("InValid data! restart UKF!!!");

			return false; 
						
			
		}
		//std::cout<<"NOW \n"<< invalid <<std::endl;
		//for (int i = 0 ; i < ukf -> state_post.size() ; i++) std::cout<< ukf -> state_post(i)<<std::endl;
		return true;
	};

};
