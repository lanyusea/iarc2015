/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	odom_estimation.cpp
* Brief: odom estimation for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/6/16 14:00
* History: modified from ROS ekf http://wiki.ros.org/robot_pose_ekf
************************************************************************/

#include "odom_estimation.h"

namespace estimation
{
	void process_func(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt)
	{
		vector3f r;
		vector4f q;
		vector4f dq;
		vector4f tmp_q;

		vector3f v;
		vector3f w;
		r[0] = x_t_1(0); r[1] = x_t_1(1); r[2] = x_t_1(2);

		q[0] = x_t_1(3);
		q[1] = x_t_1(4);
		q[2] = x_t_1(5);
		q[3] = x_t_1(6);

		v[0] = x_t_1(7); v[1] = x_t_1(8); v[2] = x_t_1(9);     // previous speed
		w[0] = x_t_1(10); w[1] = x_t_1(11); w[2] = x_t_1(12);  // previous angular speed

		r[0] += v[0]*dt;
		r[1] += v[1]*dt;
		r[2] += v[2]*dt;

		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// I change the w_c to w_t
		SORA_to_quat(dq, w[0]*dt, w[1]*dt, w[2]*dt);
		unit_quat_multi(tmp_q, q, dq);

		x_t(0) = r[0]; x_t(1) = r[1]; x_t(2) = r[2];

		x_t(3) = tmp_q[0];
		x_t(4) = tmp_q[1];
		x_t(5) = tmp_q[2];
		x_t(6) = tmp_q[3];
    
		x_t(7) = v[0] + a[0]*dt; 
		x_t(8) = v[1] + a[1]*dt; 
		x_t(9) = v[2] + a[2]*dt;

		x_t(10) = w[0]; x_t(11) = w[1]; x_t(12) = w[2];
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
		for (int i = 0; i < 10; i++)
			z_t(i) = x_t(i);
		z_t(10) = 0;
		z_t(11) = 0;
		z_t(12) = 0;
	};

	OdomEstimation::OdomEstimation():
		filter_initialized_(false),
		imu_initialized_(false),
		vo_initialized_(false)
	{
		ros::NodeHandle nh_private("~");

		// create SYSTEM MODEL
		nh_private.param("q", q_, 0.01);
		nh_private.param("r", r_, 2.0);
		nh_private.param("n", n_, 13.0);
		nh_private.param("m", m_, 13.0);

		f_func = &process_func;
		
		s_init.resize(13);
		s_meas.resize(13);

		// create VO MODEL
		voR = MatrixXf::Identity(13,13);
		voQ = MatrixXf::Identity(13,13);
		vo_meas.resize(7);

		// create VO&IMU MODEL
		voImuR = MatrixXf::Identity(13,13);
		voImuQ = MatrixXf::Identity(13,13);
		voImu_meas.resize(13);


		// measurement Std
		// T_tc x,y,z
		voImuR(0,0) = 6.0; voImuR(1,1) = 6.0; voImuR(2,2) = 6.0;  
		// q_tc
		voImuR(3,3) = 0.01; voImuR(4,4) = 0.01; voImuR(5,5) = 0.01; voImuR(6,6) = 0.01; 
		// v_tc 
		voImuR(7,7) = 10.0; voImuR(8,8) = 10.0; voImuR(9,9) = 10.0;
		// w_tc
		voImuR(10,10) = 0.009; voImuR(11,11) = 0.009; voImuR(12,12) = 0.009;

		// process Std
		// T_tc x,y,z
		voImuQ(0,0) = 5.0; voImuQ(1,1) = 5.0; voImuQ(2,2) = 5.0;  
		// q_tc
		voImuQ(3,3) = 1.0; voImuQ(4,4) = 1.0; voImuQ(5,5) = 1.0; voImuR(6,6) = 1.0; 
		// v_tc 
		voImuQ(7,7) = 1.0; voImuQ(8,8) = 1.0; voImuQ(9,9) = 1.0;
		// w_tc
		voImuQ(10,10) = 0.009; voImuQ(11,11) = 0.009; voImuQ(12,12) = 0.009;

		// create IMU MODEL
		imuR = MatrixXf::Identity(13,13);
		imuQ = MatrixXf::Identity(13,13);
		imu_meas.resize(9);

		// measurement Std
		// T_tc x,y,z
		imuR(0,0) = 9999.0; imuR(1,1) = 9999.0; imuR(2,2) = 9999.0;  
		// q_tc
		imuR(3,3) = 0.01; imuR(4,4) = 0.01; imuR(5,5) = 0.01; imuR(6,6) = 0.01; 
		// v_tc 
		imuR(7,7) = 10.0; imuR(8,8) = 10.0; imuR(9,9) = 10.0;
		// w_tc
		imuR(10,10) = 0.009; imuR(11,11) = 0.009; imuR(12,12) = 0.009;	

		// process Std
		// T_tc x,y,z
		imuQ(0,0) = 6.0; imuQ(1,1) = 6.0; imuQ(2,2) = 6.0;  
		// q_tc
		imuQ(3,3) = 1.0; imuQ(4,4) = 1.0; imuQ(5,5) =1.0; imuQ(6,6) = 1.0; 
		// v_tc 
		imuQ(7,7) = 1.0; imuQ(8,8) = 1.0; imuQ(9,9) = 1.0;
		// w_tc
		imuQ(10,10) = 0.009; imuQ(11,11) = 0.009; imuQ(12,12) = 0.009;	

	};

	OdomEstimation::~OdomEstimation(){};

	void OdomEstimation::addMeasurement_vo(const VectorXf& meas_vo)
	{
		vo_meas = meas_vo;
	};

	void OdomEstimation::addMeasurement_imu(const VectorXf& meas_imu)
	{
		
		a_ec_now[0] = meas_imu(4);
		a_ec_now[1] = meas_imu(5);
		a_ec_now[2] = meas_imu(6);
		
		a_ec_old[0] = imu_meas(4);
		a_ec_old[1] = imu_meas(5);
		a_ec_old[2] = imu_meas(6);

		imu_meas = meas_imu;
	};

	void OdomEstimation::initialize(const Time& time)
	{
		if(preInitialize(time) )
		{
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
		}
	};

	bool OdomEstimation::preInitialize(const ros::Time& time)
	{
		if(!imu_initialized_ && !vo_initialized_)
		{
			// init
			filter_time_old_ = time;
			imu_initialized_ = true;
			vo_initialized_  = true;

			// save the initalized data
			imu_init_meas = imu_meas;
			vo_init_meas  = vo_meas;

			return false;
		}
		else
		{
			// here, got the init data and compute the S_pre and S_post 
			dt = (time - filter_time_old_ ).toSec();
			filter_time_old_ = time;
			init_compose.resize(13);
			now_compose.resize(13);


			getCoVoImu(init_compose, vo_init_meas, imu_init_meas);
			getCoVoImu(now_compose , vo_meas, imu_meas);

			float diff_x = now_compose(0) - init_compose(0);
			float diff_y = now_compose(1) - init_compose(1);
			float diff_z = now_compose(2) - init_compose(2);
				
			// init S x_tc, y_tc, z_tc
			s_init(0) = now_compose(0);
			s_init(1) = now_compose(1);
			s_init(2) = now_compose(2);

			// init q_tc
			s_init(3) = now_compose(3);
			s_init(4) = now_compose(4);
			s_init(5) = now_compose(5);
			s_init(6) = now_compose(6);

			// inti v_tc
			s_init(7) = diff_x / dt;
			s_init(8) = diff_y / dt;
			s_init(9) = diff_z / dt;

			// init w_tb or w_tc
			s_init(10) = now_compose(7);
			s_init(11) = now_compose(8);
			s_init(12) = now_compose(9);

			return true;
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

	bool OdomEstimation::update(bool imu_active, bool vo_active, const ros::Time& filter_time)
	{
		dt = (filter_time - filter_time_old_ ).toSec();
		//ROS_INFO("dt is : %f", dt);
		filter_time_old_ = filter_time;
		// only get imu data
		//if(imu_active || vo_active) //test active
		if(imu_active && !vo_active)
		{
			// if lose track, only measure speed 
			// last step q_tc
			//float q_tc_filter[4];
			//q_tc_filter[0] = filter_data(3);
			//q_tc_filter[1] = filter_data(4);
			//q_tc_filter[2] = filter_data(5);
			//q_tc_filter[3] = filter_data(6);
			

			
			//debug
			vector4f q_debug;
			q_debug[0] = filter_data(3);
			q_debug[1] = filter_data(4);
			q_debug[2] = filter_data(5);
			q_debug[3] = filter_data(6);
			
			float yaw,pitch,roll;
			quat_to_eular(&yaw, &pitch, &roll, q_debug);
			cout<< "yaw  : "<<yaw / M_PI * 180.0f<<endl
			    << "pitch: "<<pitch / M_PI * 180.0f<<endl
			    << "roll : "<<roll / M_PI * 180.0f<<endl<<endl;
			/*
			//debug
			cout<< "w_t_x_f: "<<filter_data(10) / M_PI * 180.0f<<endl
			    << "w_t_y_f: "<<filter_data(11) / M_PI * 180.0f<<endl
			    << "w_t_z_f: "<<filter_data(12) / M_PI * 180.0f<<endl<<endl;
			*/

			vector4f q_tc_new, q_ct_new, q_ct_old, q_tc_old, dqtmp;
			q_ct_old[0] = vo_meas(0);
			q_ct_old[1] = vo_meas(1);
			q_ct_old[2] = vo_meas(2);
			q_ct_old[3] = vo_meas(3);

						
			get_quat_conjugate(q_tc_old, q_ct_old);
			
			SORA_to_quat(dqtmp, filter_data(10)*dt, filter_data(11)*dt, filter_data(12)*dt);
			unit_quat_multi(q_tc_new, q_tc_old, dqtmp);
			
			get_quat_conjugate(q_ct_new, q_tc_new);
			/*
			//debug
			cout<< "dq.x " << filter_data(10)*dt 
			    << "\ndq.y " << filter_data(11)*dt
			    << "\ndq.z " << filter_data(12)*dt<<endl
			    << "q_ct_new[0] "<< q_ct_new[0]
			    << "\nq_ct_new[1] "<< q_ct_new[1]
			    << "\nq_ct_new[2] "<< q_ct_new[2]
			    << "\nq_ct_new[3] "<< q_ct_new[3]<<endl
			    << "q_ct_old[0] "<< vo_meas(0)
			    << "\nq_ct_old[1] "<< vo_meas(1)
			    << "\nq_ct_old[2] "<< vo_meas(2)
			    << "\nq_ct_old[3] "<< vo_meas(3)<<endl;
			*/
			vo_meas(0) = q_ct_new[0];
			vo_meas(1) = q_ct_new[1];
			vo_meas(2) = q_ct_new[2];
			vo_meas(3) = q_ct_new[3];
			/*
			//debug
			q_debug[0] = vo_meas(0);
			q_debug[1] = vo_meas(1);
			q_debug[2] = vo_meas(2);
			q_debug[3] = vo_meas(3);
			
			quat_to_eular(&yaw, &pitch, &roll, q_debug);
			cout<< "yaw2 : "<<yaw / M_PI * 180.0f<<endl
			    << "pitc2: "<<pitch / M_PI * 180.0f<<endl
			    << "roll2: "<<roll / M_PI * 180.0f<<endl<<endl;
			*/
			// ! the vo_meas is not update in this step
			imu_meas(4) = a_ec_old[0];
			imu_meas(5) = a_ec_old[1];
			imu_meas(6) = a_ec_old[2];

			getCoVoImu(now_compose , vo_meas, imu_meas);
		
			// a_tc
			a_tc_old[0] = now_compose(10);
			a_tc_old[1] = now_compose(11);
			a_tc_old[2] = now_compose(12);
			
			//std::cout<<"a_tc: \n"<<std::endl;
			//for(int i = 0 ; i < 3 ; i ++)
				//std::cout<<a_tc[i]<<std::endl;
			// S x_tc, y_tc, z_tc 
			// for this setp, x,y,z can be any value as the measurement fanction changed
			s_meas(0) = 0;
			s_meas(1) = 0;
			s_meas(2) = 0;
			
			//  q_tc
			s_meas(3) = now_compose(3);
			s_meas(4) = now_compose(4);
			s_meas(5) = now_compose(5);
			s_meas(6) = now_compose(6);

			/*
			//debug
			vector4f q_debug;
			q_debug[0] = now_compose(3);
			q_debug[1] = now_compose(4);
			q_debug[2] = now_compose(5);
			q_debug[3] = now_compose(6);
			
			float yaw,pitch,roll;
			quat_to_eular(&yaw, &pitch, &roll, q_debug);
			cout<< "yaw  : "<<yaw / M_PI * 180.0f<<endl
			    << "pitch: "<<pitch / M_PI * 180.0f<<endl
			    << "roll : "<<roll / M_PI * 180.0f<<endl<<endl;
			*/
			// v_tc !!!!!! velocoties update upon the imu data
			s_meas(7) = filter_data(7) + a_tc_old[0]*dt;
			s_meas(8) = filter_data(8) + a_tc_old[1]*dt;
			s_meas(9) = filter_data(9) + a_tc_old[2]*dt;

			// w_tb or w_tc
			s_meas(10) = now_compose(7);
			s_meas(11) = now_compose(8);
			s_meas(12) = now_compose(9);
		

			/*
			cout<< "w_t_x  : "<<s_meas(10) / M_PI * 180.0f<<endl
			    << "w_t_y  : "<<s_meas(11) / M_PI * 180.0f<<endl
			    << "w_t_z  : "<<s_meas(12) / M_PI * 180.0f<<endl<<endl;
			*/

			ukf->predict(a_tc_old, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				float q_tmp[4];

				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_imu);
				// change the measurement noise covariance

				ukf-> setR(imuR);
				ukf-> setQ(imuQ);

				// correct
				ukf->correct(s_meas);
				q_tmp[0] = ukf->state_post(3);
				q_tmp[1] = ukf->state_post(4);
				q_tmp[2] = ukf->state_post(5);
				q_tmp[3] = ukf->state_post(6);
				quat_normalize(q_tmp); 
				ukf->state_post(3) = q_tmp[0];
				ukf->state_post(4) = q_tmp[1];
				ukf->state_post(5) = q_tmp[2];
				ukf->state_post(6) = q_tmp[3];
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;
			/*
			//debug
			cout<< "w_t_x_f: "<<filter_data(10) / M_PI * 180.0f<<endl
			    << "w_t_y_f: "<<filter_data(11) / M_PI * 180.0f<<endl
			    << "w_t_z_f: "<<filter_data(12) / M_PI * 180.0f<<endl<<endl;
			*/	
			// vo rotation update
			// vo q_ct w x y z
			
			
			q_tc_new[0] = filter_data(3);
			q_tc_new[1] = filter_data(4);			
			q_tc_new[2] = filter_data(5);
			q_tc_new[3] = filter_data(6);

			get_quat_conjugate(q_ct_new, q_tc_new);

			vo_meas(0) = q_ct_new[0];
			vo_meas(1) = q_ct_new[1];
			vo_meas(2) = q_ct_new[2];
			vo_meas(3) = q_ct_new[3];
			
			//cout<< q_tc_new[0] <<","<<-q_tc_new[1]<<","<<-q_tc_new[2]<<","<<-q_tc_new[3]<<"\n"
			//<<q_ct_new[0]<<","<<q_ct_new[1]<<","<<q_ct_new[2]<<","<<q_ct_new[3]<<"\n";

			ROS_DEBUG("Imu  sensor active!");
			std::cout<<"Imu  sensor  active!"<<std::endl;
			return true;
		}

		// both imu and vo
		if(imu_active && vo_active)
		{

			
			//debug
			vector4f q_debug;
			q_debug[0] = filter_data(3);
			q_debug[1] = filter_data(4);
			q_debug[2] = filter_data(5);
			q_debug[3] = filter_data(6);
			
			float yaw,pitch,roll;
			quat_to_eular(&yaw, &pitch, &roll, q_debug);
			cout<< "yaw  : "<<yaw / M_PI * 180.0f<<endl
			    << "pitch: "<<pitch / M_PI * 180.0f<<endl
			    << "roll : "<<roll / M_PI * 180.0f<<endl<<endl;


			imu_meas(4) = a_ec_old[0];
			imu_meas(5) = a_ec_old[1];
			imu_meas(6) = a_ec_old[2];
			getCoVoImu(now_compose , vo_meas, imu_meas);

			float diff_x = now_compose(0) - filter_data(0);
			float diff_y = now_compose(1) - filter_data(1);
			float diff_z = now_compose(2) - filter_data(2);

			// a_tc
			a_tc_old[0] = now_compose(10);
			a_tc_old[1] = now_compose(11);
			a_tc_old[2] = now_compose(12);

			//  S x_tc, y_tc, z_tc
			s_meas(0) = now_compose(0);
			s_meas(1) = now_compose(1);
			s_meas(2) = now_compose(2);

			//  q_tc
			s_meas(3) = now_compose(3);
			s_meas(4) = now_compose(4);
			s_meas(5) = now_compose(5);
			s_meas(6) = now_compose(6);

			// v_tc
			//s_meas(7) = diff_x / dt;
			//s_meas(8) = diff_y / dt;
			//s_meas(9) = diff_z / dt;

			// ! v_tc  = (Differential + Integral) /2
			s_meas(7) = (diff_x/dt + filter_data(7) + a_tc_old[0]*dt) / 2;
			s_meas(8) = (diff_y/dt + filter_data(8) + a_tc_old[1]*dt) / 2;
			s_meas(9) = (diff_z/dt + filter_data(9) + a_tc_old[2]*dt) / 2;

			// w_tb or w_tc
			s_meas(10) = now_compose(7);
			s_meas(11) = now_compose(8);
			s_meas(12) = now_compose(9);


			ukf->predict(a_tc_old, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				float q_tmp[4];

				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_voimu);

				// change the measurement noise covariance

				ukf-> setR(voImuR);
				ukf-> setQ(voImuQ);

				// correct
				ukf->correct(s_meas);
				q_tmp[0] = ukf->state_post(3);
				q_tmp[1] = ukf->state_post(4);
				q_tmp[2] = ukf->state_post(5);
				q_tmp[3] = ukf->state_post(6);
				quat_normalize(q_tmp); 
				ukf->state_post(3) = q_tmp[0];
				ukf->state_post(4) = q_tmp[1];
				ukf->state_post(5) = q_tmp[2];
				ukf->state_post(6) = q_tmp[3];
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;

			ROS_DEBUG("Both sensors active!");
			std::cout<<"Both sensors active!"<<std::endl;
			return true;
		}

		// only get vo data
		//if(vo_active || imu_active)
		if(vo_active && !imu_active)
		{
			//  ! the imu_meas is not update in this step

			imu_meas(4) = a_ec_old[0];
			imu_meas(5) = a_ec_old[1];
			imu_meas(6) = a_ec_old[2];
			
			getCoVoImu(now_compose , vo_meas, imu_meas);

			float diff_x = now_compose(0) - filter_data(0);
			float diff_y = now_compose(1) - filter_data(1);
			float diff_z = now_compose(2) - filter_data(2);

			// a_tc
			a_tc_old[0] = now_compose(10);
			a_tc_old[1] = now_compose(11);
			a_tc_old[2] = now_compose(12);

			//  S x_tc, y_tc, z_tc
			s_meas(0) = now_compose(0);
			s_meas(1) = now_compose(1);
			s_meas(2) = now_compose(2);

			//  q_tc
			s_meas(3) = now_compose(3);
			s_meas(4) = now_compose(4);
			s_meas(5) = now_compose(5);
			s_meas(6) = now_compose(6);

			// ! v_tc  = (Differential + Integral) /2
			s_meas(7) = diff_x/dt ;
			s_meas(8) = diff_y/dt ;
			s_meas(9) = diff_z/dt ;

			// w_tb or w_tc
			s_meas(10) = 0;
			s_meas(11) = 0;
			s_meas(12) = 0;


			ukf->predict(a_tc_old, dt);

			bool is_measure_valid = true;
			for (int i = 0; i< s_meas.size(); i++)
				if (s_meas(i) != s_meas(i))
			is_measure_valid = false;

			if (is_measure_valid)
			{
				float q_tmp[4];

				// change the measurement h_func
				ukf->setMeasurementFunc(measure_func_vo);
				// change the measurement noise covariance

				// ukf->setR(voR);

				// correct
				ukf->correct(s_meas);
				q_tmp[0] = ukf->state_post(3);
				q_tmp[1] = ukf->state_post(4);
				q_tmp[2] = ukf->state_post(5);
				q_tmp[3] = ukf->state_post(6);
				quat_normalize(q_tmp); 
				ukf->state_post(3) = q_tmp[0];
				ukf->state_post(4) = q_tmp[1];
				ukf->state_post(5) = q_tmp[2];
				ukf->state_post(6) = q_tmp[3];
			}
			else
			{
				ukf->state_post << ukf->state_pre;
				ukf->S_post << ukf->S_pre;
				ROS_ERROR("measure_invalid!");
			}

			// update 
			filter_data = ukf->state_post;

			ROS_DEBUG("Vo   sensor  active!");
			std::cout<<"Vo   sensor  active!"<<std::endl;
			return true;
		}
		//std::cout<<"No sensor active!"<<std::endl;
		return false;
	};

	bool OdomEstimation::getEstimate(VectorXf& estimate)
	{
		if(isValidFdata())
		{
			
			VectorXf all_data;
			all_data.resize(17);
			for (int i = 0; i < 13; i++)
				all_data(i) = filter_data(i);			
			all_data (13) = a_tc_old[0];
			all_data (14) = a_tc_old[1];
			all_data (15) = a_tc_old[2];
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
			if (ukf -> state_post(i) > 20000.0)
				invalid ++;
		if (invalid >= 3)
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
