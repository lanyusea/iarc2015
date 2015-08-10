#include "OpenRadar.h"
OpenRadar::OpenRadar(void)
{
	receive_state = 0;
	timeInit = true;
	// display related variables
	DisplayDx = RadarImageWdith/x_ratio;
	DisplayDy = RadarImageHeight/y_ratio;

	RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,3);
	cvNamedWindow("Radar",1);

	pub=n2.advertise<ukftest::ukfData>("laser_ukf",100);
	pub_xy = n2.advertise<ukftest::laserPoint>("laser_point",100);
	H = new float[polarH_length];
	H_c = new int[polarH_length-2*skip_bin_idx];
	H_c2 = new int[polarH_length-2*skip_bin_idx];

}
OpenRadar::~OpenRadar(void)
{
	cvReleaseImage(&RadarImage);
	delete[] H;
	delete[] H_c;
	delete[] H_c2;
}

float OpenRadar::denoise_across_frame(int idx)
{
	float out = 0; 
	int out_cnt;
	float mean = 0;
	variance = 0;
	//calculate mean
	out_cnt = 0;
	for (int i = 0; i< MAX_frame; i++)
	{
		if (ranges[i][idx] < range_max)
		{
			range_tmp[i] = ranges[i][idx];
			mean += range_tmp[i];
			out_cnt+= 1;
		}
		else
			continue;

	}
	if (out_cnt == 0)
		return range_max*5;
	mean /= out_cnt;
	//calculate variance
	for (int i = 0; i< MAX_frame; i++)
		if (range_tmp[i] < range_max)
			variance += (mean-range_tmp[i])*(mean-range_tmp[i]);
		else
			variance += (mean-range_max)*(mean-range_max);
	variance /= MAX_frame;

	if (variance<500)
		return mean;
	else
		return std::numeric_limits<float>::quiet_NaN();
}

void OpenRadar::laser_proc()
{
	int start_step = 0;
	int end_step = num_data - 2;
	float dis;
	

	// first apply a 3*3 box filter to ranges data
	//for (int i = 1; i < MAX_frame - 1; i++)
	//	for (int j = 1; j < end_step - start_step; j++)
	//		ranges[i][j] = (ranges[i+1][j]+ranges[i][j]+ranges[i-1][j])*0.33333333333; 
	//for (int i = 1; i < MAX_frame - 1; i++)
	//	for (int j = 1; j < end_step - start_step; j++)
	//		ranges[i][j] = (ranges[i][j+1]+ranges[i][j]+ranges[i][j-1])*0.33333333333; 
    	
	//Inilize RadarTheta
	for(int i = 0; i < end_step ;i++)
	{
    		RadarTheta[i] = i*angle_increment + angle_min;
		//from noisy laser measurement to a relatively stable mesurement
		dis = denoise_across_frame(i);
		//cout << i<<"," << dis <<"|";
		/*
		if (dis - 300 > range_min)
			//RadarRho[i] = dis - 300; //add a safe distance 30cm
			RadarRho[i] = dis; 
		else
			RadarRho[i] = dis;
		//if (RadarRho[i] > 2500) RadarRho[i] = -1;
		*/
		if(dis > range_max) RadarRho[i] = range_max; 
		if(dis < range_min) RadarRho[i] = range_min; 
		if(dis >= range_min && dis<= range_max) RadarRho[i] = dis;
	}

	RadarDataCnt = num_data;

	VFH();	
	//ROS_INFO("calculated scale: %f", v_scale);
	//ROS_INFO("calculated angle: %f", v_angle/M_PI*180);

    	RadarBreak();
	DrawRadarData();	
	cvShowImage("Radar",RadarImage);
	if (cvWaitKey(3) == 27)
		ros::shutdown();
}

void OpenRadar::VFH()
{
	v_scale = 0;
	v_angle = 0;
	int sector = 0;
	int min_H, max_H;

	// initialize the histogram bin
	for (int i=0; i<polarH_length;i++)
		H[i] = 0;
	
	// put range data into Histogram bins
	//cout<<RadarDataCnt<<" "<<polarH_length<<endl;
	for (int i = 0; i < RadarDataCnt; i++)
	{
		if (RadarRho[i] != RadarRho[i])
			continue;
		else
		{
			if (RadarRho[i] <= range_max)
				H[sector] += constA - constB*RadarRho[i];
			else
				H[sector] += constA - constB*range_max;
				
		}
		if (i >	(sector+1)*RadarDataCnt/polarH_length)
			sector++;
	}
	//smooth Histogram bins
	for (int i=(sw_size-1)/2; i<polarH_length-(sw_size-1)/2; i++)
	{
		float tmp = 0;
		for (int ii=i-(sw_size-1)/2;ii<=i+(sw_size-1)/2;ii++)
			tmp += H[ii];
		H[i] = tmp/sw_size;
	}

	// debug
	/*
	cout<< "polarH_length:"<<polarH_length<<endl;
	for(int i = 0 ; i < polarH_length ; i ++)
	{
		cout<<"H"<< i << ":" <<H[i] <<endl;	
	}
	*/
	// find small H has direction
	//copy
	for (int i=skip_bin_idx; i<polarH_length-skip_bin_idx;i++)
		H_c[i-skip_bin_idx] = H[i];
	sort(H_c,H_c+polarH_length-2*skip_bin_idx);
	
	
	//debug
	/* 
	for (int i= skip_bin_idx; i<polarH_length-skip_bin_idx; i++)
	{
		cout<<"H_c_sort"<< i-skip_bin_idx << ":" <<H_c[i-skip_bin_idx] <<endl;	
	}
	*/
	//select best direction
	int s_idx, b_idx;
	min_H = H_c[0];
	max_H = H_c[polarH_length-2*skip_bin_idx-1];
	//get the index of largest bin
	for (b_idx=skip_bin_idx; b_idx<polarH_length-2*skip_bin_idx;b_idx++)
	{
		if ((H[b_idx] - max_H) >=0)
			break;
	}

	//debug
	//cout<<"Max: "<<b_idx<<","<<H[b_idx]<<","<<max_H<<endl;

	//rewrite H_c to be angle between this bin and max H bin 
	for (int i=skip_bin_idx; i<polarH_length-skip_bin_idx;i++)
		H_c[i-skip_bin_idx] = constA - 0.5*abs(i - b_idx) + abs(H[i]-min_H);
		//H_c[i-skip_bin_idx] = abs(H[i]-min_H);
	/*
	//debug 
	for (int i= skip_bin_idx; i<polarH_length-skip_bin_idx; i++)
	{
		cout<<"H_c_angle"<< i-skip_bin_idx << ":" <<H_c[i-skip_bin_idx] <<endl;	
	}
	*/
	//
	for (int i=0; i<polarH_length-2*skip_bin_idx;i++)
		H_c2[i] = H_c[i];
	sort(H_c2, H_c2+polarH_length-2*skip_bin_idx-1);

	for (s_idx=0; s_idx<polarH_length-2*skip_bin_idx;s_idx++)
	{
		if ((H_c[s_idx] - H_c2[0]) <1)
			break;
	}
	s_idx += skip_bin_idx;
	
	//debug
	//cout<<"Min: "<<s_idx<<","<<H[s_idx]<<","<<min_H<<endl;
	//ROS_INFO("smallest H %d, larget H %d", min_H, max_H);

	// output generation, do speed check 
	// make sure v_scale is between 0 - 1.5 
	if (H[s_idx] - max_H < equal_block_thres)
	{
		float sumH;
		for(int i = 18 ; i <=30 ; i++)
		{
			sumH += H[i];
		}
		cout<< "sumH: "<< sumH << endl;
		
		if (max_H < clear_thres)
		{
			v_scale = 0.3;
			v_angle = 0;
		}
		else if (min_H > block_thres || sumH > 5000 ) 
		{
			// a case where a lot of obstacles in front, turn back
			v_scale = 1.0;
			v_angle = -M_PI;
		}
		else
		{
			v_scale = (1 - H[s_idx]/max_H) * 5;
			v_scale = v_scale>1.5? 1.5 : v_scale;
			v_scale = v_scale<0? 0 : v_scale;
			v_angle = (-120+s_idx*polarH_resolution)*M_PI/180.0f;
		}
		//debug
		cout<< "polarH_length:"<<polarH_length<<endl;
		//for(int i = 0 ; i < polarH_length ; i ++)
		//{
		//	cout<<"H"<< i << ":" <<H[i] <<" | ";	
		//}
		cout<< endl;
		cout<< "Min:" << s_idx <<" | "<< min_H<<endl;
		cout<< "Max:" << b_idx <<" | "<< H[b_idx]<<endl;
		cout<< "clear_thres: " << clear_thres<< endl;
		cout<< "block_thres: " << block_thres<< endl;
		cout<< "v_angle :"<<v_angle<<endl;
		cout<< "v_scale :"<<v_scale<<endl;
	}
}

int OpenRadar::RadarBreak()
{
	int break_cnt = 0, dis = 0;
	for (int i = 1; i < num_data; i++)
	{
		dis = abs(RadarRho[i] - RadarRho[i-1]);
		if (dis > break_thres)
		{
			RadarRho[i] = -1;
			break_cnt++;
			i++;
		}	
	}
	return break_cnt;
}

// call backfunction should be good enough
void OpenRadar::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//cout<< "callback!"<<endl;
	//TODO: data integrity check (probably dont need this because ROS should already handle this)
	if (receive_state == 0)
	{
		receive_state = 1;
	}
	else
		receive_state += 1;
	if (receive_state == 1)
	{
		num_data = (msg->angle_max-msg->angle_min)/msg->angle_increment-1;

		angle_increment = msg -> angle_increment;
		angle_min = msg -> angle_min;
		angle_max = msg -> angle_max;
		range_min = msg -> range_min * 1000;
		range_max = msg -> range_max * 1000;
		range_min = 0.20*1000; //20cm
		range_max = 1.5 *1000; //max 1.5m
		for (int i = 0 ; i<num_data;i++)
			ranges[0][i] = msg->ranges[i]*1000;
	}
	else
	{
		for (int i = 0 ; i<num_data;i++)
			ranges[receive_state-1][i] = msg->ranges[i]*1000;
	}
	
	if (receive_state == MAX_frame)
	{
		laser_proc();
		receive_state = 0;
		return;
	}

}

// display function should be good enough
void OpenRadar::DrawRadarData()
{
	int usualColor[15] = {16777215,255,128,65280,32768,
		      16711680,16711935,8421376,65535,32896 }; /*<usual color*/
	CvPoint pt1, pt2;

	cvZero(RadarImage);
	cvCircle(RadarImage, cvPoint(DisplayDx,DisplayDy),3, CV_RGB(0,255,255), -1, 8,0);
	int x,y;
	unsigned char * pPixel = 0;
	int colorIndex = 0, colorRGB;
	int R = 255, G = 0, B = 0;
    
	for (int i = 0; i < RadarDataCnt;i++)
	{  
		if (RadarRho[i] < 0)
		{
			
			//change color
			colorRGB = usualColor[colorIndex];
			R = colorRGB/65536;
			G = (colorRGB%65536)/256;
			B = colorRGB%256;
			colorIndex = (colorIndex + 1)%10;
			
		}
		else 
		{
			x = (int)(RadarRho[i]*cos(RadarTheta[i])/DisplayRatio) + DisplayDx;
			y = (int)(-RadarRho[i]*sin(RadarTheta[i])/DisplayRatio)+ DisplayDy;
	
			if (x >= 0 && x < RadarImageWdith && y >= 0 && y < RadarImageHeight)
			{
				pPixel = (unsigned char*)RadarImage->imageData + y*RadarImage->widthStep + 3*x;
				pPixel[0] = B;
				pPixel[1] = G;
				pPixel[2] = R;
			}
		}     
	}
	
	pt1.x = DisplayDx; pt1.y = DisplayDy;
	pt2.x = DisplayDx+line_length*v_scale*sin(v_angle + 0.5*M_PI); 
	pt2.y = DisplayDy+line_length*v_scale*cos(v_angle + 0.5*M_PI);
	cvLine(RadarImage, pt1, pt2, CV_RGB(255,255,255),2,8,0);

	float angle;
	int line_length;
	for (int i=0; i<polarH_length;i++)
	{
		angle = (-30+i*polarH_resolution)*M_PI/180;
		line_length = H[i]/10;
		pt2.x = DisplayDx+line_length*sin(angle); 
		pt2.y = DisplayDy+line_length*cos(angle);
		cvCircle(RadarImage, pt2, 2, CV_RGB(255,255,255),1,8,0);
	}

	////////////////////////////////////////////////////////////////////////////////////
	// mine
	////////////////////////////////////////////////////////////////////////////////////
	Mat binImg = Mat::zeros(RadarImageHeight,RadarImageWdith,CV_8UC1);
	vector< Point> centerRaw;
	centerRaw.clear();
	for (int i = 0; i < RadarDataCnt;i++)
	{  
		if (RadarRho[i] > 300)
		{
			x = (int)(RadarRho[i]*cos(RadarTheta[i])/DisplayRatio) + DisplayDx;
			y = (int)(-RadarRho[i]*sin(RadarTheta[i])/DisplayRatio)+ DisplayDy;
			//centerRaw.push_back(Point(x,y));
			//cout<<"P:" <<centerRaw[i].x<<","<<centerRaw[i].y<<endl;
			if (x >= 0 && x < RadarImageWdith && y >= 0 && y < RadarImageHeight)
			{
				 circle( binImg,Point(x,y),1,Scalar(255),-1);
			}
		}     
	}
	//imshow(" ",binImg);
	Mat element = getStructuringElement(MORPH_RECT, Size(1,2));
	Mat element2 = getStructuringElement(MORPH_RECT, Size(10,10));
	erode(binImg, binImg, element);
	morphologyEx(binImg, binImg, MORPH_OPEN, element);
	dilate(binImg, binImg, element2);
	morphologyEx(binImg, binImg, MORPH_CLOSE, element2);
	imshow("after",binImg);

	vector< vector<Point> > contours;	
	vector< vector<Point> > filterContours;	
	vector< Vec4i > hierarchy;	
	vector< Point2f> center;
	vector< float > radius;
	vector<Point2f> realPoint;
	

	findContours(binImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	center.resize(contours.size());
	radius.resize(contours.size());
	//realPoint.resize(contours.size());
	for(int i = 0; i< contours.size(); i++)
	{
		minEnclosingCircle(Mat(contours[i]),center[i],radius[i]);//对轮廓进行多变形逼近
		circle(binImg,center[i],650/DisplayRatio,Scalar(255),1); 
		cout<<"No."<<i<<" | P: "<< center[i].x<<","<<center[i].y<<endl;
		float realX = (center[i].x - DisplayDx) * DisplayRatio;
		float realY = (center[i].y - DisplayDy) * DisplayRatio;

		realPoint.push_back(Point2f(realX,realY));
		cout<<"No."<<i<<" | P: "<< realPoint[i].x<<","<<realPoint[i].y<<endl;
	}
	imshow("after2",binImg);
	// colar map
	Mat mapImg = Mat::zeros(RadarImageHeight,RadarImageWdith,CV_8UC3);
	circle(mapImg, Point(DisplayDx,DisplayDy),3, CV_RGB(255,255,255),-1);
	line(mapImg, Point(DisplayDx,DisplayDy), Point(DisplayDx+40,DisplayDy), Scalar(0,0,255),1);
	line(mapImg, Point(DisplayDx,DisplayDy), Point(DisplayDx,DisplayDy+40), Scalar(0,255,0),1);
	for(int i = 0; i< center.size(); i++)
	{
		circle(mapImg,center[i],650/DisplayRatio,Scalar(255,255,0),1,CV_AA); 
		circle(mapImg,center[i],100/DisplayRatio,Scalar(0,255,255),-1); 
	}
	imshow("Map",mapImg);
	////////////////////////////////////
	float freq = 50.0f;
	if(timeInit)
	{
		time_old = ros::Time::now();
		timeInit = false;
	}
	else 
	{
		float dt = (ros::Time::now() -time_old).toSec();
		cout<<"Freqence: "<<1/dt<<" Hz"<<endl;
		if( (ros::Time::now() -time_old).toSec() > (1/ freq))
		{
			time_old = ros::Time::now();
			ukftest::laserPoint msg;
			vector <float> xvec;
			vector <float> yvec;
			for(int i = 0 ; i < realPoint.size(); i++)
			{
				// cm
				xvec.push_back(realPoint[i].x/10.0f);
				yvec.push_back(realPoint[i].y/10.0f);
			}

			// msg
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "hokuyo_laser";
			msg.x =xvec;
			msg.y =yvec;
			if(realPoint.size() >0) msg.isBlocking = 1;
			else msg.isBlocking = 0;
			pub_xy.publish(msg);

			// msg
			ukftest::ukfData ukfmsg;
			ukfmsg.avoid.x = v_scale*sin(v_angle + 0.5*M_PI); 
			ukfmsg.avoid.y = v_scale*cos(v_angle + 0.5*M_PI); 
			ukfmsg.dt = dt;
			ukfmsg.isBlocking = 1;
			cout<< "xyz: "<<  ukfmsg.avoid.x <<"|"<<ukfmsg.avoid.y <<"|"<<ukfmsg.avoid.z <<endl;
			cout<< "isBlocking:" <<  ukfmsg.isBlocking<< endl;
			pub.publish(ukfmsg);
		}
	}	
	

}













