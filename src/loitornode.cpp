#include "ros/ros.h" 
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"

#include <dynamic_reconfigure/server.h>
#include <loitor_ros/LoitorConfig.h>
#include <loitor_ros/LoitorCam.h>
#include <loitor_ros/LoitorIMU.h>

#include <cv.h>
#include <highgui.h>
#include "cxcore.hpp"
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loitorcam.h"
#include "loitorimu.h"


#include <sstream>


using namespace std;
using namespace cv;


ros::Publisher pub_imu;

ros::Publisher pub_msgcam;
ros::Publisher pub_msgimu;

ros::Subscriber sub_msgcam;
ros::Subscriber sub_msgimu;

static loitor_ros::LoitorConfig sConfig;



int imufd=-1;

/*
*  用于构造cv::Mat 的左右眼图像
*/
cv::Mat img_left;
cv::Mat img_right;

/*
*  当前左右图像的时间戳
*/
//timeval 
double left_stamp,right_stamp;

/*
*  imu viewer
*/
bool visensor_Close_IMU_viewer=false;
bool imu_start_transfer=false;


void* imu_data_stream(void *)
{
	int counter=0;
	imu_start_transfer=false;


	while((!visensor_Close_IMU_viewer)&&!imu_start_transfer){ usleep(1000); }
	while(!visensor_Close_IMU_viewer)
	{
		if(visensor_imu_have_fresh_data())
           	{


			visensor_imudata imudata;
			if(!visensor_get_imudata_latest(&imudata))
			{
				sensor_msgs::Imu imu_msg;
				imu_msg.header.frame_id = "/imu";
				ros::Time imu_time;
				imu_time.sec=imudata.timestamp;
				imu_time.nsec=1000*imudata.timestamp;
				imu_msg.header.stamp = ros::Time(imudata.timestamp);
				imu_msg.header.seq=0;

				imu_msg.linear_acceleration.x=imudata.ax;
				imu_msg.linear_acceleration.y=imudata.ay;
				imu_msg.linear_acceleration.z=imudata.az;
				imu_msg.angular_velocity.x=3.1415926f*imudata.rx/180.0f;
				imu_msg.angular_velocity.y=3.1415926f*imudata.ry/180.0f;
				imu_msg.angular_velocity.z=3.1415926f*imudata.rz/180.0f;
				imu_msg.orientation.w=imudata.qw;
				imu_msg.orientation.x=imudata.qx;
				imu_msg.orientation.y=imudata.qy;
				imu_msg.orientation.z=imudata.qz;

				pub_imu.publish(imu_msg);
			}
			

		}
		usleep(10);
	}
	pthread_exit(NULL);
}

//int visensor_reset();
void visensor_swap();

 
void callback(loitor_ros::LoitorConfig& config, uint32_t level) 
{
	sConfig=config;

	ROS_INFO( "EG_MODE %d", config.EG_MODE );
	visensor_set_auto_EG( config.EG_MODE );

	ROS_INFO( "man_exp %d", config.man_exp );
	visensor_set_exposure( config.man_exp);

	ROS_INFO( "man_gain %d", config.man_gain );
	visensor_set_gain( config.man_gain);

	//visensor_swap();
	//gSwap=!gSwap;


	/*
	ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
			  config.int_param, config.double_param, 
			  config.str_param.c_str(), 
			  config.bool_param?"True":"False", 
			  config.size);
			  */
}

dynamic_reconfigure::Server<loitor_ros::LoitorConfig>* gServer=NULL;

boost::recursive_mutex dynamic_reconfigure_mutex_;

void updateConfig()
{
	boost::recursive_mutex::scoped_lock dyn_reconf_lock(dynamic_reconfigure_mutex_);	
	gServer->updateConfig(sConfig);
	dyn_reconf_lock.unlock();
}

void callback_msgcam(const loitor_ros::LoitorCam::ConstPtr& msg)
{
	ROS_INFO("set_exposure [%d]", msg->man_exp );
	visensor_set_exposure( msg->man_exp );
	sConfig.man_exp=msg->man_exp;
	updateConfig();
}

void callback_msgimu(const loitor_ros::LoitorIMU::ConstPtr& msg)
{
	//ROS_INFO("set_gain [%d]", msg->data );
	//visensor_set_gain( msg->data );
	//sConfig.man_gain=msg->data;
	//updateConfig();
}

int main(int argc, char **argv)
{ 
	/************************ Start Cameras ************************/
	//if(argv[1])
	//visensor_load_settings(argv[1]);
	//else 

	ros::init(argc, argv, "loitor_node");

	dynamic_reconfigure::Server<loitor_ros::LoitorConfig> server(dynamic_reconfigure_mutex_);
	dynamic_reconfigure::Server<loitor_ros::LoitorConfig>::CallbackType f;
	gServer=&server;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
  
	
	ros::NodeHandle nh;


	

		
	//-- package relative path. seems pretty hacky.	
	//visensor_load_settings("../../../src/loitor-ros/Loitor_VISensor_Setups.txt");

	// 手动设置相机参数
	//set_current_mode(5);
	//set_auto_EG(0);
	//set_exposure(50);
	//set_gain(200);
	//set_visensor_cam_selection_mode(2);
	//set_resolution(false);
	//set_fps_mode(true);
	// 保存相机参数到原配置文件
	//save_current_settings();
	

	int r = visensor_Start_Cameras(
		CAMERAMODE_NORMAL_STEREO_WVGA,
		EGMODE_MANUAL_MANUAL,
		sConfig.man_exp, sConfig.man_gain,
		300, 5, 58,
		300, 5, 200,
		"/dev/ttyUSB0", 5,
		52.0, 32.0, -243.0 );
	if(r<0)
	{
		printf("Opening cameras failed...\r\n");
		return r;
	}
	// 创建用来接收camera数据的图像
	//if(!visensor_resolution_status)
	if(!visensor_get_resolution())
	{
		img_left.create(cv::Size(640,480),CV_8U);
		img_right.create(cv::Size(640,480),CV_8U);
		img_left.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
		img_right.data=new unsigned char[IMG_WIDTH_VGA*IMG_HEIGHT_VGA];
	}
	else
	{
		img_left.create(cv::Size(752,480),CV_8U);
		img_right.create(cv::Size(752,480),CV_8U);
		img_left.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
		img_right.data=new unsigned char[IMG_WIDTH_WVGA*IMG_HEIGHT_WVGA];
	}
	float hardware_fps=visensor_get_hardware_fps();
	
	/*
	//************************** Start IMU **************************
	imufd=visensor_Start_IMU();
	if(imufd<0)
	{
		printf("open_port error...\r\n");
		return 0;
	}
	printf("open_port success...\r\n");
	usleep(100000);
	//************************ ************ ************************

	//Create imu_data_stream thread
	pthread_t imu_data_thread;
	int temp;
	if(temp = pthread_create(&imu_data_thread, NULL, imu_data_stream, NULL))
	printf("Failed to create thread imu_data_stream\r\n");
	*/

	pub_msgcam = nh.advertise<loitor_ros::LoitorCam>("/loitor_node/get_cam", 1 );
	sub_msgcam = nh.subscribe("/loitor_node/set_cam", 1, callback_msgcam );

	pub_msgimu = nh.advertise<loitor_ros::LoitorIMU>("/loitor_node/get_imu", 1 );
	sub_msgimu = nh.subscribe("/loitor_node/set_imu", 1, callback_msgimu );
	
	// imu publisher
	pub_imu = nh.advertise<sensor_msgs::Imu>("/loitor_node/imu", 200);
 
	// publish 到这两个 topic
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/camera/left/image_raw", 1);
	sensor_msgs::ImagePtr msg;

	image_transport::ImageTransport it1(nh);
	image_transport::Publisher pub1 = it1.advertise("/camera/right/image_raw", 1);
	sensor_msgs::ImagePtr msg1;

	// 使用camera硬件帧率设置发布频率
	ros::Rate loop_rate((int)hardware_fps);

	int static_ct=0;

	timeval img_time_test,img_time_offset;
	img_time_test.tv_usec=0;
	img_time_test.tv_sec=0;
	img_time_offset.tv_usec=50021;
	img_time_offset.tv_sec=0;

	while (ros::ok())
	{
		imu_start_transfer=true;

		//cout<<"visensor_get_hardware_fps() ==== "<<visensor_get_hardware_fps()<<endl;

		if(visensor_get_cam_selection_mode()==0)
		{

			//visensor_imudata paired_imu=visensor_get_stereoImg((char *)img_left.data,(char *)img_right.data,left_stamp,right_stamp);		
			if( !visensor_get_left_latest_img(img_left.data,&left_stamp,NULL) &&
				!visensor_get_right_latest_img(img_right.data,&right_stamp,NULL) )
			{


			// 显示同步数据的时间戳（单位微秒）
			//cout<<"left_time : "<<left_stamp.tv_usec<<endl;
			//cout<<"right_time : "<<right_stamp.tv_usec<<endl;
			//cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;
			//cout<<"visensor_get_hardware_fps() ==== "<<1.0f/visensor_get_hardware_fps()<<endl;

			cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);
			cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);

			// 加时间戳(right_time=left_time)
			ros::Time msg_time;
			msg_time.sec=left_stamp;
			msg_time.nsec=1000*left_stamp;
			t_left.header.stamp = msg_time;
			
			ros::Time msg1_time;
			msg1_time.sec=left_stamp;
			msg1_time.nsec=1000*left_stamp;
			t_right.header.stamp = msg1_time;
			t_right.header.seq=0;
			t_left.header.seq=0;

			msg = t_left.toImageMsg();
			msg1 = t_right.toImageMsg();

			static_ct++;
			{
				pub.publish(msg);
				pub1.publish(msg1);
				
				loitor_ros::LoitorCam msgcam;
				msgcam.man_exp=visensor_get_exposure();
				msgcam.man_gain=visensor_get_gain();
				pub_msgcam.publish(msgcam);

				//loitor_ros::LoitorIMU msgimu;
				//pub_msgimu.publish(msgimu);

				static_ct=0;
			}
			
			// 显示时间戳
			//cout<<"left_time : "<<left_stamp.tv_usec<<endl;
			//cout<<"right_time : "<<right_stamp.tv_usec<<endl<<endl;

			}
		}
		else if(visensor_get_cam_selection_mode()==1)
		{
			//visensor_imudata paired_imu=visensor_get_rightImg((char *)img_right.data,right_stamp);
			
			if( !visensor_get_right_latest_img(img_right.data,&right_stamp,NULL) )
			{

			// 显示同步数据的时间戳（单位微秒）
			//cout<<"right_time : "<<right_stamp.tv_usec<<endl;
			//cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

			cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);

			// 加时间戳
			ros::Time msg1_time;
			msg1_time.sec=right_stamp;
			msg1_time.nsec=1000*right_stamp;
			t_right.header.stamp = msg1_time;
			t_right.header.seq=0;
			
			msg1 = t_right.toImageMsg();
			
			pub1.publish(msg1);
			}
		}
		else if(visensor_get_cam_selection_mode()==2)
		{
			//visensor_imudata paired_imu=visensor_get_leftImg((char *)img_left.data,left_stamp);
			if( !visensor_get_left_latest_img(img_left.data,&left_stamp,NULL) )
			{

			// 显示同步数据的时间戳（单位微秒）
			//cout<<"left_time : "<<left_stamp.tv_usec<<endl;
			//cout<<"paired_imu time ===== "<<paired_imu.system_time.tv_usec<<endl<<endl;

			cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);

			// 加时间戳
			ros::Time msg_time;
			msg_time.sec=left_stamp;
			msg_time.nsec=1000*left_stamp;
			t_left.header.stamp = msg_time;
			t_left.header.seq=0;


			msg = t_left.toImageMsg();

			
			static_ct++;
			if(static_ct>=5)
			{
				pub.publish(msg);
				static_ct=0;
			}
			}
		}

		ros::spinOnce(); 

		loop_rate.sleep(); 
		
	}

	/*
	visensor_Close_IMU_viewer=true;
	if(imu_data_thread !=0)
	{
		pthread_join(imu_data_thread,NULL);
	}
	*/

	cout<<endl<<"shutting-down Cameras"<<endl;

	/* close cameras */
	visensor_Close_Cameras();

	//visensor_Close_IMU();
	
	return 0;
}











