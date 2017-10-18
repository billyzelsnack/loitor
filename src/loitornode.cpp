#include "ros/ros.h" 
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo.h"
//#include <tf/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

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
#include <thread>
#include <mutex>


//using namespace std;
using namespace cv;



std::mutex gMutex;


ros::Publisher pub_imu;
ros::Publisher pub_odom;

ros::Publisher pub_msgcam;
ros::Publisher pub_msgimu;

ros::Publisher pub_info_left;
ros::Publisher pub_info_right;


ros::Subscriber sub_msgcam;
ros::Subscriber sub_msgimu;

static loitor_ros::LoitorConfig sConfig;

//const string frame_id="/camera_link";

//const string frame_id="/stereo_camera";



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
				imu_msg.header.frame_id = "imu_link";
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
	std::unique_lock<std::mutex> lock(gMutex);	

	sConfig=config;

	ROS_INFO( "EG_MODE %d", config.EG_MODE );
	ROS_INFO( "man_exp %d", config.man_exp );	
	ROS_INFO( "man_gain %d", config.man_gain );


	visensor_set_auto_EG( config.EG_MODE );
	visensor_set_exposure( config.man_exp);
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
	std::unique_lock<std::mutex> lock(gMutex);	

	gServer->updateConfig(sConfig);
}

void callback_msgcam(const loitor_ros::LoitorCam::ConstPtr& msg)
{
	std::unique_lock<std::mutex> lock(gMutex);	
	
	ROS_INFO("set_exposure [%d]", msg->man_exp );
	visensor_set_exposure( msg->man_exp );
	sConfig.man_exp=msg->man_exp;
	updateConfig();
}

void callback_msgimu(const loitor_ros::LoitorIMU::ConstPtr& msg)
{
	std::unique_lock<std::mutex> lock(gMutex);	
	
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
		//CAMERAMODE_HIGHSPEED_STEREO_WVGA,
		EGMODE_MANUAL_MANUAL,
		//sConfig.man_exp, sConfig.man_gain,
		//5, 127,
		250, 127,
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



	
	/*
	Camera.fx: 448.803530
	Camera.fy: 448.803530
	Camera.cx: 366.972919
	Camera.cy: 242.598818
	*/

	int seq=0;

	sensor_msgs::CameraInfo info_left;
	info_left.header.seq=0;
	info_left.header.stamp=ros::Time::now();
	info_left.header.frame_id=std::string("left_link");
	info_left.height=480;	
	info_left.width=752;
	info_left.distortion_model="plumb_bob";
	info_left.D.push_back(-0.4082720348293939);
	info_left.D.push_back(0.15640744929855369);
	info_left.D.push_back(-0.00010483438752646738);
	info_left.D.push_back(0.00015318217681164253);
	info_left.D.push_back(0.0);

	info_left.K[0]=460.86210251750975; 
	info_left.K[1]=0.0;
	info_left.K[2]=349.31462624678466;
	info_left.K[3]=0.0;
	info_left.K[4]=460.5167045663297;
	info_left.K[5]=262.51727165666836;
	info_left.K[6]=0.0;
	info_left.K[7]=0.0;
	info_left.K[8]=1.0;
	
	info_left.R[0]=0.999974368893191;
	info_left.R[1]=-0.0012394558679958269;
	info_left.R[2]=0.007051617248225689;
	info_left.R[3]=0.0012260516558872633;
	info_left.R[4]=0.9999974341189767;
	info_left.R[5]=0.0019048760589784617;
	info_left.R[6]=-0.007053960164423911;
	info_left.R[7]=-0.0018961815878928763;
	info_left.R[8]=0.9999733227148535;

	info_left.P[0]=448.8035299304444;
	info_left.P[1]=0.0;
	info_left.P[2]=366.9729194641113;
	info_left.P[3]=0.0;
	info_left.P[4]=0.0;
	info_left.P[5]=448.8035299304444;
	info_left.P[6]=242.59881782531738;
	info_left.P[7]=0.0;
	info_left.P[8]=0.0;
	info_left.P[9]=0.0;
	info_left.P[10]=1.0;
	info_left.P[11]=0.0;
	
	info_left.binning_x=0;
	info_left.binning_y=0;
	info_left.roi.x_offset=0;
	info_left.roi.y_offset=0;
	info_left.roi.height=480;
	info_left.roi.width=752;
	info_left.roi.do_rectify=false;

	sensor_msgs::CameraInfo info_right;
	info_right.header.seq=0;
	info_right.header.stamp=ros::Time::now();
	info_right.header.frame_id=std::string("right_link");
	info_right.height=480;
	info_right.width=752;
	info_right.distortion_model="plumb_bob";
	
	info_right.D.push_back(-0.3963313031702811);
	info_right.D.push_back(0.13921261728908707);
	info_right.D.push_back(0.0002743786374625205);
	info_right.D.push_back(0.0008348992999562335);
	info_right.D.push_back(0.0);

	info_right.K[0]=459.0873954684523;
	info_right.K[1]=0.0;
	info_right.K[2]=372.92769732374114;
	info_right.K[3]=0.0;
	info_right.K[4]=458.8076221980105;
	info_right.K[5]=223.88525060354567;
	info_right.K[6]=0.0;
	info_right.K[7]=0.0;
	info_right.K[8]=1.0;
	
	info_right.R[0]=0.9999526729468252;
	info_right.R[1]=0.004591964561965642;
	info_right.R[2]=-0.00857704657567081;
	info_right.R[3]=-0.004608257490109048;
	info_right.R[4]=0.9999876131996607;
	info_right.R[5]=-0.0018808004016310775;
	info_right.R[6]=0.008568303764714956;
	info_right.R[7]=0.0019202366280157976;
	info_right.R[8]=0.9999614476878037;
	
	info_right.P[0]=448.8035299304444;
	info_right.P[1]=0.0;
	info_right.P[2]=366.9729194641113;
	info_right.P[3]=-45.369437121756086;
	info_right.P[4]=0.0;
	info_right.P[5]=448.8035299304444;
	info_right.P[6]=242.59881782531738;
	info_right.P[7]=0.0;
	info_right.P[8]=0.0;
	info_right.P[9]=0.0;
	info_right.P[10]=1.0;
	info_right.P[11]=0.0;
	
	info_right.binning_x=0;
	info_right.binning_y=0;
	info_right.roi.x_offset=0;
	info_right.roi.y_offset=0;
	info_right.roi.height=480;
	info_right.roi.width=752;
	info_right.roi.do_rectify=false;
	


	pub_msgcam = nh.advertise<loitor_ros::LoitorCam>("/get_cam", 1 );
	sub_msgcam = nh.subscribe("/loitor_node/set_cam", 1, callback_msgcam );

	pub_msgimu = nh.advertise<loitor_ros::LoitorIMU>("/get_imu", 1 );
	sub_msgimu = nh.subscribe("/loitor_node/set_imu", 1, callback_msgimu );

	pub_info_left=nh.advertise<sensor_msgs::CameraInfo>("/left/camera_info", 1 );
	//pub_info_left.publish(info_left);

	pub_info_right=nh.advertise<sensor_msgs::CameraInfo>("/right/camera_info", 1 );
	//pub_info_right.publish(info_right);

	// imu publisher
	pub_imu = nh.advertise<sensor_msgs::Imu>("/imu", 200);

	//pub_odom = nh.advertise<nav_msgs::Odometry>("/stereo_camera/odom", 1);
 
	// publish 到这两个 topic
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub1 = it.advertise("/left/image_raw", 1 );
	sensor_msgs::ImagePtr msg1;

	image_transport::ImageTransport it1(nh);
	image_transport::Publisher pub2 = it1.advertise("/right/image_raw", 1 );
	sensor_msgs::ImagePtr msg2;

	//ros::Rate loop_rate(500);//(int)hardware_fps);

	int static_ct=0;

	timeval img_time_test,img_time_offset;
	img_time_test.tv_usec=0;
	img_time_test.tv_sec=0;
	img_time_offset.tv_usec=50021;
	img_time_offset.tv_sec=0;

	/*
	tf2_ros::TransformBroadcaster br;
	tf2::Transform cltrans;
	cltrans.setIdentity();
	cltrans.setOrigin( tf2::Vector3( 0, 0, 0 ) );
	cltrans.setRotation( tf2::Quaternion( -0.5, 0.5, -0.5, 0.5) );			
	geometry_msgs::TransformStamped camera_link;
	camera_link.header.stamp=ros::Time::now();//msg_time;
	camera_link.header.frame_id=std::string("/base_link");
	camera_link.child_frame_id=std::string("/stereo_camera");
	camera_link.transform = tf2::toMsg(cltrans);
	*/

 
	while( ros::ok() )
	{
		if( visensor_get_cam_selection_mode()!=0 ){ continue; }
		if( !visensor_is_left_img_new() || !visensor_is_right_img_new() ){ continue; }

		bool validLeft=visensor_get_left_latest_img(img_left.data,&left_stamp,NULL);
		bool validRight=visensor_get_right_latest_img(img_right.data,&right_stamp,NULL);

		cv_bridge::CvImage t_left=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_left);
		cv_bridge::CvImage t_right=cv_bridge::CvImage(std_msgs::Header(), "mono8", img_right);


		ros::Time msg_time;
		msg_time.sec=left_stamp;
		msg_time.nsec=1000*left_stamp;

		msg1 = t_left.toImageMsg();
		msg1->header.frame_id=std::string("left_link");
		msg1->header.stamp = msg_time;
		msg1->header.seq=seq;
			
		msg2 = t_right.toImageMsg();
		msg2->header.frame_id=std::string("right_link");
		msg2->header.stamp = msg_time;
		msg2->header.seq=seq;


				


		info_left.header.stamp=msg_time;
		info_left.header.seq=seq;
		info_right.header.stamp=msg_time;
		info_right.header.seq=seq;


		std::unique_lock<std::mutex> lock(gMutex);				
		pub1.publish(msg1);
		pub_info_left.publish(info_left);
		pub2.publish(msg2);
		pub_info_right.publish(info_right);
		lock.unlock();

		/*
		loitor_ros::LoitorCam msgcam;
		msgcam.man_exp=visensor_get_exposure();
		msgcam.man_gain=visensor_get_gain();
		pub_msgcam.publish(msgcam);
		*/
			
		//loitor_ros::LoitorIMU msgimu;
		//pub_msgimu.publish(msgimu);

		seq++;
			
		// 显示时间戳
		//cout<<"left_time : "<<left_stamp.tv_usec<<endl;
		//cout<<"right_time : "<<right_stamp.tv_usec<<endl<<endl;

		
		ros::spinOnce(); 

		//loop_rate.sleep(); 
		
	}

	/*
	visensor_Close_IMU_viewer=true;
	if(imu_data_thread !=0)
	{
		pthread_join(imu_data_thread,NULL);
	}
	*/

	std::cout<<std::endl<<"shutting-down Cameras"<<std::endl;

	/* close cameras */
	visensor_Close_Cameras();

	//visensor_Close_IMU();
	
	return 0;
}

